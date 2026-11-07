# k1lidar/pa.py - taratura Pressure Advance col LiDAR.
#
# Sequenza (identica a quella di fabbrica catturata il 2026-07-15, validata a luglio
# e il 2026-11-07): livellamento -> piatto in temperatura -> scan del piatto NUDO con
# l'ugello ancora a g28_ext_temp (non cola) -> ugello a temperatura -> pattern ->
# raffreddamento "alla Creality" -> scan delle linee -> analisi.
#
# Analizzatore: algo/pa_phys.py (nostro). Misura quanto e' PIENO lo spigolo di ogni V
# del pattern rispetto al tratto dritto: con PA basso lo spigolo fa grumo, con PA
# alto si svuota; il PA scelto e' quello a cui e' pieno quanto il dritto. Sostituisce
# l'analizzatore di serie di Creality, che si e' dimostrato inaffidabile: sullo
# stesso filamento dava 0.054 o 0.0375 a seconda di dove cadeva il valore nel pattern.
#
# Rispetto all'originale:
#  - pattern generato da noi (pa_pattern.py): stessa geometria, valori a scelta per
#    materiale, spostamenti a Z5;
#  - scansione lineare sincronizzata con klipper (session.linear_sweep) e catture
#    dense (una ogni ~0.6 mm);
#  - risultato continuo con intervallo di confidenza e avvisi espliciti;
#  - passata FINE opzionale (secondo pattern 30 mm piu' a destra, passo 0.001);
#  - risultato salvato per filamento (store.py) e riapplicato alle stampe successive.
import os, sys, time

from .session import (PC, ROOT, kl, log, echo, creality_cooldown, park, heaters_off, laser_up,
                      linear_sweep,
                      self_cal, apply_exposure, acquire_lock, release_lock, LOCK, archive_run,
                      factory_level, load_calib, save_calib)
from .i18n import T
from . import pa_pattern as P

ALGO_DIRS = [os.path.join(ROOT, "algo"), "/usr/data/lidar_algo"]
SCAN_Y0, SCAN_Y1 = 27.2, 221.2
# Intervalli per materiale (start, step) dei 32 nodi. Il PA atteso deve cadere a
# META' pattern: le prime righe (davanti) si stampano peggio (ripartenza dopo lo
# spostamento) e un ginocchio al bordo non e' misurabile. Run 2026-11-07: col range
# di fabbrica 0.020..0.082 il PLA cadeva nella riga 1, la peggio stampata.
#   PLA/ABS/ASA: 0.006..0.0525 (atteso ~0.02-0.035 su estrusore diretto)
#   PETG:        0.016..0.078  (atteso ~0.035-0.06)
#   TPU:         0.050..0.205  sperimentale, da validare
# Altri: quello di fabbrica 0.020..0.082 a passi di 0.002.
MATERIAL_SWEEP = {"PLA": (0.006, 0.0015), "ABS": (0.006, 0.0015), "ASA": (0.006, 0.0015),
                  "PETG": (0.016, 0.002), "TPU": (0.050, 0.005)}


def sweep_for(material, start=None, step=None):
    s0, st = MATERIAL_SWEEP.get((material or "").upper(), (P.FACTORY_START, P.FACTORY_STEP))
    return P.grid(start if start is not None else s0, step if step is not None else st)


# ------------------------------------------------------------------------- scan --
def scan_sweep(L, save_path, label, key_prefix, x=P.SCAN_X, y0=SCAN_Y0, y1=SCAN_Y1,
               n_frames=170, mode="cont", speed=400):
    """Scan DETERMINISTICO, un frame per chiave {key_prefix+"0": [...], ...}: l'analizzatore
    usa l'INDICE del frame come posizione Y, quindi anche i frame vuoti restano.
    - "cont" (default, come l'originale): passata lineare continua, Y di ogni frame
      dalla traiettoria vera di klipper (session.linear_sweep);
    - "step": passi a testa ferma (~0.44 s/passo, solo diagnostica)."""
    import json
    step = (y1 - y0) / (n_frames - 1)
    log("  [%s] X%.1f Y%.1f->%.1f, %d frame (%.3fmm/frame, %s)" % (label, x, y0, y1, n_frames, step, mode))
    kl("G90\nG0 Z3 F600\nG0 X%.2f Y%.2f F12000\nM400" % (x, y0))
    L.open_scan()

    def grab(retry=3):
        for _ in range(retry):
            pc = L.grab()
            if pc:
                return pc
            time.sleep(0.03)
        return []

    frames = []; empty = 0; sweep = None
    try:
        if mode == "step":
            for i in range(n_frames):
                kl("G0 X%.2f Y%.3f F9000\nM400" % (x, y0 + i * step))
                pc = grab()
                empty += not pc
                frames.append(pc)
        else:
            # passata lineare continua sincronizzata con klipper (vedi session.linear_sweep)
            frames, sweep = linear_sweep(L, x, y0, y1, n_frames, speed, label)
            empty = sweep["empty"]
            dense = sweep.pop("dense", None)
            if dense:      # catture piene con Y vera, accanto al .temp (per pa_phys)
                with open(save_path + ".dense.json", "w") as fh:
                    json.dump({"x_nozzle": x, "y0": y0, "y1": y1, "y": [d[0] for d in dense],
                               "frames": [d[1] for d in dense]}, fh)
    finally:
        L.close_scan()
    data = {"%s%d" % (key_prefix, i): fr for i, fr in enumerate(frames)}
    npts = sum(len(fr) for fr in frames)
    with open(save_path, "w") as f:
        json.dump(data, f)
    log("  [%s] %d frame (%d vuoti), %d punti -> %s" % (label, len(frames), empty, npts, os.path.basename(save_path)))
    if npts == 0:
        raise RuntimeError("scan '%s' VUOTO: laser non attivo, analisi impossibile" % label)
    out = {"frames": len(frames), "empty": empty, "points": npts}
    if sweep:
        out["sweep"] = sweep
    return out


# ---------------------------------------------------------------------- analisi --
def _import_algo():
    for d in ALGO_DIRS:
        if os.path.isdir(d) and d not in sys.path:
            sys.path.insert(0, d)


def analyze(tablef, linef, values=None):
    """Analizza tabella (piatto nudo) e linee con algo/pa_phys.py; usa le catture dense
    (<file>.dense.json) se ci sono. `values` = i 32 PA del pattern (None = fabbrica).
    Ritorna dict: pa, pa_lo, pa_hi (intervallo 80%), confidence {level, warnings},
    dettagli; oppure {"pa": None, "error": ...}."""
    _import_algo()
    import pa_phys
    td, ld = tablef + ".dense.json", linef + ".dense.json"
    try:
        if os.path.isfile(td) and os.path.isfile(ld):
            r = pa_phys.analyze_dense(td, ld, values); src = "catture dense"
        else:
            r = pa_phys.analyze(tablef, linef, values); src = "170 frame"
    except Exception as e:
        return {"pa": None, "error": "analisi fallita: %s" % e}
    for line in pa_phys.report(r).splitlines():
        log("    " + line)
    if r.get("pa") is None:
        return {"pa": None, "error": r.get("error") or "analisi fallita"}
    warns = r.get("warnings") or []
    if any("ESTRAPOLATO" in w for w in warns):
        # il valore cade fuori dai nodi stampati: non e' una misura, e' una stima
        # tirata fuori dal pattern. Meglio fallire con un'indicazione chiara.
        lo, hi = r["measured"]
        side = "sotto" if r["pa"] < lo else "sopra"
        return {"pa": None, "error": "PA fuori dal range del pattern (%s %.4f-%.4f, stima %.4f): "
                "ripeti con START=%.3f" % (side, lo, hi, r["pa"],
                                           max(0.0, r["pa"] - 16 * (hi - lo) / 23.0))}
    return {"pa": round(r["pa"], 4), "pa_lo": r["pa_lo"], "pa_hi": r["pa_hi"],
            "source": "pa_phys (%s)" % src, "values": list(values) if values else None,
            "confidence": {"level": "low" if warns else "high", "warnings": warns},
            "detail": {k: r.get(k) for k in ("slope", "slope_t", "node_bias", "sigma", "n",
                                             "measured")}}


def print_pattern(values, hot, bed, x_offset=0.0, prime_y=2.0):
    """Stampa il pattern generato (a blocchi: una gcode/script enorme bloccherebbe klippy)."""
    lines = P.generate(values, hot, bed, x_offset=x_offset, prime_y=prime_y).splitlines()
    for i in range(0, len(lines), 60):
        kl("\n".join(lines[i:i + 60]))
    kl("M400")


# --------------------------------------------------------------------------- run --
class Opts(object):
    """Parametri di una taratura (gli stessi delle opzioni di lidar_pa.py)."""
    def __init__(self, **kw):
        self.material = "PLA"; self.fil = "PLA"; self.hot = 220; self.bed = 60
        self.frames = 170; self.scan_mode = "cont"; self.scan_speed = 400
        self.no_selfcal = False; self.factory_level = True; self.level = False
        self.skip_print = False; self.apply = True; self.fine = False
        self.start = None; self.step = None; self.exposure = None; self.backend = "driver"
        self.remember = False
        self.name = None          # nome del profilo filamento dello slicer (per gli elenchi)
        self.__dict__.update(kw)


def run(o, L=None, prepared=False):
    """Esegue la taratura. Con `L` (backend laser gia' acceso) e `prepared=True` la
    usa dentro un job piu' grande (avvio stampa): niente lock, livellamento, self-cal ne'
    spegnimento finale, che restano a carico del chiamante. Ritorna il dict di analyze
    (+ "fine": risultato della passata fine, se fatta)."""
    own = L is None
    tab_a, lin_a = PC + "/scan_flow_table_point.temp", PC + "/scan_flow_line_point.temp"
    tab_b, lin_b = PC + "/scan_flow_table_fine.temp", PC + "/scan_flow_line_fine.temp"
    values = sweep_for(o.material, o.start, o.step)
    res = {"pa": None}; stats = {}; vexp = None; fine = None
    if own and not acquire_lock():
        raise RuntimeError(T("busy") + " (%s)" % LOCK)
    echo(T("pa_start", fil=o.fil, hot=o.hot, bed=o.bed))
    try:
        if not prepared:
            echo(T("level"))
            if o.factory_level:
                factory_level(o.hot, o.bed)
            elif o.level:
                kl("G29 PROBE_COUNT=4,4")
            else:
                kl("CX_ROUGH_G28 EXTRUDER_TEMP=%d BED_TEMP=%d" % (max(o.hot, 180), o.bed))
                kl("CX_NOZZLE_CLEAR")
                kl("ACCURATE_G28")
        if not o.skip_print:
            # NON scaldare ora l'ugello a o.hot: lo scan del piatto nudo si fa con
            # l'ugello a g28_ext_temp, altrimenti il PLA cola sul piano di riferimento.
            kl("M140 S%d" % o.bed)
        if own:
            echo(T("laser"))
            L = laser_up(o.backend)
            if L is None:
                raise RuntimeError("laser non disponibile (ne' driver ne' middleware)")
            vexp = apply_exposure(L, o, load_calib(o.fil))
            if not o.no_selfcal:
                self_cal(L)

        if not o.skip_print:
            echo(T("scan_table"))
            kl("M190 S%d" % o.bed)
            stats["tabella"] = scan_sweep(L, tab_a, "tabella", "table", n_frames=o.frames,
                                          mode=o.scan_mode, speed=o.scan_speed)
            if o.fine:   # il piatto nudo sotto il secondo pattern si acquisisce ORA
                stats["tabella_fine"] = scan_sweep(L, tab_b, "tabella fine", "table",
                                                   x=P.SCAN_X + P.FINE_OFFSET_X, n_frames=o.frames,
                                                   mode=o.scan_mode, speed=o.scan_speed)
            echo(T("print_pattern"))
            kl("M109 S%d" % o.hot)
            print_pattern(values, o.hot, o.bed)
            echo(T("cooldown"))
            creality_cooldown()

        echo(T("scan_lines"))
        stats["linee"] = scan_sweep(L, lin_a, "linee", "scan", n_frames=o.frames,
                                    mode=o.scan_mode, speed=o.scan_speed)
        res = analyze(tab_a, lin_a, values)
        if res.get("pa") is None:
            raise RuntimeError(res.get("error", "analisi fallita"))
        log("    PA = %.4f (80%%: %.4f-%.4f)  [%s]  confidenza %s"
            % (res["pa"], res["pa_lo"], res["pa_hi"], res["source"], res["confidence"]["level"]))

        if o.fine and not o.skip_print:
            fvals = P.fine_grid(res["pa"])
            echo(T("pa_fine", pa=res["pa"]))
            kl("M109 S%d" % o.hot)
            print_pattern(fvals, o.hot, o.bed, x_offset=P.FINE_OFFSET_X, prime_y=P.FINE_PRIME_Y)
            creality_cooldown()
            stats["linee_fine"] = scan_sweep(L, lin_b, "linee fine", "scan",
                                             x=P.SCAN_X + P.FINE_OFFSET_X, n_frames=o.frames,
                                             mode=o.scan_mode, speed=o.scan_speed)
            fine = analyze(tab_b, lin_b, fvals)
            log("    passata fine: %s  [%s]  confidenza %s"
                % (fine.get("pa"), fine.get("source"), (fine.get("confidence") or {}).get("level")))
        kl("G0 Z20 F600\nM400")
    finally:
        if own:
            if L is not None:
                L.shutdown()
            heaters_off()
            park(z=20)        # niente M84: il display non deve chiedere di rifare l'home
            release_lock()
        archive_run({
            "stamp": time.strftime("%Y%m%d_%H%M%S"), "tag": "pa_" + o.fil,
            "kind": "pressure_advance", "material": o.material, "filament": o.fil,
            "hotend": o.hot, "bed": o.bed, "backend": getattr(L, "name", o.backend),
            "exposure": vexp, "frames": o.frames, "scan_mode": o.scan_mode,
            "scan_speed": o.scan_speed, "factory_level": bool(o.factory_level),
            "values": values, "scan_stats": stats, "result": res, "fine": fine,
        }, [p for base in ([tab_a, lin_a] + ([tab_b, lin_b] if o.fine else []))
            for p in (base, base + ".dense.json") if os.path.isfile(p)])

    final = fine if (fine and fine.get("pa") is not None) else res
    conf = (final.get("confidence") or {}).get("level", "?")
    echo(T("pa_result", pa="%.4f" % final["pa"], lo="%.4f" % final["pa_lo"],
           hi="%.4f" % final["pa_hi"], conf=T("conf_" + conf) if conf in ("low", "high") else conf))
    if conf == "low":
        for w in (final.get("confidence") or {}).get("warnings", []):
            echo("  - " + w)
        echo(T("pa_low_conf"))
    save_calib(o.fil, pa=round(final["pa"], 4), pa_confidence=final.get("confidence"),
               pa_range=[round(final["pa_lo"], 4), round(final["pa_hi"], 4)],
               pa_source=final.get("source"), hotend=o.hot, bed=o.bed, pa_fine=bool(fine),
               name=o.name, material=o.material,
               exposure=(o.exposure if (o.remember and o.exposure) else None))
    if o.apply:
        kl("SET_PRESSURE_ADVANCE ADVANCE=%.4f" % final["pa"])
        echo(T("pa_saved", pa="%.3f" % final["pa"], fil=o.fil))
    res["fine"] = fine
    res["final_pa"] = final["pa"]
    return res
