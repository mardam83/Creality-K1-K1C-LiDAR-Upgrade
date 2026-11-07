# k1lidar/job.py - orchestratore: cosa fare all'avvio di una stampa, e la taratura a pulsante.
#
#   python3 lidar_job.py print FILE.gcode --flow 1 --firstlayer 1 [--on-bad pause|cancel]
#   python3 lidar_job.py pa --nozzle 220 --bed 60 --filament "PLA arancione" [--fine]
#   python3 lidar_job.py pa --gcode FILE.gcode           # profilo e temperature dal gcode
#   python3 lidar_job.py set --gcode FILE.gcode --pa 0.027   (o --filament "..." --nozzle 220)
#   python3 lidar_job.py list
#
# Avvio stampa (come il flusso Creality, ma con un solo livellamento):
#   1. dal gcode: filamento, temperature, larghezza linea (gcode_meta);
#   2. PA: se il filamento e' gia' tarato si applica il valore salvato; altrimenti
#      (o se --flow 2) si tara stampando il pattern sul bordo sinistro;
#   3. primo strato (se attivo): tabella -> layer 1 -> scan -> C2 -> riprendi/pausa;
#   4. altrimenti si avvia la stampa normalmente (senza rifare il livellamento).
# Con la taratura attiva (--flow 1/2) il PA LiDAR sostituisce anche quello scritto nel
# gcode dallo slicer (lock_pa -> macro SET_PRESSURE_ADVANCE di k1lidar_hook.cfg).
import argparse, os, time
from types import SimpleNamespace

from .session import (MATS, GCODES, kl, kl_query, log, echo, heaters_off, park, laser_up, self_cal,
                      acquire_lock, release_lock, LOCK, factory_level, resolve_temps)
from .i18n import T
from . import gcode_meta as GM
from . import store
from . import pa as PA
from . import firstlayer as FL


def _hook_installed():
    """True se SDCARD_PRINT_FILE e' intercettato dal nostro hook (allora l'originale
    si chiama SDCARD_PRINT_FILE_BASE)."""
    try:
        st = kl_query({"configfile": ["settings"]}).get("configfile", {}).get("settings", {})
        return "gcode_macro sdcard_print_file" in st
    except Exception:
        return False


def _lock_installed():
    try:
        st = kl_query({"configfile": ["settings"]}).get("configfile", {}).get("settings", {})
        return "gcode_macro _lidar_pa_lock" in st
    except Exception:
        return False


def lock_pa(pa, armed=False):
    """Con LIDAR_FLOW attivo il PA LiDAR vince su quello del gcode: la macro
    SET_PRESSURE_ADVANCE (k1lidar_hook.cfg) sostituisce ADVANCE durante la stampa.
    armed=True: la stampa partira' dal primo strato (FIRST_FLOOR_PRINT via hook)."""
    if pa is None or not _lock_installed():
        return False
    kl("_LIDAR_PA_LOCK PA=%.4f ARMED=%d" % (pa, 1 if armed else 0))
    return True


def start_print(gpath, prepared=False):
    """Avvia la stampa standard. Con prepared=True START_PRINT salta il livellamento
    (gia' fatto in questo job)."""
    name = os.path.relpath(gpath, GCODES) if gpath.startswith(GCODES) else os.path.basename(gpath)
    if prepared:
        kl("PRINT_PREPARED")
    cmd = "SDCARD_PRINT_FILE_BASE" if _hook_installed() else "SDCARD_PRINT_FILE"
    echo(T("job_print", file=name))
    kl('%s FILENAME="%s"' % (cmd, name))


def _temps(material, hot, bed):
    """Temperature validate (limiti e min_extrude_temp) via resolve_temps."""
    ns = SimpleNamespace(material=material if material in MATS else "PLA", hotend=hot, bed=bed)
    h, b, _ = resolve_temps(ns)
    return h, b


def do_print(a):
    gpath = FL.resolve_gcode(a.file)
    meta = GM.read(gpath)
    material = (meta.get("filament_type") or a.material or "PLA").upper()
    if material not in MATS:
        material = a.material if a.material in MATS else "PLA"
    hot, bed = _temps(material, meta.get("nozzle_temp") or a.hotend or None,
                      meta.get("bed_temp") or a.bed or None)
    key = GM.filament_key(meta)
    name = meta.get("filament_name") or material
    legacy = store.normalize_key(name)
    saved = store.get_pa(key, legacy)
    log("== job stampa: %s | filamento %s (%s) | ugello %d / piatto %d | PA salvato %s"
        % (os.path.basename(gpath), name, key, hot, bed, saved))
    slicer_pa = meta.get("slicer_pa_enabled")
    if slicer_pa and not a.flow and a.pa_apply:
        echo(T("slicer_pa", pa=meta.get("slicer_pa")))

    need_pa = a.flow == 2 or (a.flow == 1 and saved is None)
    if saved is not None and not need_pa and (a.flow or a.pa_apply):
        kl("SET_PRESSURE_ADVANCE ADVANCE=%.4f" % saved)
        echo(T("pa_reuse", pa="%.3f" % saved, fil=name))

    def lock(pa, armed):
        # taratura PA attiva: il PA LiDAR sostituisce anche quello scritto nel gcode
        if a.flow and lock_pa(pa, armed) and slicer_pa:
            echo(T("slicer_pa_lock", pa=meta.get("slicer_pa"), our="%.4f" % pa))

    if not need_pa and not a.firstlayer:
        lock(saved, False)
        start_print(gpath)
        return {"pa": saved, "firstlayer": None}

    if not acquire_lock():
        raise RuntimeError(T("busy") + " (%s)" % LOCK)
    L = None; printing = False; result = {"pa": saved}
    try:
        echo(T("level"))
        factory_level(hot, bed)
        kl("M140 S%d" % bed)
        echo(T("laser"))
        L = laser_up(a.backend)
        if L is None:
            raise RuntimeError("laser non disponibile")
        self_cal(L)
        if need_pa:
            echo(T("pa_new", fil=name))
            # il PA si misura alla temperatura degli strati dopo il primo (quella della chiave)
            hot_pa = meta.get("nozzle_temp_other") or hot
            o = PA.Opts(material=material, fil=key, hot=hot_pa, bed=bed, backend=a.backend,
                        apply=True, no_selfcal=True, name=name)
            r = PA.run(o, L=L, prepared=True)
            result["pa"] = r.get("final_pa")
        if a.firstlayer:
            fa = FL.build_parser().parse_args([
                "--gcode", gpath, "--material", material, "--hotend", str(hot),
                "--bed", str(bed), "--filament", name, "--resume", "--on-bad", a.on_bad,
                "--no-selfcal", "--backend", a.backend])
            lock(result["pa"], True)
            printing = True          # da qui la stampa la gestisce firstlayer.run
            result["firstlayer"] = FL.run(fa, L=L, prepared=True)
        else:
            L.shutdown(); L = None
            lock(result["pa"], False)
            printing = True
            start_print(gpath, prepared=True)
        return result
    except BaseException as e:
        if not printing:
            heaters_off()
            park(z=20)
        echo(T("error", err=e))
        raise
    finally:
        if L is not None:
            L.shutdown()
        release_lock()


def _profile(a):
    """(nome profilo, materiale, ugello, piatto) da --gcode e/o dagli argomenti: gli
    argomenti espliciti vincono sul gcode."""
    meta = {}
    if getattr(a, "gcode", None):
        meta = GM.read(FL.resolve_gcode(a.gcode))
        log("    profilo dal gcode: %s (%s) ugello %s / piatto %s"
            % (meta.get("filament_name"), meta.get("filament_type"),
               meta.get("nozzle_temp_other"), meta.get("bed_temp")))
    material = (a.material or meta.get("filament_type") or "PLA").upper()
    if material not in MATS:
        material = "PLA"
    name = a.filament or meta.get("filament_name") or material
    nozzle = a.nozzle or meta.get("nozzle_temp_other")
    bed = getattr(a, "bed", None) or meta.get("bed_temp")
    return name, material, nozzle, bed


def do_pa(a):
    name, material, nozzle, bed = _profile(a)
    hot, bed = _temps(material, nozzle, bed)
    key = GM.filament_key(name=name, nozzle=hot)
    if not a.filament and not a.gcode:
        echo(T("pa_noname", key=key))
    o = PA.Opts(material=material, fil=key, hot=hot, bed=bed, backend=a.backend,
                fine=a.fine, start=a.start, step=a.step, apply=True, name=name)
    r = PA.run(o)
    print("\n>>> PA OTTIMALE (LiDAR): %s <<<" % ("%.4f" % r["final_pa"]).rstrip("0").rstrip("."))
    return r


def do_set(a):
    """Assegna a mano un PA a un profilo filamento (senza tarare)."""
    if a.pa is None or not 0.0 <= a.pa <= 1.0:
        raise RuntimeError("PA mancante o fuori dai limiti (0-1)")
    name, material, nozzle, _bed = _profile(a)
    if not nozzle:
        raise RuntimeError("serve la temperatura dell'ugello (NOZZLE=) o un GCODE=")
    key = GM.filament_key(name=name, nozzle=int(nozzle))
    store.save_calib(key, pa=round(a.pa, 4), pa_source="manual", hotend=int(nozzle),
                     name=name, material=material)
    echo(T("pa_set", pa="%.4f" % a.pa, fil=name, hot=int(nozzle)))
    return {"pa": a.pa, "key": key}


def do_list(_a):
    rows = store.list_filaments()
    if not rows:
        echo(T("list_empty"))
        return {}
    echo(T("list_head"))
    for key, v in rows:
        rng = v.get("pa_range")
        conf = (v.get("pa_confidence") or {}).get("level")
        if conf in ("low", "high"):
            how = " " + T("conf_" + conf)
        elif v.get("pa_source") == "manual":
            how = " " + T("manual")
        else:
            how = ""
        echo("  %s @ %s C: PA %s%s%s  (%s)" % (
            v.get("name") or key, v.get("hotend", "?"), v.get("pa", "-"),
            (" [%s-%s]" % tuple(rng)) if rng else "", how, v.get("updated", "")))
    return {"n": len(rows)}


def build_parser():
    ap = argparse.ArgumentParser(description="Job LiDAR K1: avvio stampa e taratura PA")
    sub = ap.add_subparsers(dest="cmd")
    p = sub.add_parser("print", help="avvio di una stampa con taratura flusso / controllo primo strato")
    p.add_argument("file")
    p.add_argument("--flow", type=int, default=0, choices=[0, 1, 2],
                   help="0 = no; 1 = tara solo i filamenti nuovi (altrimenti PA salvato); 2 = sempre")
    p.add_argument("--firstlayer", type=int, default=0, choices=[0, 1])
    p.add_argument("--on-bad", default="pause", choices=["pause", "cancel"])
    p.add_argument("--pa-apply", type=int, default=1, choices=[0, 1],
                   help="applica il PA salvato del filamento anche senza taratura")
    p.add_argument("--material", default="PLA", help="ripiego se il gcode non lo indica")
    p.add_argument("--hotend", type=int, default=0, help="ripiego se il gcode non lo indica")
    p.add_argument("--bed", type=int, default=0, help="ripiego se il gcode non lo indica")
    p.add_argument("--backend", default="driver", choices=["driver", "middleware"])
    q = sub.add_parser("pa", help="taratura PA a pulsante")
    q.add_argument("--nozzle", type=int, default=None)
    q.add_argument("--bed", type=int, default=None)
    q.add_argument("--filament", default=None, help="nome del filamento (come nello slicer)")
    q.add_argument("--material", default=None)
    q.add_argument("--gcode", default=None, help="prendi profilo filamento e temperature da questo gcode")
    q.add_argument("--fine", action="store_true")
    q.add_argument("--start", type=float)
    q.add_argument("--step", type=float)
    q.add_argument("--backend", default="driver", choices=["driver", "middleware"])
    s = sub.add_parser("set", help="assegna a mano il PA di un profilo filamento")
    s.add_argument("--pa", type=float, required=True)
    s.add_argument("--filament", default=None)
    s.add_argument("--gcode", default=None)
    s.add_argument("--nozzle", type=int, default=None)
    s.add_argument("--material", default=None)
    sub.add_parser("list", help="elenca i profili filamento tarati")
    return ap


def main(argv=None):
    a = build_parser().parse_args(argv)
    t0 = time.time()
    try:
        if a.cmd == "print":
            r = do_print(a)
        elif a.cmd == "pa":
            r = do_pa(a)
        elif a.cmd == "set":
            r = do_set(a)
        elif a.cmd == "list":
            r = do_list(a)
        else:
            build_parser().print_help()
            return 2
    except Exception as e:
        log(">>> JOB FALLITO: %s" % e)
        return 1
    log(">>> JOB OK (%.0fs): %s" % (time.time() - t0, {k: v for k, v in r.items() if k != "firstlayer"}))
    return 0
