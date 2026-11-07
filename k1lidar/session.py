# k1lidar/session.py - parti COMUNI del toolkit LiDAR K1 (prima erano copiate in ogni script).
# Gira SULLA STAMPANTE (python3.8 del firmware, solo stdlib).
#
# Contiene:
#  - client Klipper (/tmp/klippy_uds): kl, kl_query, kl_rpc
#  - client middleware (/tmp/ai_server_uds) con connessione PERSISTENTE: ai
#  - alimentazione laser (GPIO PA14), lock condiviso tra gli script, archivio dei run
#  - calibrazioni persistenti per filamento (esposizione, PA, baseline altezza C2)
#  - DUE backend laser con la stessa interfaccia:
#      DriverLaser      seriale diretta col nostro laser_driver.py (default)
#      MiddlewareLaser  via cx_ai_middleware (il metodo storico, resta come ripiego)
#
# Perche' il driver diretto e' il default (verifica su hardware 2026-11-06):
# frame identici al middleware, nessuna trasformazione
# dei punti, ~80ms/frame invece di ~140; e il middleware ha un fd leak sulla
# seriale e si e' impiccato piu' volte. Il middleware serve ancora solo per il
# detector Creality del primo strato (interno al middleware, solo informativo).

import json, os, socket, subprocess, sys, time

PKG = os.path.dirname(os.path.abspath(__file__))
HERE = ROOT = os.path.dirname(PKG)        # radice del toolkit (laser_driver.py, algo/, ...)
AI = "/tmp/ai_server_uds"
KL = "/tmp/klippy_uds"
PC = "/usr/data/creality/tmp/pointCloud"
# Archivio PERSISTENTE: PC viene SVUOTATA dal software di serie (master-server) a
# ogni avvio della stampante.
# /usr/data sta su mmcblk0p10 ed e' fuori da quella pulizia.
ARCHIVE = "/usr/data/lidar_archive"
GCODES = "/usr/data/printer_data/gcodes"
# Calibrazioni per filamento: fuori dalla cartella del toolkit, cosi' un
# reinstall/deploy non le sovrascrive.
CALIB = "/usr/data/lidar_calib.json"
PORT = "/dev/serial/by-id/creality-laser"
POWER = "/usr/bin/laser_power.sh"
LOCK = "/tmp/k1lidar.lock"          # UNO per tutti gli script: il laser e' esclusivo

MATS = {"PLA": (220, 60), "PETG": (240, 75), "ABS": (250, 90), "TPU": (230, 40), "ASA": (250, 90)}
# Limiti temperature custom: minimi richiesti, massimi dall'hardware
# (printer.cfg: extruder max_temp 320, heater_bed max_temp 115).
MIN_HOTEND, MAX_HOTEND = 150, 320
MIN_BED, MAX_BED = 40, 115
# CX_ROUGH_G28 accetta EXTRUDER_TEMP solo tra 180 e 320 (custom_macro.py).
LEVEL_MIN_HOTEND = 180
EXPOSURE_MIN, EXPOSURE_MAX = 50, 5000


def log(msg=""):
    print(msg)
    sys.stdout.flush()


# --------------------------------------------------------------------- argomenti --
def temp_arg(lo, hi, what):
    """Validatore argparse per una temperatura, con messaggio parlante."""
    import argparse

    def parse(v):
        try:
            n = int(round(float(v)))
        except ValueError:
            raise argparse.ArgumentTypeError("%s: '%s' non e' un numero" % (what, v))
        if not lo <= n <= hi:
            raise argparse.ArgumentTypeError(
                "%s %d fuori dai limiti consentiti (%d-%d)" % (what, n, lo, hi))
        return n
    return parse


def add_common_args(ap):
    """Opzioni condivise da lidar_pa.py e lidar_firstlayer.py."""
    ap.add_argument("--material", default="PLA", choices=list(MATS),
                    help="preset materiale (temperature di partenza)")
    ap.add_argument("--filament", default=None,
                    help="nome del filamento (es. PLA_arancione): chiave delle calibrazioni "
                         "salvate (esposizione, PA, baseline). Default = --material")
    ap.add_argument("--hotend", type=temp_arg(MIN_HOTEND, MAX_HOTEND, "ugello"),
                    help="temperatura ugello custom %d-%d C" % (MIN_HOTEND, MAX_HOTEND))
    ap.add_argument("--bed", type=temp_arg(MIN_BED, MAX_BED, "piatto"),
                    help="temperatura piatto custom %d-%d C" % (MIN_BED, MAX_BED))
    ap.add_argument("--backend", default="driver", choices=["driver", "middleware"],
                    help="driver = seriale diretta (default); middleware = cx_ai_middleware")
    ap.add_argument("--exposure", type=int, default=None,
                    help="esposizione del sensore per gli scan (default: quella salvata per il "
                         "filamento, altrimenti quella del modulo, di solito 1000)")


def resolve_temps(a):
    """Preset materiale + override custom. Ritorna (hotend, bed, sorgente).
    Verifica che l'ugello sia sopra il min_extrude_temp REALE della stampante:
    sotto quella soglia klipper rifiuta di estrudere ('Extrude below minimum temp')."""
    hot, bed = MATS[a.material]
    src = "preset %s" % a.material
    if getattr(a, "hotend", None) is not None:
        hot = a.hotend; src = "custom"
    if getattr(a, "bed", None) is not None:
        bed = a.bed; src = "custom"
    met = kl_query({"configfile": ["settings"]}).get("configfile", {}) \
        .get("settings", {}).get("extruder", {}).get("min_extrude_temp")
    if met and hot < float(met):
        raise SystemExit(
            "ugello %d C < min_extrude_temp della stampante (%.0f C): klipper "
            "rifiuterebbe di estrudere e la stampa fallirebbe.\n"
            "  Alza --hotend ad almeno %.0f, oppure abbassa min_extrude_temp in printer.cfg."
            % (hot, float(met), float(met)))
    return hot, bed, src


def filament_key(a, hot=None):
    """Chiave delle calibrazioni: nome del filamento (--filament, altrimenti il
    materiale) + temperatura ugello, come per le stampe (gcode_meta.filament_key)."""
    from .gcode_meta import filament_key as fk
    return fk(name=getattr(a, "filament", None) or a.material, nozzle=hot)


def say_cmd(line):
    """Gcode che scrive `line` nella console di Klipper. Sulla K1 M118 NON esiste
    ('Unknown command'): si usa la macro _LIDAR_SAY di k1lidar.cfg
    (action_respond_info). Le virgolette doppie romperebbero il parametro MSG."""
    return '_LIDAR_SAY MSG="%s"' % str(line).replace('"', "'")[:200]


def echo(msg):
    """Messaggio per l'utente: nel log E nella console di Klipper (Fluidd/Mainsail/
    display). Non deve mai far fallire un run."""
    log(msg)
    try:
        for line in str(msg).splitlines():
            line = line.strip()
            if line:
                kl(say_cmd(line), t=20)
    except Exception:
        pass


def factory_level(hot, bed):
    """Sequenza di livellamento di FABBRICA (stesso z-offset del display):
    CX_ROUGH_G28 (scalda, homing) + pulizia ugello + homing fine + controllo mesh.
    CX_ROUGH_G28 rifiuta EXTRUDER_TEMP < 180: si livella col minimo accettato,
    la temperatura vera la mette M109 al momento di stampare."""
    level_hot = max(hot, LEVEL_MIN_HOTEND)
    if level_hot != hot:
        log("    (livellamento a EXTRUDER_TEMP=%d: CX_ROUGH_G28 non accetta %d)" % (level_hot, hot))
    kl("CX_ROUGH_G28 EXTRUDER_TEMP=%d BED_TEMP=%d" % (level_hot, bed))
    kl("CX_NOZZLE_CLEAR")
    kl("ACCURATE_G28")
    kl("CX_PRINT_LEVELING_CALIBRATION")


# ------------------------------------------------------------------------ klipper --
def _sock(path, t):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(t)
    s.connect(path)
    return s


def _kl_call(req_id, method, params, t):
    s = _sock(KL, t)
    req = {"id": req_id, "method": method}
    if params is not None:
        req["params"] = params
    s.sendall(json.dumps(req).encode() + b"\x03")
    buf = b""
    try:
        while b"\x03" not in buf:
            c = s.recv(1 << 16)
            if not c:
                break
            buf += c
    finally:
        s.close()
    for fr in buf.split(b"\x03"):
        if not fr.strip():
            continue
        try:
            m = json.loads(fr)
        except Exception:
            continue
        if m.get("id") == req_id:
            return m
    return {}


def kl(gcode, t=400.0):
    """Esegue gcode su Klipper; solleva RuntimeError se Klipper risponde errore."""
    m = _kl_call(88, "gcode/script", {"script": gcode}, t)
    if "error" in m:
        raise RuntimeError(m["error"])
    return m


def kl_query(objs, t=10.0):
    return _kl_call(89, "objects/query", {"objects": objs}, t).get("result", {}).get("status", {})


def kl_rpc(method, params=None, t=30.0):
    """Endpoint webhooks di klipper (non gcode). Ritorna il result."""
    m = _kl_call(91, method, params, t)
    if "error" in m:
        raise RuntimeError(m["error"])
    return m.get("result", {})


def kl_query_ev(objs, t=10.0):
    """Come kl_query, ma ritorna anche l'eventtime di klippy (stesso orologio di mono())."""
    r = _kl_call(90, "objects/query", {"objects": objs}, t).get("result", {})
    return r.get("eventtime"), r.get("status", {})


# --------------------------------------------------------------- passata lineare --
# Perche' esiste (bug trovato il 2026-11-07): il vecchio scan "cont" mandava il G0
# della passata e faceva partire un orologio quando la chiamata ritornava, poi
# scattava i frame a tempo. Ma klipper (toolhead._check_pause) TRATTIENE la
# risposta finche' in coda restano piu' di buffer_time_high = 2 s di moto: su una
# passata di 15 s la chiamata ritorna dopo ~11 s. Risultato: frame presi con la
# testa gia' a meta' corsa e poi ~130 frame a testa FERMA a fine corsa (run PA
# PLA_arancione: righe 3-8 del pattern mai lette). Il log diceva "ritardo 0ms".
#
# Ora: UN solo G0 continuo (come l'originale) lanciato da un thread; il thread
# principale cattura frame a raffica con timestamp sull'orologio di klippy; la Y di
# ogni frame si ricava dalla traiettoria VERA (motion_report/dump_trapq) o, se non
# disponibile, dal trapezio di moto ancorato al print_time di fine moto. Infine
# per ogni posizione bersaglio si tiene il frame piu' vicino: l'indice del frame
# torna a essere la posizione Y, come vogliono gli analizzatori.
SENSOR_LAG = 0.0      # s tra esposizione del sensore e risposta (da tarare, costante)


def mono():
    """Orologio di klippy: reactor.monotonic() = CLOCK_MONOTONIC_RAW (chelper/pyhelper.c)."""
    return time.clock_gettime(time.CLOCK_MONOTONIC_RAW)


def move_time(dist, v, a):
    """Durata di un moto da fermo a fermo (trapezio, o triangolo se troppo corto)."""
    d_acc = v * v / (2.0 * a)
    if 2 * d_acc >= dist:
        return 2.0 * (dist / a) ** 0.5
    return 2.0 * v / a + (dist - 2 * d_acc) / v


def _trapezoid_pos(y_a, y_b, v, a, dt):
    """Posizione lungo un moto da fermo a fermo y_a->y_b, dt secondi dopo l'inizio."""
    dist = abs(y_b - y_a); sgn = 1.0 if y_b >= y_a else -1.0
    d_acc = v * v / (2.0 * a)
    if 2 * d_acc >= dist:
        v = (dist * a) ** 0.5; d_acc = dist / 2.0
    ta = v / a; tc = (dist - 2 * d_acc) / v; tot = 2 * ta + tc
    if dt <= 0:
        s = 0.0
    elif dt < ta:
        s = 0.5 * a * dt * dt
    elif dt < ta + tc:
        s = d_acc + v * (dt - ta)
    elif dt < tot:
        r = tot - dt; s = dist - 0.5 * a * r * r
    else:
        s = dist
    return y_a + sgn * s


class TrapqWatch(object):
    """Riceve da klippy i segmenti di moto ESEGUITI (motion_report/dump_trapq,
    aggiornamento ogni 0.5 s). Ogni segmento: (print_time, durata, v0, accel,
    pos_iniziale, direzione). Se l'endpoint non c'e', moves resta vuota."""

    def __init__(self):
        import threading
        self.moves = []
        self.ok = False
        self._stop = False
        try:
            self.s = _sock(KL, 5.0)
            req = {"id": 95, "method": "motion_report/dump_trapq",
                   "params": {"name": "toolhead", "response_template": {"k1lidar": 1}}}
            self.s.sendall(json.dumps(req).encode() + b"\x03")
            self.s.settimeout(0.3)
            self.t = threading.Thread(target=self._run)
            self.t.daemon = True
            self.t.start()
            self.ok = True
        except Exception as e:
            log("    (dump_trapq non disponibile: %s)" % e)

    def _run(self):
        buf = b""
        while not self._stop:
            try:
                c = self.s.recv(1 << 16)
                if not c:
                    break
                buf += c
            except socket.timeout:
                continue
            except Exception:
                break
            while b"\x03" in buf:
                fr, buf = buf.split(b"\x03", 1)
                try:
                    m = json.loads(fr)
                except Exception:
                    continue
                d = (m.get("params") or {}).get("data")
                if d:
                    self.moves.extend(d)

    def close(self):
        self._stop = True
        try:
            self.s.close()
        except Exception:
            pass

    def covers(self, pt):
        return any(m[0] <= pt <= m[0] + m[1] + 1e-6 for m in self.moves)

    def y_at(self, pt):
        """Y della testa al print_time pt, o None se pt non cade in un segmento noto."""
        for m in self.moves:
            t0, mt, v0, acc, sp, dr = m
            if t0 <= pt <= t0 + mt + 1e-6:
                dt = pt - t0
                return sp[1] + dr[1] * (v0 + 0.5 * acc * dt) * dt
        return None


def _seg_end_y(m):
    t0, mt, v0, acc, sp, dr = m
    return sp[1] + dr[1] * (v0 + 0.5 * acc * mt) * mt


def _sweep_segments(moves, y_pre, y_post, pt_hint):
    """Segmenti trapq della passata y_pre->y_post (la mesh la spezza in piu' pezzi):
    dall'ultimo segmento che parte da fermo in y_pre verso +Y, fino a quello che
    arriva in y_post. None se la trapq non li contiene (ancora) tutti."""
    starts = [i for i, m in enumerate(moves) if m[0] < pt_hint + 0.5 and m[5][1] > 0.5
              and abs(m[4][1] - y_pre) < 0.05 and abs(m[2]) < 1e-6]
    if not starts:
        return None
    seg = []
    for m in sorted(moves[starts[-1]:], key=lambda m: m[0]):
        if seg and m[0] > seg[-1][0] + seg[-1][1] + 1e-4:
            break                                    # buco: non e' piu' la passata
        seg.append(m)
        if abs(_seg_end_y(m) - y_post) < 0.05:
            return seg
    return None


def linear_sweep(L, x, y0, y1, n_frames, speed, label, y_min=0.0, y_max=None, margin=1.0,
                 lag=None):
    """Passata LINEARE continua da y0 a y1 (con pre/post-roll per accelerare fuori
    finestra) e n_frames frame alle Y esatte y0 + i*passo. Il backend L deve avere
    gia' fatto open_scan(). Ritorna (frames, info). Un frame senza cattura entro
    tolleranza resta [] (l'indice deve restare = posizione)."""
    import threading
    lag = SENSOR_LAG if lag is None else lag
    step = (y1 - y0) / float(n_frames - 1)
    th = kl_query({"toolhead": ["max_accel", "axis_maximum"]}).get("toolhead", {})
    a = float(th.get("max_accel") or 500.0)
    if y_max is None:
        try:
            y_max = float(th["axis_maximum"][1])
        except Exception:
            y_max = 221.2
    # Il sensore da' ~11 immagini/s (misurato 88-95 ms): perche' ogni posizione
    # bersaglio abbia una cattura vicina, le catture devono cadere ogni <= mezzo passo.
    for _ in range(2):
        L.grab()                                   # warmup pipeline laser
    t0 = mono(); L.grab(); L.grab(); t_frame = max(0.03, (mono() - t0) / 2.0)
    v_max = 0.5 * step / t_frame
    if speed / 60.0 > v_max:
        log("  [%s] sensore %.0f ms/immagine: velocita' F%d -> F%d (catture ogni <= %.2fmm)"
            % (label, t_frame * 1000, speed, int(v_max * 60), 0.5 * step))
        speed = int(v_max * 60)
    v = speed / 60.0
    d_acc = v * v / (2.0 * a)
    y_pre = max(y_min, y0 - d_acc - margin)
    y_post = min(y_max, y1 + d_acc + margin)
    if y_pre > y0 - d_acc or y_post < y1 + d_acc:
        log("  [%s] ATTENZIONE: poco spazio per accelerare ai bordi corsa (Y%.1f-%.1f)"
            % (label, y_pre, y_post))
    kl("G90\nG0 X%.2f Y%.3f F12000\nM400" % (x, y_pre))
    L.grab()
    watch = TrapqWatch()
    st = {}

    def mover():
        # NB: il print_time letto DURANTE il moto non e' la fine della passata: la
        # compensazione della mesh spezza il G0 in segmenti e klipper li pianifica
        # a pezzi. Dopo M400 invece e' la fine vera (misurato 2026-11-07).
        try:
            kl("G0 X%.2f Y%.3f F%d\nM400" % (x, y_post, int(round(speed))))
            ev, s = kl_query_ev({"toolhead": ["print_time", "estimated_print_time"]})
            tq = s.get("toolhead", {})
            if ev is None or tq.get("print_time") is None:
                st["err"] = "stato del toolhead non leggibile"
                return
            st.update(E=ev, Q=tq["estimated_print_time"], pt_end=tq["print_time"], done=True)
        except Exception as e:
            st["err"] = str(e)

    D = move_time(y_post - y_pre, v, a)
    caps = []
    mv = threading.Thread(target=mover)
    mv.daemon = True
    t_dead = mono() + D + 20.0
    mv.start()
    try:
        while True:
            t0 = mono(); pc = L.grab(); t1 = mono()
            caps.append((0.5 * (t0 + t1), pc))
            if "err" in st:
                raise RuntimeError("passata %s: %s" % (label, st["err"]))
            if st.get("done"):
                break
            if t1 > t_dead:
                raise RuntimeError("passata %s: timeout (moto non concluso)" % label)
        mv.join(5)
        pt_end = st["pt_end"]
        to_pt = lambda t: st["Q"] + (t - st["E"]) - lag
        # (il print_time dopo M400 e' ~20 ms oltre la fine del moto: va bene per il
        # modello di riserva, la trapq da' gli istanti esatti)
        pt_start = pt_end - D
        # traiettoria vera: i segmenti arrivano dopo l'esecuzione, aggiornati ogni 0.5 s
        src = "modello"
        if watch.ok:
            t_w = mono() + 2.5
            while True:
                sw = _sweep_segments(watch.moves, y_pre, y_post, pt_end)
                if sw or mono() > t_w:
                    break
                time.sleep(0.1)
            if sw:
                src = "trapq"
                pt_start = sw[0][0]
                pt_end = sw[-1][0] + sw[-1][1]
    finally:
        watch.close()

    ys, dmax = [], 0.0
    for t, pc in caps:
        pt = to_pt(t)
        ym = _trapezoid_pos(y_pre, y_post, v, a, pt - pt_start)
        if pt <= pt_start:
            yv = y_pre                       # testa ancora ferma all'inizio
        elif pt >= pt_end:
            yv = y_post                      # moto concluso
        else:
            yv = watch.y_at(pt) if src == "trapq" else None
        if yv is not None and pt_start < pt < pt_end:
            dmax = max(dmax, abs(yv - ym))
        ys.append(yv if yv is not None else ym)

    try:      # diagnostica: tempi e posizioni di ogni cattura (niente nuvole di punti)
        sig = []
        for t, pc in caps:
            zz = sorted(p[2] for p in pc) if pc else []
            sig.append([len(pc), zz[len(zz) // 2] if zz else None,
                        [p[2] for p in pc[::max(1, len(pc) // 24)]][:24] if pc else []])
        with open("/tmp/k1lidar_sweep_%s.json" % label.replace(" ", "_"), "w") as fh:
            json.dump({"st": st, "D": D, "v": v, "a": a, "y_pre": y_pre, "y_post": y_post,
                       "pt_start": pt_start, "moves": watch.moves, "src": src,
                       "caps": [[t, to_pt(t), y] for (t, _), y in zip(caps, ys)],
                       "sig": sig}, fh)
    except Exception:
        pass

    tol = max(0.5 * step, 0.4)
    frames, errs, empty = [], [], 0
    for k in range(n_frames):
        yk = y0 + k * step
        best, bd = None, None
        for (t, pc), yv in zip(caps, ys):
            if not pc:
                continue
            d = abs(yv - yk)
            if bd is None or d < bd:
                best, bd = pc, d
        if best is not None and bd <= tol:
            frames.append(best); errs.append(bd)
        else:
            frames.append([]); empty += 1
    in_win = [i for i, yv in enumerate(ys) if y0 <= yv <= y1]
    spacing = (y1 - y0) / max(1, len(in_win) - 1)
    # tutte le catture nella finestra con la loro Y vera (per gli analizzatori che
    # sfruttano la densita' piena invece dei frame ricampionati)
    dense = [[round(yv, 3), pc] for (t, pc), yv in zip(caps, ys) if pc and y0 - 2 <= yv <= y1 + 2]
    info = {"captured": len(caps), "in_window": len(in_win), "spacing_mm": round(spacing, 3),
            "max_err_mm": round(max(errs) if errs else -1, 3), "empty": empty,
            "speed_F": int(round(speed)), "position_source": src,
            "model_vs_trapq_mm": round(dmax, 3) if src == "trapq" else None,
            "move_s": round(D, 2)}
    log("  [%s] passata lineare F%d: %d catture (%d in finestra, una ogni %.2fmm), "
        "Y da %s%s, errore max frame %.2fmm, %d frame senza cattura"
        % (label, speed, len(caps), len(in_win), spacing, src,
           (" (scarto dal modello %.2fmm)" % dmax) if src == "trapq" else "",
           info["max_err_mm"], empty))
    if spacing > step:
        log("  [%s] ATTENZIONE: catture piu' rade del passo (%.2f > %.2fmm): abbassa la velocita'"
            % (label, spacing, step))
    info["dense"] = dense
    return frames, info


def wait_state(want, timeout, poll=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = kl_query({"print_stats": ["state"]}).get("print_stats", {}).get("state", "?")
        if st in want:
            return st
        time.sleep(poll)
    return None


def creality_cooldown(thresh=150.0, timeout=240):
    """Blocco Creality pre-scan (dalla cattura, identico prima di scan linee PA e
    primo strato): ventole max + FIRST_FLOOR_PAUSE_POSITION (= M104 g28_ext_temp +
    park a bordo piatto Z0.2, l'ugello sbava FUORI) + attesa discesa temperatura +
    ventole off per scansionare (l'originale attende ~48s)."""
    kl("M106 P0 S255\nM106 P2 S255")
    kl("FIRST_FLOOR_PAUSE_POSITION")
    t0 = time.time()
    while time.time() - t0 < timeout:
        t = kl_query({"extruder": ["temperature"]}).get("extruder", {}).get("temperature", 999)
        if t <= thresh:
            log("    ugello a %.0fC (<=%.0f) dopo %.0fs" % (t, thresh, time.time() - t0))
            break
        time.sleep(3)
    else:
        log("    timeout raffreddamento (%.0fs), proseguo" % timeout)
    kl("M106 P0 S0\nM106 P2 S0")


def park(z=60):
    """Parcheggia la testa fuori dal piatto SENZA spegnere i motori.

    Prima si faceva M84: azzera l'homing e il display chiedeva di rifare l'home a
    ogni run (roadmap B2). I motori restano accesi; Klipper li spegne da solo col
    suo idle_timeout."""
    try:
        homed = kl_query({"toolhead": ["homed_axes"]}).get("toolhead", {}).get("homed_axes", "")
        if "xyz" in (homed or "").lower():
            kl("G90\nG0 Z%d F900\nG0 X110 Y215 F6000\nM400" % z)
    except Exception:
        pass


def heaters_off():
    try:
        kl("TURN_OFF_HEATERS", t=30)
    except Exception:
        pass


# --------------------------------------------------------------------- middleware --
_ai_sock = None
_ai_buf = b""


def ai(obj, t=30.0):
    """Comando al middleware su connessione PERSISTENTE (una socket per comando
    esaurisce gli fd del middleware e lo impicca). Riconnette se cade."""
    global _ai_sock, _ai_buf
    try:
        if _ai_sock is None:
            _ai_sock = _sock(AI, t)
            _ai_buf = b""
        _ai_sock.settimeout(t)
        _ai_sock.sendall(json.dumps(obj).encode() + b"\x03")
        while b"\x03" not in _ai_buf:
            c = _ai_sock.recv(1 << 20)
            if not c:
                raise IOError("connessione chiusa dal middleware")
            _ai_buf += c
        frame, _, _ai_buf = _ai_buf.partition(b"\x03")
        return json.loads(frame.decode("utf-8", "replace"))
    except Exception as e:
        ai_close()
        return {"result": {"err": str(e)}}


def ai_close():
    global _ai_sock, _ai_buf
    if _ai_sock is not None:
        try:
            _ai_sock.close()
        except Exception:
            pass
    _ai_sock = None
    _ai_buf = b""


def ubus(arg):
    subprocess.run(["ubus", "call", "laser", "set_state", arg],
                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def laser_power(state):
    """Alimenta il laser via GPIO PA14. Da fw 1.3.5.22 il flag ubus NON basta."""
    subprocess.run(["sh", POWER, state], stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def _wait_port(timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if os.path.exists(PORT):
            return True
        time.sleep(0.25)
    return False


def _wait_ota(timeout=60):
    """All'hotplug mdev lancia laser_ota_util che TIENE APERTA la seriale finche'
    controlla il firmware del modulo (flag /tmp/.laser_updating)."""
    for _ in range(timeout):
        if not os.path.exists("/tmp/.laser_updating"):
            return
        time.sleep(1)


def port_holders():
    """PID dei processi (diversi da noi) che tengono aperta la seriale VIVA del laser.
    I fd del middleware verso ttyACM "(deleted)" (il suo fd leak) non contano."""
    try:
        target = os.path.realpath(PORT)
    except OSError:
        return []
    me = os.getpid()
    out = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == me:
            continue
        d = "/proc/%s/fd" % pid
        try:
            for fd in os.listdir(d):
                try:
                    if os.readlink(os.path.join(d, fd)) == target:
                        out.append(int(pid))
                        break
                except OSError:
                    pass
        except OSError:
            pass
    return out


# --------------------------------------------------------------------- backend laser --
class MiddlewareLaser(object):
    """Laser via cx_ai_middleware (socket JSON). Metodo storico, validato da luglio."""
    name = "middleware"

    def __init__(self):
        self.exposure_orig = None

    def up(self, timeout=20, tries=3):
        laser_power("on")
        if not _wait_port(timeout):
            log("    ERRORE: creality-laser non enumerato dopo %ds (laser scollegato?)" % timeout)
            return False
        _wait_ota()
        time.sleep(2)
        for i in range(tries):
            v = ai({"control": "get_laser_version"}).get("result", {})
            if v.get("code") == 0:
                log("    laser OK (middleware): %s" % v)
                return True
            log("    tentativo %d/%d: laser non pronto (%s), forzo re-init" % (i + 1, tries, v))
            # se e' gia' a 1 un altro set_state 1 non genera transizione: forziamo 0->1
            ubus('{"laser_plugged":0}'); time.sleep(1)
            ubus('{"laser_plugged":1}'); time.sleep(3)
        return False

    def cali_step(self, step):
        return ai({"control": "set_laser_cali_step", "step": step}).get("result", {}).get("code")

    def fill_light_off(self):
        ai({"control": "set_fill_light_close"})

    def get_exposure(self):
        r = ai({"control": "get_laser_exposure"}).get("result", {})
        return r.get("exposure") if r.get("code") == 0 else None

    def set_exposure(self, v):
        if self.exposure_orig is None:
            self.exposure_orig = self.get_exposure()
        return ai({"control": "set_laser_exposure", "exposure": int(v)}).get("result", {}).get("code") == 0

    def open_scan(self):
        ai({"control": "set_laser_coor", "coor": 1})
        ai({"control": "set_point_cloud_level", "level": 2})
        ai({"control": "set_take_over_model", "takeOverModel": 1})
        r = ai({"control": "open_flow"})
        if r.get("result", {}).get("code") != 0:
            log("    ATTENZIONE open_flow: %s" % r)
            return False
        return True

    def grab(self):
        return ai({"control": "get_point_cloud"}).get("result", {}).get("point_cloud", []) or []

    def close_scan(self):
        ai({"control": "close_flow"})

    def detect_first_floor(self, table, scan, gcode_red):
        return ai({"control": "get_first_floor_detect_new", "scan1_path": table,
                   "scan2_path": scan, "gcode_path": gcode_red}, t=90.0).get("result", {})

    def release(self):
        """Lascia il middleware PULITO: con take_over a 1 la calibrazione da display va in errore."""
        if self.exposure_orig:
            ai({"control": "set_laser_exposure", "exposure": int(self.exposure_orig)})
            self.exposure_orig = None
        for c in ({"control": "close_flow"},
                  {"control": "set_take_over_model", "takeOverModel": 0},
                  {"control": "set_fill_light_close"},
                  {"control": "set_laser_close"}):
            try:
                ai(c)
            except Exception:
                pass
        ai_close()
        ubus('{"laser_plugged":0}')

    def shutdown(self):
        self.release()
        laser_power("off")


class DriverLaser(object):
    """Laser via seriale diretta (laser_driver.py). Il middleware rilascia la porta
    (laser_plugged=0) e noi parliamo al modulo con lo stesso protocollo."""
    name = "driver"

    def __init__(self):
        self.m = None
        self.exposure_orig = None
        self.status4 = 0          # frame a cui il modulo ha risposto "nessun punto"
        self.errors = 0
        self._mw = None           # middleware acceso solo per il detector Creality

    def _ld(self):
        sys.path.insert(0, HERE)
        import laser_driver
        return laser_driver

    def up(self, timeout=20):
        ld = self._ld()
        self._close_port()
        laser_power("on")
        if not _wait_port(timeout):
            log("    ERRORE: creality-laser non enumerato dopo %ds (laser scollegato?)" % timeout)
            return False
        _wait_ota()
        # mdev fa inizializzare la porta al middleware: gliela facciamo rilasciare
        time.sleep(1)
        ubus('{"laser_plugged":0}')
        for _ in range(20):
            if not port_holders():
                break
            time.sleep(0.25)
        else:
            log("    ERRORE: la seriale del laser e' ancora aperta da pid %s" % port_holders())
            return False
        try:
            self.m = ld.LaserModule(ld.open_transport(PORT), timeout=3.0)
            info = self.m.init(set_coor=True)
        except Exception as e:
            log("    ERRORE driver laser: %s" % e)
            self._close_port()
            return False
        log("    laser OK (driver diretto): versione %s, SN %s" % (info.get("version"), info.get("sn")))
        return True

    def _close_port(self):
        if self.m is not None:
            try:
                self.m.close()
            except Exception:
                pass
        self.m = None

    def _safe(self, fn, *args):
        try:
            fn(*args)
            return True
        except Exception as e:
            log("    ATTENZIONE laser: %s" % e)
            return False

    def cali_step(self, step):
        """Il modulo risponde allo step di auto-calibrazione solo a calcolo finito:
        con 3s andava in timeout (visto sul campo, step 1). Il middleware attende
        fino a 8s (800 x 10ms): qui 15s, solo per questo comando."""
        ld = self._ld()
        old = self.m.timeout
        self.m.timeout = 15.0
        try:
            self.m.cali_step(step)
            return 0
        except ld.ModuleStatusError as e:
            return e.status
        except Exception as e:
            log("    cali_step %d: %s" % (step, e))
            return -1
        finally:
            self.m.timeout = old

    def fill_light_off(self):
        self._safe(self.m.fill_light, False)

    def get_exposure(self):
        try:
            return self.m.get_exposure()
        except Exception:
            return None

    def set_exposure(self, v):
        if self.exposure_orig is None:
            self.exposure_orig = self.get_exposure()
        return self._safe(self.m.set_exposure, int(v))

    def open_scan(self):
        ok = (self._safe(self.m.set_coor, 1) and self._safe(self.m.set_level, 2)
              and self._safe(self.m.set_take_over, 1) and self._safe(self.m.open_flow))
        if port_holders():
            # qualcuno (master-server -> laser_plugged=1) ha ridato la porta al
            # middleware: due lettori sulla stessa seriale si rubano i frame
            log("    ATTENZIONE: il middleware ha riaperto la seriale (pid %s): la richiudo"
                % port_holders())
            ubus('{"laser_plugged":0}')
            time.sleep(0.5)
        return ok

    def grab(self):
        ld = self._ld()
        try:
            return self.m.get_point_cloud()
        except ld.ModuleStatusError:
            self.status4 += 1          # stato 4 = nessun punto valido in questo frame
            return []
        except Exception:
            self.errors += 1
            return []

    def close_scan(self):
        self._safe(self.m.close_flow)

    def detect_first_floor(self, table, scan, gcode_red):
        """Il detector Creality vive nel middleware e risponde solo con il laser
        inizializzato (guard code 101): restituiamo la porta al middleware per il
        tempo del detect, poi la rilasciamo di nuovo."""
        self._close_port()
        if self._mw is None:
            self._mw = MiddlewareLaser()
            if not self._mw.up():
                self._mw.release()
                self._mw = None
                return {"code": -1, "err": "middleware non disponibile per il detector Creality"}
        return self._mw.detect_first_floor(table, scan, gcode_red)

    def release(self):
        if self._mw is not None:
            self._mw.release()
            self._mw = None
        if self.m is not None:
            if self.exposure_orig:
                self._safe(self.m.set_exposure, int(self.exposure_orig))
                self.exposure_orig = None
            for fn, arg in ((self.m.close_flow, None), (self.m.set_take_over, 0),
                            (self.m.fill_light, False), (self.m.laser, False)):
                try:
                    fn() if arg is None else fn(arg)
                except Exception:
                    pass
        self._close_port()
        if self.errors:
            log("    (driver: %d errori di lettura durante il run)" % self.errors)
        ubus('{"laser_plugged":0}')
        ai_close()

    def shutdown(self):
        self.release()
        laser_power("off")


def laser_up(backend="driver"):
    """Crea e accende il backend richiesto; se il driver diretto non parte ripiega
    sul middleware (un run non deve fallire solo per questo). Ritorna il backend o None."""
    L = DriverLaser() if backend == "driver" else MiddlewareLaser()
    if L.up():
        return L
    if backend == "driver":
        log("    driver diretto non disponibile: ripiego sul middleware")
        L.shutdown()
        L = MiddlewareLaser()
        if L.up():
            return L
    return None


def self_cal(L):
    """Self-cal del laser sull'adesivo a pallini (X7 Y140, fuori area stampa).
    Sequenza Creality: alterna Z1 (step 1) / Z3 (step 2), fino a 4 tentativi."""
    log("  self-cal laser sull'adesivo (X7 Y140)...")
    kl("G90\nG0 Z5 F600\nG0 X7 Y140 F8000\nM400")
    L.fill_light_off()
    for i in range(4):
        kl("G0 Z1 F600\nM400")
        r1 = L.cali_step(1)
        kl("G0 Z3 F600\nM400")
        r2 = L.cali_step(2)
        log("    iter %d: step1=%s step2=%s" % (i + 1, r1, r2))
        if r2 == 0:
            return True
    log("    self-cal senza step2=0, proseguo (come l'originale)")
    return False


def apply_exposure(L, a, calib):
    """Esposizione per lo scan: --exposure > valore salvato per il filamento > modulo.
    Ritorna il valore usato (None = quello del modulo). Il valore originale viene
    ripristinato da L.release()."""
    v = a.exposure if a.exposure is not None else calib.get("exposure")
    if v is None:
        cur = L.get_exposure()
        log("    esposizione: %s (del modulo)" % cur)
        return cur
    v = max(EXPOSURE_MIN, min(EXPOSURE_MAX, int(v)))
    ok = L.set_exposure(v)
    src = "--exposure" if a.exposure is not None else "salvata per il filamento"
    log("    esposizione: %d (%s)%s" % (v, src, "" if ok else " - NON applicata"))
    return v if ok else L.get_exposure()


# ------------------------------------------------------------------------- lock --
def acquire_lock():
    """Un solo run alla volta: il laser e' una risorsa esclusiva. Ritorna False se
    un'altra istanza VIVA tiene il lock (i lock orfani vengono rimossi)."""
    if os.path.exists(LOCK):
        try:
            pid = int(open(LOCK).read().strip())
        except Exception:
            pid = None
        if pid is not None:
            try:
                os.kill(pid, 0)
                return False
            except OSError:
                log("    (lock orfano di pid %s: lo rimuovo)" % pid)
        try:
            os.unlink(LOCK)
        except OSError:
            pass
    with open(LOCK, "w") as f:
        f.write(str(os.getpid()))
    return True


def release_lock():
    try:
        if os.path.exists(LOCK) and int(open(LOCK).read().strip()) == os.getpid():
            os.unlink(LOCK)
    except Exception:
        pass


# ---------------------------------------------------------------- archivio/calib --
def fw_version():
    try:
        with open("/usr/data/creality/userdata/config/system_version.json") as f:
            d = json.load(f)
        return d.get("sys_version") or d.get("softwareVersion") or d.get("version") or "?"
    except Exception:
        return "?"


def archive_run(meta, files):
    """Archivia i file del run in una cartella DATATA e PERSISTENTE + manifest.json
    auto-descrittivo. Va chiamata nel finally (un run abortito e' il caso piu'
    prezioso). Non deve MAI far fallire un run: tutto sotto except."""
    try:
        os.makedirs(ARCHIVE, exist_ok=True)
        d = os.path.join(ARCHIVE, meta["stamp"] + "_" + meta.get("tag", "run"))
        os.makedirs(d, exist_ok=True)
        saved = []
        for p in files:
            if not p or not os.path.exists(p):
                continue
            try:
                with open(p, "rb") as fs, open(os.path.join(d, os.path.basename(p)), "wb") as fd:
                    while True:
                        c = fs.read(1 << 16)
                        if not c:
                            break
                        fd.write(c)
                saved.append(os.path.basename(p))
            except Exception as e:
                log("    archivio: salto %s (%s)" % (os.path.basename(p), e))
        meta["files"] = saved
        meta.setdefault("fw", fw_version())
        with open(os.path.join(d, "manifest.json"), "w") as f:
            json.dump(meta, f, indent=2, sort_keys=True)
        subprocess.run(["sync"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        log("    archiviato in %s (%d file)" % (d, len(saved)))
        return d
    except Exception as e:
        log("    ATTENZIONE: archiviazione fallita (%s)" % e)
        return None


from .store import load_calib, save_calib  # noqa: E402,F401  (compatibilita')


def load_frames(path):
    """Legge un .temp (una chiave per frame) come lista di array numpy in ORDINE di
    indice. I frame vuoti vanno preservati: l'indice E' la posizione Y."""
    import numpy as np
    with open(path) as f:
        d = json.load(f)
    ks = sorted(d, key=lambda k: int("".join(c for c in k if c.isdigit()) or 0))
    return [np.array(d[k], dtype=np.float64).reshape(-1, 3) if d[k]
            else np.zeros((0, 3)) for k in ks]
