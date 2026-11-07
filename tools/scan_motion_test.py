#!/usr/bin/env python3
# tools/scan_motion_test.py - prova a VUOTO della passata di scansione PA (niente
# riscaldamento, niente stampa): esegue scan_sweep come nella taratura e riporta
# come sono state catturate e posizionate le immagini del laser.
#
#   cd /usr/data/k1-lidar && python3 tools/scan_motion_test.py [--mode cont|step] [--speed 800]
#
# Piatto LIBERO: la testa scende a Z3. Fa G28 solo se gli assi non sono homati.
import os, sys, json, argparse
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from k1lidar import session as S
from k1lidar import pa as PA

ap = argparse.ArgumentParser()
ap.add_argument("--mode", default="cont")
ap.add_argument("--speed", type=int, default=800)
ap.add_argument("--frames", type=int, default=170)
ap.add_argument("--backend", default="driver")
ap.add_argument("--out", default="/tmp/scan_motion_test.temp")
a = ap.parse_args()

if not S.acquire_lock():
    sys.exit("laser occupato (%s)" % S.LOCK)
L = None
try:
    homed = S.kl_query({"toolhead": ["homed_axes"]}).get("toolhead", {}).get("homed_axes", "")
    if homed != "xyz":
        S.kl("G28")
    L = S.laser_up(a.backend)
    if L is None:
        raise RuntimeError("laser non disponibile")
    st = PA.scan_sweep(L, a.out, "test", "table", n_frames=a.frames, mode=a.mode, speed=a.speed)
    print(json.dumps(st, indent=1))
finally:
    if L is not None:
        L.shutdown()
    try:
        S.kl("G0 Z20 F600")
    except Exception:
        pass
    S.release_lock()
