#!/usr/bin/env python3
"""Test di regressione di C2 (fl_analyze.py) su scansioni REALI archiviate.

Rianalizza ogni run archiviato (manifest.json + .temp) e confronta il verdetto con
quello atteso. Gli attesi vengono da run con esito verificato SUL PEZZO
(roadmap A2, 2026-07-18): pezzo intatto -> GOOD, sollevato -> BAD, sollevato +
materiale asportato -> BAD. Un cambio a fl_analyze.py o ai parametri di scan che
rompe uno di questi casi va visto PRIMA di andare sulla stampante.

Uso (sul PC, serve numpy):
    python tests/test_c2_regression.py [CARTELLA_ARCHIVIO] [--gcodes CARTELLA]

CARTELLA_ARCHIVIO: default ../archivio/scansioni-lidar (fuori dal repo, come da
regola del progetto). Il gcode del run si cerca, in ordine: `segs_mm` nel manifest
(run da novembre 2026 in poi), il .gcode accanto ai run, la cartella --gcodes.
"""
import argparse, json, os, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import lidar_firstlayer as lf  # noqa: E402

# verdetti verificati sul pezzo; i run non elencati usano il verdetto del manifest
EXPECTED = {
    "20260718_211601_Corpo1_PLA_10m33s.gcode": ("GOOD", "pezzo intatto (confermato)"),
    "20260718_212132_Corpo1_PLA_10m33s.gcode": ("BAD", "sollevato in basso a sinistra"),
    "20260718_213113_Corpo1_PLA_10m33s.gcode": ("BAD", "sollevato + materiale asportato"),
}


def find_segs(run_dir, man, gdirs):
    if man.get("segs_mm"):
        return [tuple(s) for s in man["segs_mm"]], man.get("layer_h_mm", 0.2)
    name = man.get("gcode") or ""
    cands = [os.path.join(run_dir, name)] + [os.path.join(d, name) for d in gdirs]
    for c in cands:
        if name and os.path.isfile(c):
            return lf.first_layer_segments(c)
    return None, None


def run_one(run_dir, gdirs):
    man = json.load(open(os.path.join(run_dir, "manifest.json")))
    rid = os.path.basename(run_dir.rstrip("/\\"))
    exp, why = EXPECTED.get(rid, ((man.get("verdict_c2") or {}).get("verdict"), "dal manifest"))
    if not exp:
        return rid, "SALTATO", "nessun verdetto atteso"
    need = ("bands", "scan_y0_mm", "scan_y1_mm", "n_frames")
    if any(k not in man for k in need):
        return rid, "SALTATO", "manifest senza parametri di scan"
    segs, lz = find_segs(run_dir, man, gdirs)
    if segs is None:
        return rid, "SALTATO", "gcode %r non trovato" % man.get("gcode")
    nb = len(man["bands"])
    tabs = [os.path.join(run_dir, "scan_table_%d.temp" % k) for k in range(nb)]
    lays = [os.path.join(run_dir, "scan_layer_%d.temp" % k) for k in range(nb)]
    if not all(os.path.isfile(p) for p in tabs + lays):
        return rid, "SALTATO", "file .temp mancanti"
    t0 = time.time()
    r, _fa = lf.analyze_c2(man["bands"], tabs, lays, segs, lz, man["scan_y0_mm"],
                           man["scan_y1_mm"], man["n_frames"], man.get("line_width", 0.5))
    got = r.get("verdict")
    m = r.get("metrics", {})
    info = "atteso %s (%s), ottenuto %s | copertura %.1f%% | %.1fs" % (
        exp, why, got, 100.0 * (m.get("coverage") or 0), time.time() - t0)
    for reason in r.get("reasons", [])[:3]:
        info += "\n        - %s" % reason
    return rid, ("OK" if got == exp else "FALLITO"), info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("archive", nargs="?",
                    default=os.path.join(ROOT, "..", "archivio", "scansioni-lidar"))
    ap.add_argument("--gcodes", action="append", default=[])
    a = ap.parse_args()
    if not os.path.isdir(a.archive):
        print("archivio non trovato: %s (test saltato)" % a.archive)
        return 0
    gdirs = [a.archive] + a.gcodes
    runs = sorted(d for d in (os.path.join(a.archive, x) for x in os.listdir(a.archive))
                  if os.path.isfile(os.path.join(d, "manifest.json")))
    fails = 0; done = 0
    for d in runs:
        rid, st, info = run_one(d, gdirs)
        print("[%s] %s\n      %s" % (st, rid, info))
        fails += st == "FALLITO"
        done += st in ("OK", "FALLITO")
    print("\n%d run verificati, %d falliti" % (done, fails))
    return 1 if fails or not done else 0


if __name__ == "__main__":
    sys.exit(main())
