#!/usr/bin/env python3
"""Con zhop=False il pattern generato deve coincidere, mossa per mossa, con quello
di fabbrica; con zhop=True (default) nessuno spostamento deve avvenire vicino al piatto.

Il gcode originale Creality NON e' nel repository pubblico: se manca il test lo
salta (lo si esegue nel repo di sviluppo). Controlla anche le griglie fini.
Uso: python tests/test_pa_pattern.py
"""
import os, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from k1lidar import pa_pattern as P  # noqa: E402

ORIG = os.path.join(ROOT, "pa_pattern_original.gcode")


def check_travel_height(text, z_min=P.TRAVEL_Z):
    """Ogni mossa XY senza estrusione deve avvenire con Z >= z_min."""
    z = None
    for n, line in enumerate(text.splitlines(), 1):
        w = line.split(";")[0].split()
        if not w or w[0] not in ("G0", "G1"):
            continue
        d = {t[0]: float(t[1:]) for t in w[1:] if t[0] in "XYZEF"}
        if "Z" in d:
            z = d["Z"]
        if ("X" in d or "Y" in d) and "E" not in d and z is not None and z < z_min:
            raise AssertionError("riga %d: spostamento a Z%.2f: %s" % (n, z, line))
    print("OK: spostamenti tutti a Z >= %.1f" % z_min)


def main():
    g = P.grid()
    assert g[0] == 0.020 and g[-1] == 0.082 and len(g) == 32, g
    f = P.fine_grid(0.030)
    assert len(f) == 32 and abs(f[1] - f[0] - 0.001) < 1e-9 and f[0] <= 0.030 <= f[-1], f
    assert P.fine_grid(0.005)[0] == 0.0
    check_travel_height(P.generate(P.grid(), 220, 60))
    check_travel_height(P.generate(P.fine_grid(0.03), 220, 60, x_offset=P.FINE_OFFSET_X,
                                   prime_y=P.FINE_PRIME_Y))
    gen = P.generate(P.grid(), 220, 60, zhop=False)
    if not os.path.isfile(ORIG):
        print("pattern originale assente: confronto saltato (ok nel repo pubblico)")
        return 0
    a, b = P.moves(open(ORIG).read()), P.moves(gen)
    if a != b:
        for i, (x, y) in enumerate(zip(a, b)):
            if x != y:
                print("prima differenza alla mossa %d:\n  originale %r\n  generato  %r" % (i, x, y))
                break
        print("lunghezze: originale %d, generato %d" % (len(a), len(b)))
        return 1
    print("OK: pattern generato identico all'originale (%d mosse)" % len(a))
    return 0


if __name__ == "__main__":
    sys.exit(main())
