# k1lidar/pa_pattern.py - generatore del pattern di taratura PA.
#
# Riproduce la GEOMETRIA del pattern di fabbrica della K1 (8 righe x 4 nodi a zig-zag
# lungo il bordo sinistro del piatto, quella che algo/pa_phys.py analizza), ma con i
# valori di PA scelti da noi e gli spostamenti a Z5. Cosi' non serve distribuire il
# gcode di Creality e si possono fare sweep diversi (intervalli per materiale, fine).
#
# Geometria (misurata dal pattern originale):
#   - linea di innesco Y2 da X160 a X40, due riquadri X5-30 a Y0-5 e Y200-205;
#   - 4 nodi = 4 COPPIE di passate verticali a X_hi = 15+3n e 15.6+3n (n = 0..3);
#   - ogni passata e' uno zigzag da Y5 a Y205: 8 righe da 25 mm, in ognuna due
#     segmenti a 45 gradi (X_hi -> X_hi-12.5 -> X_hi) con E 0.883883 ciascuno;
#   - PA della riga r, nodo n = start + step * (4r + n)  (32 valori consecutivi).
# Di fabbrica: start 0.020, step 0.002 -> 0.020 ... 0.082.

ROWS, NODES = 8, 4
FACTORY_START, FACTORY_STEP = 0.020, 0.002
SEG_E = 0.883883


def grid(start=FACTORY_START, step=FACTORY_STEP):
    """32 valori di PA, indice k = 4*riga + nodo."""
    return [round(start + step * k, 4) for k in range(ROWS * NODES)]


def fine_grid(center, step=0.001):
    """Griglia fine centrata sul PA trovato (32 valori, non sotto 0)."""
    start = max(0.0, center - step * (ROWS * NODES // 2))
    return grid(start, step)


def _f(v):
    return ("%.4f" % v).rstrip("0").rstrip(".") if v else "0.00"


SCAN_X = 46.0               # X dell'ugello durante lo scan del pattern di fabbrica
FINE_OFFSET_X = 30.0        # la passata fine si stampa 30 mm piu' a destra (piatto pulito)
FINE_PRIME_Y = 214.0        # e innesca in fondo al piatto, fuori dalla zona scansionata
TRAVEL_Z = 5.0              # quota degli spostamenti col zhop (richiesta utente: 5 mm)


def generate(values, hotend=220, bed=60, x_offset=0.0, prime_y=2.0, zhop=True):
    """Gcode del pattern. `values` = 32 PA (vedi grid). Temperature intere.
    x_offset sposta tutto il pattern in X (lo scan va spostato dello stesso valore);
    prime_y e' la Y della linea di innesco (2 = come di fabbrica).
    zhop=True (default): ogni spostamento avviene a Z5 (TRAVEL_Z). Il pattern di
    fabbrica invece, a fine passata, trascina l'ugello a Z0.2 lungo i riquadri
    (Y205->Y200) e lo riporta a Z0.2 sul riquadro in basso prima della passata
    successiva: l'ugello striscia sul materiale gia' stampato. zhop=False
    riproduce il pattern di fabbrica mossa per mossa (usato dal test)."""
    if len(values) != ROWS * NODES:
        raise ValueError("servono %d valori di PA" % (ROWS * NODES))
    if any(v < 0 or v > 1.0 for v in values):
        raise ValueError("PA fuori intervallo (0..1)")
    ox = float(x_offset)
    zt = "G1 Z%s F600" % _f(TRAVEL_Z if zhop else 3.0)    # quota degli spostamenti

    def X(v):
        return _f(round(v + ox, 1))

    g = []
    a = g.append
    # --- preparazione, innesco e riquadri (identici al pattern di fabbrica se ox=0)
    for line in (
        "M140 S%.1f" % bed, "M190 S%.1f" % bed, "M204 S5000", "M109 S%.1f" % hotend,
        "G21", "G92 E0 ; Reset Extruder", "G1 F2400 E-0.8", "M220 S100", "M221 S100",
        "M205 X10 Y10", ("G1 Z%s F600" % _f(TRAVEL_Z)) if zhop else "G1 Z2.0 F600", "G1 X160 Y%s F6000.0" % _f(prime_y), "G1 Z0.20 F600.0",
        "G1 X%s Y%s F3000.0 E10" % (X(40), _f(prime_y)), "G92 E0", "G1 F2400 E-0.8",
        zt, "G1 F7800",
        "G90", "M83", "M106 S0",
        "SET_VELOCITY_LIMIT ACCEL=5000", "SET_VELOCITY_LIMIT ACCEL_TO_DECEL=2500",
        "SET_PRESSURE_ADVANCE ADVANCE=0.04",
        "G1 Y0 X%s F30000" % X(5)) + (
        ("G1 Z.2 F600", "G1 F2400 E0.8") if zhop else ("G1 F2400 E0.8", "G1 Z.2 F600")) + (
        "G1 Y0 X%s E1.25 F30000" % X(30), "G1 Y5 X%s E0.25 F1200" % X(30),
        "G1 Y5 X%s E1.25 F1200" % X(5), "G1 Y0 X%s E0.25 F1200" % X(5),
        "G1 F2400 E-0.8", zt,
        "G1 Y205 X%s F30000" % X(5), "G1 Z.2 F600", "G1 F2400 E0.8",
        "G1 Y205 X%s E1.3 F1200" % X(30), "G1 Y200 X%s E0.25 F1200" % X(30),
        "G1 Y200 X%s E1.3 F1200" % X(5), "G1 Y205 X%s E0.25 F1200" % X(5),
        "G1 F2400 E-0.8", zt):
        a(line)
    # --- 8 passate: per ogni nodo due passate affiancate (0.6 mm)
    first = True
    for n in range(NODES):
        for dx in (0.0, 0.6):
            hi = round(15.0 + 3 * n + dx + ox, 1)
            lo = round(hi - 12.5, 1)
            a("SET_PRESSURE_ADVANCE ADVANCE=0.04")
            if zhop:           # arrivo in quota, poi giu' e de-ritrazione
                a("G1 Y0 X%s F30000" % _f(hi))
                a("G1 Z.2 F600")
                a("G1 E0.8 F2400")
            elif first:        # la prima passata arriva dal riquadro alto, in quota
                a("G1 Y0 X%s F30000" % _f(hi))
                a("G1 E0.8 F2400")
                a("G1 Z.2 F600")
            else:
                a("G1 Z.2 F600")
                a("G1 X%s Y0 F2400" % _f(hi))
                a("G1 E0.8 F2400")
            first = False
            a("G1 X%s Y5 E0.5 F1200" % _f(hi))
            for r in range(ROWS):
                a("SET_PRESSURE_ADVANCE ADVANCE=%s" % _f(values[NODES * r + n]))
                a("G1 X%s Y%.1f E%s  F7800" % (_f(lo), 17.5 + 25 * r, SEG_E))
                a("G1 X%s Y%.1f E%s  F7800" % (_f(hi), 30.0 + 25 * r, SEG_E))
            a("SET_PRESSURE_ADVANCE ADVANCE=0.00")
            a("G1 E-0.8 F2400")
            if zhop:           # su subito, poi spostamento in quota
                a(zt)
                a("G0 X%s Y0 F30000" % X(30))
            else:
                a("G0 X%s Y205.0 F30000" % X(30))
                a("G0 X%s Y200.0 F30000" % X(30))
                a("G1 Z3 F600")
                a("G0 X%s Y0 F30000" % X(30))
    a("M400")
    return "\n".join(g) + "\n"


def moves(text):
    """Sequenza normalizzata per il confronto: (comando, X, Y, Z, E) e i valori di PA.
    Ignora feedrate, spazi e commenti."""
    import re
    out = []
    for line in text.splitlines():
        line = line.split(";")[0].strip()
        if not line:
            continue
        m = re.match(r"SET_PRESSURE_ADVANCE\s+ADVANCE=([0-9.]+)", line)
        if m:
            out.append(("PA", round(float(m.group(1)), 4)))
            continue
        w = line.split()
        if w[0] in ("G0", "G1"):
            d = {}
            for t in w[1:]:
                if t[0] in "XYZE":
                    d[t[0]] = round(float(t[1:]), 4)
            if d:
                out.append(("G", tuple(sorted(d.items()))))
        else:
            out.append((w[0],) + tuple(t for t in w[1:] if not t.startswith("F")))
    return out
