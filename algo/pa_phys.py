#!/usr/bin/env python3
# algo/pa_phys.py - analizzatore PA del toolkit. Sostituisce l'analizzatore di serie
# di Creality, che non viene usato.
#
# Cosa misura. Ogni nodo del pattern (un valore di PA) e' una V a 45 gradi: la testa
# frena fino allo spigolo basso e riaccelera. Con PA basso lo spigolo fa un grumo,
# con PA alto l'estrusore ritrae troppo in frenata e lo SPIGOLO SI SVUOTA. Misura:
# RIEMPIMENTO = volume del cordone allo spigolo / volume sul tratto dritto dello
# stesso nodo. Sui run veri (2026-11-07, PLA) cala in modo quasi LINEARE col PA, con
# pendenza ripetibile fra run diversi (-6.2..-7.1 per unita' di PA).
# PA scelto = quello a cui lo spigolo e' pieno quanto il dritto (riempimento 1):
# estrusione uniforme anche in curva. Regressione lineare su tutti i nodi insieme,
# con uno scarto per POSIZIONE del nodo nella riga (i nodi 3-4 escono sempre piu'
# bassi: effetto della lama/ombra, non del PA) a media nulla; intervallo col bootstrap.
# Il livello assoluto risente della qualita' di stampa: per questo il PA atteso deve
# cadere DENTRO i valori misurati (range per materiale), non essere estrapolato.
#
# Differenze rispetto all'originale Creality:
#  - usa tutte le righe insieme invece di scegliere prima "la riga migliore"
#    (sul riferimento di fabbrica le 8 righe distano ~2%: scelta quasi casuale);
#  - misura volumi (um*mm) nella posizione attesa di ogni nodo, anche dove il
#    cordone e' sottile o manca (proprio li' c'e' l'informazione);
#  - trova da solo gli spigoli (fit a V delle linee), niente finestre di frame fisse;
#  - dice quando il risultato non e' affidabile (ginocchio al bordo dell'intervallo,
#    nessun calo, intervallo largo) invece di restituire comunque un valore.
#
# Ingressi: i .temp della scansione (tabella = piatto nudo, linee = pattern)
# oppure i .dense.json di scan_sweep (tutte le catture con la Y vera). Solo numpy.
import json, sys
import numpy as np

H_MIN = 80.0          # um sopra il piatto = materiale
GAP_MERGE = 0.6       # mm: frammenti piu' vicini sono la stessa striscia
AREA_MIN = 200.0      # um*mm: sotto e' rumore/frammento
X_LIM = 17.5          # mm (x laser): bordo utile della lama
PITCH = (2.8, 4.8)    # mm: distanza fra strisce di nodi vicini (3 mm macchina, ~3.7 mm laser)
WIN = 9.2             # mm attorno allo spigolo basso (gli spigoli alti distano 12.5)
HW = 1.5              # mm: semi-larghezza della finestra x in cui si integra un nodo
CORNER = 1.2          # mm: zona dello spigolo
BASE = (5.0, 9.0)     # mm: tratto dritto (distanza dallo spigolo) per normalizzare
ROW1_Y = 40.8         # Y ugello attesa dello spigolo della riga 1 (17.5 + LASER_DY 23.3)
FRAME_Y0, FRAME_STEP = 27.2, 194.0 / 169    # Y ugello nominale dei 170 frame


def _flatten(tab, lin):
    """Profilo delle linee raddrizzato sulla retta del piatto nudo (come Creality)."""
    p1 = np.asarray(tab, float); p2 = np.asarray(lin, float)
    if p1.ndim != 2 or p2.ndim != 2:
        return None, None
    p1 = p1[np.abs(p1[:, 2]) < 32700]; p2 = p2[np.abs(p2[:, 2]) < 32700]
    if len(p1) < 120 or len(p2) < 10:
        return None, None
    k, b = np.polyfit(p1[50:-50, 0], p1[50:-50, 2], 1)
    th = np.arctan(k)
    c, s = np.cos(th), np.sin(th)
    x = p2[:, 0]; z = p2[:, 2] - b
    xr = x * c + z * s
    zr = -x * s + z * c
    o = np.argsort(xr)
    return xr[o] / 1000.0, -zr[o]          # mm, altezza in um (positiva = sopra)


def stripes(x, h):
    """Le 4 strisce del pattern nel frame: [centro_mm, larghezza_mm, area_um*mm] o []."""
    if x is None:
        return []
    on = h > H_MIN
    segs = []; s = None
    for j in range(len(x) + 1):
        a = j < len(x) and on[j]
        if a and s is None:
            s = j
        if not a and s is not None:
            if j - s >= 2:
                segs.append([x[s], x[j - 1], float(np.trapz(h[s:j], x[s:j]))])
            s = None
    m = []
    for x0, x1, ar in segs:
        if m and x0 - m[-1][1] < GAP_MERGE:
            m[-1][1] = x1; m[-1][2] += ar
        else:
            m.append([x0, x1, ar])
    c = [[(a + b) / 2, b - a, ar] for a, b, ar in m
         if ar > AREA_MIN and b - a > 0.8 and -X_LIM < (a + b) / 2 < X_LIM]
    best, bscore = [], None
    for k in range(len(c) - 3):                 # 4 consecutive a passo regolare
        q = c[k:k + 4]
        d = np.diff([s[0] for s in q])
        if d.min() < PITCH[0] or d.max() > PITCH[1]:
            continue
        score = float(np.std(d))
        if bscore is None or score < bscore:
            best, bscore = q, score
    return best


def _vfit(ys, xs):
    """Fit x = x0 + s*|y - yc| con s < 0 (spigolo basso = massimo di x, lama
    specchiata); ritorna (yc, residuo)."""
    best = (None, 1e18)
    for yc in np.arange(ys.min() + 2, ys.max() - 2, 0.05):
        A = np.column_stack([np.ones_like(ys), np.abs(ys - yc)])
        coef, *_ = np.linalg.lstsq(A, xs, rcond=None)
        r = float(np.sum((A @ coef - xs) ** 2))
        if r < best[1] and coef[1] < 0:
            best = (float(yc), r)
    return best


def _corners(Y, S):
    """Spigoli bassi: massimi locali della x media delle 4 strisce, rifiniti col fit a V."""
    ok = [i for i in range(len(Y)) if len(S[i]) == 4]
    if len(ok) < 10:
        return []
    yo = Y[ok]; cx = np.array([np.mean([s[0] for s in S[i]]) for i in ok])
    cand = []
    for k in range(len(ok)):
        near = np.abs(yo - yo[k]) <= WIN
        if cx[k] >= cx[near].max() - 1e-9 and cx[k] - cx[near].min() > 4.0:
            if not cand or yo[k] - cand[-1] > 15.0:
                cand.append(float(yo[k]))
    out = []
    for yc in cand:
        res = None
        for _ in range(3):
            sel = [i for i in ok if abs(Y[i] - yc) <= WIN]
            if len(sel) < 8:
                yc = None
                break
            xs = np.array([np.mean([s[0] for s in S[i]]) for i in sel])
            yc, res = _vfit(Y[sel], xs)
            if yc is None:
                break
        if yc is not None:
            out.append((yc, (res / len(sel)) ** .5, sel))
    return out


def analyze(table_path, line_path, values=None):
    """Sui .temp classici (un frame per chiave): Y = posizione nominale del frame."""
    t = json.load(open(table_path)); l = json.load(open(line_path))
    n = min(len(t), len(l))
    ys = [FRAME_Y0 + i * FRAME_STEP for i in range(n)]
    return analyze_frames(ys, [t.get("table%d" % i, []) for i in range(n)],
                          [l.get("scan%d" % i, []) for i in range(n)], values)


def analyze_dense(table_dense, line_dense, values=None):
    """Sulle catture dense (.dense.json di scan_sweep): Y vera di ogni cattura; per
    ogni cattura delle linee si usa la cattura del piatto nudo piu' vicina in Y."""
    t = json.load(open(table_dense)); l = json.load(open(line_dense))
    ty = np.array(t["y"])
    tabs = [t["frames"][int(np.argmin(np.abs(ty - y)))] for y in l["y"]]
    return analyze_frames(list(l["y"]), tabs, l["frames"], values)


def analyze_frames(ys, tabs, lins, values=None):
    vals = list(values) if values else [round(0.020 + 0.002 * k, 4) for k in range(32)]
    order = np.argsort(ys)
    Y = np.array(ys, float)[order]
    FL = [_flatten(tabs[i], lins[i]) for i in order]
    S = [stripes(*f) for f in FL]
    corners = _corners(Y, S)
    if len(corners) >= 2:
        period = float(np.median(np.diff([c[0] for c in corners])))
    else:
        period = 25.0
    rows, pts = [], []
    for yc, rms, sel in corners:
        r = int(round((yc - ROW1_Y) / period))
        if not 0 <= r < 8 or any(rw.get("row") == r for rw in rows):
            continue
        row = {"row": r, "corner_y": round(yc, 2), "fit_rms_mm": round(rms, 3), "nodes": []}
        rows.append(row)
        if rms > 0.6:
            row["skip"] = "linee irregolari (fit a V %.2fmm)" % rms
            continue
        # posizione di ogni nodo: x_j = c_j + s*|y - yc| (pendenza comune)
        A, b = [], []
        for i in sel:
            for j in range(4):
                e = np.zeros(5); e[j] = 1; e[4] = abs(Y[i] - yc)
                A.append(e); b.append(S[i][j][0])
        coef = np.linalg.lstsq(np.array(A), np.array(b), rcond=None)[0]
        win = [i for i in range(len(Y)) if abs(Y[i] - yc) <= WIN and FL[i][0] is not None]
        dy = Y[win] - yc
        prof = np.zeros((len(win), 4))
        for a_, i in enumerate(win):
            x, h = FL[i]
            for j in range(4):
                xc = coef[j] + coef[4] * abs(Y[i] - yc)
                m = (x > xc - HW) & (x < xc + HW)
                prof[a_, j] = np.trapz(np.clip(h[m], 0, None), x[m]) if m.sum() > 1 else 0.0
        base_m = (np.abs(dy) >= BASE[0]) & (np.abs(dy) <= BASE[1])
        corn_m = np.abs(dy) <= CORNER
        if base_m.sum() < 3 or corn_m.sum() < 1:
            row["skip"] = "troppo pochi frame attorno allo spigolo"
            continue
        for j in range(4):
            base = float(np.median(prof[base_m, j]))
            if base <= 0:
                continue
            fill = float(np.mean(prof[corn_m, j])) / base
            node = 3 - j                          # lama specchiata: x crescente = X macchina decrescente
            pa = vals[4 * r + node]
            pts.append((pa, node, fill, r))
            row["nodes"].append({"node": node, "pa": pa, "fill": round(fill, 3), "base": round(base)})
    rows.sort(key=lambda rw: rw["row"])
    return _fit(rows, pts, vals)


TARGET_FILL = 1.0     # spigolo pieno quanto il tratto dritto


def _linfit(x, node, y):
    """y = a + b*pa + d[nodo] (d a media nulla). Ritorna (a, b, d[4], sse)."""
    A = np.column_stack([np.ones_like(x), x] +
                        [(node == k).astype(float) - (node == 3).astype(float) for k in range(3)])
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    sse = float(np.sum((A @ c - y) ** 2))
    d = list(c[2:]) + [-float(sum(c[2:]))]
    return float(c[0]), float(c[1]), d, sse


def _fit(rows, pts, vals):
    out = {"rows": rows, "points": pts, "pa": None, "warnings": []}
    lo, hi = min(vals), max(vals)
    if len(pts) < 12:
        out["error"] = "troppi pochi nodi misurati (%d/32)" % len(pts)
        return out
    P = np.array(pts)
    x, node, y = P[:, 0], P[:, 1].astype(int), P[:, 2]
    a, b, d, sse = _linfit(x, node, y)
    sigma = (sse / max(1, len(x) - 5)) ** .5
    if b >= 0:
        out["error"] = "il riempimento dello spigolo non cala col PA: lettura non valida"
        return out
    pa = (TARGET_FILL - a) / b
    rng = np.random.RandomState(7)        # numpy della K1: niente default_rng
    bs = []
    for _ in range(300):
        i = rng.randint(0, len(x), len(x))
        aa, bb, _, _ = _linfit(x[i], node[i], y[i])
        if bb < 0:
            bs.append((TARGET_FILL - aa) / bb)
    p10, p50, p90 = (float(np.percentile(bs, q)) for q in (10, 50, 90))
    mlo, mhi = float(x.min()), float(x.max())
    step = (hi - lo) / 31.0
    out.update(pa=float(pa), pa_boot=p50, pa_lo=p10, pa_hi=p90, slope=b, node_bias=d,
               sigma=sigma, n=len(x), range=(lo, hi), measured=(mlo, mhi))
    if pa < mlo + step:
        out["warnings"].append("valore ESTRAPOLATO sotto i nodi misurati (%.4f): abbassa l'inizio "
                               "del range" % mlo)
    if pa > mhi - step:
        out["warnings"].append("valore ESTRAPOLATO sopra i nodi misurati (%.4f): alza la fine "
                               "del range" % mhi)
    # pendenza significativa? (errore standard della pendenza dalla regressione)
    xc = x - x.mean()
    se_b = sigma / max(1e-12, float(np.sqrt(np.sum(xc ** 2))))
    out["slope_t"] = abs(b) / se_b
    if out["slope_t"] < 4.0:
        out["warnings"].append("calo del riempimento poco netto (t=%.1f): lettura incerta"
                               % out["slope_t"])
    if p90 - p10 > 6 * step:
        out["warnings"].append("intervallo di confidenza largo (%.4f-%.4f)" % (p10, p90))
    out["confident"] = not out["warnings"]
    return out


def report(r):
    lines = []
    for row in r["rows"]:
        head = "riga %d spigolo Y%.2f (fit %.2fmm)" % (row["row"] + 1, row["corner_y"], row["fit_rms_mm"])
        if row.get("skip"):
            lines.append("%s: SCARTATA, %s" % (head, row["skip"]))
            continue
        lines.append("%s: %s" % (head, "  ".join(
            "PA%.4f %.2f" % (nd["pa"], nd["fill"]) for nd in sorted(row["nodes"], key=lambda d: d["pa"]))))
    if r.get("pa") is not None:
        lines.append("PA = %.4f  (80%%: %.4f-%.4f)  pendenza %.1f (t=%.1f), rumore %.2f, nodi %d, "
                     "scarto posizione nodi %s" % (r["pa"], r["pa_lo"], r["pa_hi"], r["slope"],
                                                   r.get("slope_t", 0),
                                                   r["sigma"], r["n"],
                                                   " ".join("%+.2f" % v for v in r["node_bias"])))
    else:
        lines.append("nessun risultato: %s" % r.get("error"))
    for w in r.get("warnings", []):
        lines.append("ATTENZIONE: " + w)
    return "\n".join(lines)


if __name__ == "__main__":
    vals = None
    if "--values" in sys.argv:
        vals = [float(v) for v in sys.argv[sys.argv.index("--values") + 1].split(",")]
    if sys.argv[1].endswith(".dense.json"):
        res = analyze_dense(sys.argv[1], sys.argv[2], vals)
    else:
        res = analyze(sys.argv[1], sys.argv[2], vals)
    print(report(res))
