# -*- coding: utf-8 -*-
# fl_analyze.py - Analisi quantitativa primo strato LiDAR K1 (C2), multi-passata.
#
# Mappa delle altezze del primo strato, allineata al gcode; cerca zone mancanti o sollevate.
# Vincoli rispettati: Python 3.8, solo numpy (niente scipy/sklearn), costi
# compatibili con MIPS (maschera per stamping, registrazione su ~200 valutazioni
# di correlazione su ~10^3 celle, nessuna matrice N_punti x N_segmenti).
#
# Convenzioni:
#  - unita' interne del sensore: um. z piu' NEGATIVO = piu' ALTO.
#  - "elev" = elevazione sopra il piatto in um, POSITIVA per il materiale:
#       elev = -((z_layer - z_table) - dz_reg)
#  - coordinate macchina in mm. X_mach = (X_nozzle - C_NOM) - x_int/1000 + dx_reg
#    (mirror), Y_mach(frame i) = Y0 + i*step + dy_reg.
#
# Uso tipico:
#   res = analyze(bands, segs_all, layer_h_mm=0.20)
#   print(format_report(res))
# dove bands = [{"table": frames, "layer": frames, "x_nozzle": mm}, ...]
# e frames = lista di array (N,3) [x_int_um, y_int_um, z_um] (vuoti ammessi).

import math
import numpy as np

# ---------------------------------------------------------------- costanti ---
C_NOM = 37.2         # centro ottico nominale: X_mach = X_nozzle - C_NOM - x_int
                     # (auto-ricalibrato dalla registrazione, vedi dx_reg)
SAT_UM = 32700       # |z| >= SAT = nessuna lettura
X_INT_TRUST_UM = 17000   # oltre: code della lama sottocampionate -> non fidarsi
GRID_RES = 0.25      # mm, raster maschera gcode
XBIN_MM = 1.0        # mm, larghezza bin X delle celle
MIN_PTS_BIN = 3      # punti minimi per bin per fidarsi della mediana
REG_DX_MM = 4.0      # semi-finestra ricerca dx
REG_DY_MM = 3.0      # semi-finestra ricerca dy
REG_MIN_CORR = 0.40  # sotto: registrazione non affidabile -> INCERTO
W_LINE_DEFAULT = 0.50    # mm, larghezza linea estrusa (parametro slicer)
MASK_TOL = 0.30      # mm, tolleranza registrazione residua sulla maschera "on"
FAR_MM = 1.0         # mm, distanza minima dai segmenti per "piatto sicuro"
INTERIOR_FRAC = 0.8  # frazione di footprint su maschera per cella "interna"

# ---------------------------------------------------------------- utilita' ---

def robust_sigma(v):
    """sigma robusta = 1.4826 * MAD. Insensibile a outlier (riflessi residui)."""
    v = np.asarray(v, dtype=np.float64)
    if len(v) == 0:
        return 0.0
    med = np.median(v)
    return 1.4826 * np.median(np.abs(v - med))


def _stamp_disk(res, radius):
    r = int(math.ceil(radius / res))
    off = []
    for iy in range(-r, r + 1):
        for ix in range(-r, r + 1):
            if (ix * res) ** 2 + (iy * res) ** 2 <= radius ** 2:
                off.append((iy, ix))
    return np.array(off, dtype=np.int64)


class GcodeMask(object):
    """Maschere raster dalla edge-list dei segmenti di estrusione (mm).

    near : entro w/2 + MASK_TOL da un segmento ("qui DEVE esserci materiale,
           a meno di errore di registrazione residuo")
    far  : oltre FAR_MM da ogni segmento ("qui NON deve esserci materiale":
           piatto nudo sicuro, usato per la registrazione in z)
    Costruzione per stamping di dischi lungo i segmenti: O(lunghezza/res)
    invece di O(celle x segmenti) -> fattibile su MIPS.
    """

    def __init__(self, segs, w_line=W_LINE_DEFAULT, margin=4.5, res=GRID_RES,
                 r_stray=None):
        segs = np.asarray(segs, dtype=np.float64)
        self.res = res
        self.x0 = segs[:, [0, 2]].min() - margin
        self.y0 = segs[:, [1, 3]].min() - margin
        nx = int((segs[:, [0, 2]].max() + margin - self.x0) / res) + 2
        ny = int((segs[:, [1, 3]].max() + margin - self.y0) / res) + 2
        self.near = np.zeros((ny, nx), dtype=bool)
        far_block = np.zeros((ny, nx), dtype=bool)
        stray_block = np.zeros((ny, nx), dtype=bool) \
            if r_stray and r_stray > FAR_MM else None
        d_on = _stamp_disk(res, w_line / 2.0 + MASK_TOL)
        d_far = _stamp_disk(res, FAR_MM)
        d_str = _stamp_disk(res, r_stray) if stray_block is not None else None
        step = res / 2.0
        for x0, y0, x1, y1 in segs:
            L = math.hypot(x1 - x0, y1 - y0)
            n = max(1, int(L / step) + 1)
            ts = np.linspace(0.0, 1.0, n)
            iys = ((y0 + ts * (y1 - y0) - self.y0) / res).astype(np.int64)
            ixs = ((x0 + ts * (x1 - x0) - self.x0) / res).astype(np.int64)
            targets = [(self.near, d_on), (far_block, d_far)]
            if stray_block is not None:
                targets.append((stray_block, d_str))
            for dgrid, dsk in targets:
                yy = (iys[:, None] + dsk[None, :, 0]).ravel()
                xx = (ixs[:, None] + dsk[None, :, 1]).ravel()
                ok = (yy >= 0) & (yy < ny) & (xx >= 0) & (xx < nx)
                dgrid[yy[ok], xx[ok]] = True
        self.far = ~far_block
        self.far_stray = ~stray_block if stray_block is not None else self.far

    def _idx(self, X, Y):
        ix = np.clip(((np.asarray(X) - self.x0) / self.res).astype(np.int64),
                     0, self.near.shape[1] - 1)
        iy = np.clip(((np.asarray(Y) - self.y0) / self.res).astype(np.int64),
                     0, self.near.shape[0] - 1)
        return iy, ix

    def near_at(self, X, Y):
        iy, ix = self._idx(X, Y)
        return self.near[iy, ix]

    def far_at(self, X, Y):
        iy, ix = self._idx(X, Y)
        return self.far[iy, ix]

    def far_stray_at(self, X, Y):
        iy, ix = self._idx(X, Y)
        return self.far_stray[iy, ix]

    def interior_frac(self, xc, yc, wx=XBIN_MM, wy=1.41):
        """Frazione del footprint (wx x wy) della cella dentro 'near'.
        Campionamento 5x5: 25 lookup, solo per le celle da classificare."""
        oxs = np.linspace(-wx / 2, wx / 2, 5)
        oys = np.linspace(-wy / 2, wy / 2, 5)
        OX, OY = np.meshgrid(oxs, oys)
        XX = np.asarray(xc, dtype=np.float64)[:, None] + OX.ravel()[None, :]
        YY = np.asarray(yc, dtype=np.float64)[:, None] + OY.ravel()[None, :]
        v = self.near_at(XX.ravel(), YY.ravel()).reshape(len(XX), 25)
        return v.mean(axis=1)


# ------------------------------------------------------------ binning bande ---

def _band_bins(table, layer, x_nozzle, y_rows):
    """Mediane accoppiate per (frame, bin X da XBIN_MM) di tabella e strato.

    Ritorna dict di array paralleli: frame, xc (mm, coordinate NOMINALI pre-
    registrazione), zt, zl (um), nt, nl, x_int (um, mediana lato layer),
    yl (um, mediana y_int lato layer: serve alla correzione Y per-bin).
    Solo bin con >= MIN_PTS_BIN punti in ENTRAMBI gli scan e |x_int| entro la
    zona affidabile della lama. I confronti z avvengono SOLO qui, a x
    accoppiate: il profilo z(x) della lama (fino a ~0.2mm di curvatura) si
    cancella per differenza; confrontare x diverse lo farebbe entrare intero
    nell'errore.
    """
    C0 = x_nozzle - C_NOM
    out = {k: [] for k in ("frame", "xc", "zt", "zl", "nt", "nl", "x_int", "yl")}
    for i, (ft, fl) in enumerate(zip(table, layer)):
        if len(ft) == 0 or len(fl) == 0:
            continue
        ft = ft[np.abs(ft[:, 0]) <= X_INT_TRUST_UM]
        fl = fl[np.abs(fl[:, 0]) <= X_INT_TRUST_UM]
        if len(ft) < MIN_PTS_BIN or len(fl) < MIN_PTS_BIN:
            continue
        Xt = C0 - ft[:, 0] / 1000.0
        Xl = C0 - fl[:, 0] / 1000.0
        lo = math.floor(max(Xt.min(), Xl.min()))
        hi = math.ceil(min(Xt.max(), Xl.max()))
        edges = np.arange(lo, hi + XBIN_MM, XBIN_MM)
        it = np.digitize(Xt, edges)
        il = np.digitize(Xl, edges)
        for j in range(1, len(edges)):
            mt = it == j
            ml = il == j
            if mt.sum() < MIN_PTS_BIN or ml.sum() < MIN_PTS_BIN:
                continue
            out["frame"].append(i)
            out["xc"].append(edges[j - 1] + XBIN_MM / 2.0)
            out["zt"].append(np.median(ft[mt, 2]))
            out["zl"].append(np.median(fl[ml, 2]))
            out["nt"].append(int(mt.sum()))
            out["nl"].append(int(ml.sum()))
            out["x_int"].append(np.median(fl[ml, 0]))
            out["yl"].append(np.median(fl[ml, 1]))
    return {k: np.asarray(v, dtype=np.float64) for k, v in out.items()}


def _bin_Y(bins, y_rows, y_ref_um):
    """Y macchina per bin: riga del frame + parte VARIABILE di y_int.

    y_int e' una misura fisica: y_int = skew(x) - 0.949*z (accoppiamento di
    triangolazione misurato, costante su bande e scan). La lama e' inclinata
    di ~0.9mm da un estremo all'altro in X e i punti piu' alti vengono
    campionati a Y spostata: entrambe le cose sono dentro y_int. La componente
    COSTANTE (mediana globale y_ref) e' indistinguibile da un errore di
    LASER_DY e viene assorbita da dy_reg; qui si applica solo la variazione.
    """
    return y_rows[bins["frame"].astype(np.int64)] + \
        (bins["yl"] - y_ref_um) / 1000.0


def _register_joint(bins_list, Y_list, mask, coarse=0.5, fine=0.25,
                    jitter=0.5):
    """Registrazione CONGIUNTA di tutte le bande: un solo (dx, dy) rigido.

    Perche' congiunta: dx e' l'errore del centro ottico (montaggio) e dy
    l'errore di LASER_DY piu' il bias di trigger - entrambi condivisi da tutte
    le bande dello stesso run. Stimarli per banda regala gradi di liberta' al
    rumore: su una banda con poca "struttura" (features deboli) la superficie
    di score e' piatta e il massimo locale puo' cadere a 0.5-1mm dal vero
    (osservato sui dati reali: banda 0 da sola sceglieva dy=-1.5 contro il
    -2.0 vero, producendo falsi 'mancante'). La banda ricca di struttura
    vincola la soluzione per tutte.

    Poi un raffinamento dy PER BANDA limitato a +-jitter intorno al dy
    congiunto modella l'unica differenza fisica reale tra bande: il jitter di
    trigger della singola sweep (~0.2-0.4mm), senza restituire al rumore la
    liberta' appena tolta.

    Score = correlazione punto-biseriale maschera vs elevazione: invariante a
    offset/scala di z, quindi non richiede di conoscere dz ne' l'altezza vera.
    Ritorna (dx, dy_per_banda, corr_pooled, corr_per_banda).
    """
    elevs = [-(b["zl"] - b["zt"]) for b in bins_list]
    pool_e = np.concatenate(elevs)
    if len(pool_e) < 20 or pool_e.std() < 1e-9:
        return 0.0, [0.0] * len(bins_list), 0.0, [0.0] * len(bins_list)

    def pooled_score(dx, dy):
        ms = [mask.near_at(b["xc"] + dx, Y + dy).astype(np.float64)
              for b, Y in zip(bins_list, Y_list)]
        m = np.concatenate(ms)
        if m.std() < 1e-9:
            return -1.0
        return float(np.corrcoef(m, pool_e)[0, 1])

    best = (0.0, 0.0, pooled_score(0.0, 0.0))
    for dx in np.arange(-REG_DX_MM, REG_DX_MM + 1e-9, coarse):
        for dy in np.arange(-REG_DY_MM, REG_DY_MM + 1e-9, coarse):
            s = pooled_score(dx, dy)
            if s > best[2]:
                best = (dx, dy, s)
    bx, by, _ = best
    for dx in np.arange(bx - coarse, bx + coarse + 1e-9, fine):
        for dy in np.arange(by - coarse, by + coarse + 1e-9, fine):
            s = pooled_score(dx, dy)
            if s > best[2]:
                best = (dx, dy, s)
    dx0, dy0, corr_pool = best

    def band_score(k, dy):
        m = mask.near_at(bins_list[k]["xc"] + dx0,
                         Y_list[k] + dy).astype(np.float64)
        if m.std() < 1e-9:
            return -1.0
        return float(np.corrcoef(m, elevs[k])[0, 1])

    dys, corrs = [], []
    for k in range(len(bins_list)):
        s0 = band_score(k, dy0)
        bk, sk = dy0, s0
        for dy in np.arange(dy0 - jitter, dy0 + jitter + 1e-9, fine):
            s = band_score(k, dy)
            if s > sk:
                bk, sk = dy, s
        if sk < s0 + 0.02:            # migliorie sotto il rumore: non pagare
            bk, sk = dy0, s0          # il grado di liberta'
        dys.append(bk)
        corrs.append(sk)
    return dx0, dys, corr_pool, corrs


def _profile(fr, bin_um=500.0):
    """Profilo del frame lungo la lama: bin di x_int -> (centri, z_med, y_med)."""
    if len(fr) < 6:
        return np.zeros(0), np.zeros(0), np.zeros(0)
    x = fr[:, 0]
    ed = np.arange(x.min(), x.max() + bin_um, bin_um)
    ib = np.digitize(x, ed)
    c, z, y = [], [], []
    for j in range(1, len(ed)):
        m = ib == j
        if m.sum() >= 3:
            c.append((ed[j - 1] + ed[j]) / 2.0)
            z.append(np.median(fr[m, 2]))
            y.append(np.median(fr[m, 1]))
    return np.array(c), np.array(z), np.array(y)


def _edge_residuals(table, layer, C0, dx, dyb, y_rows, y_ref, mask,
                    thr_um=150.0):
    """Coppie (x_int_mm, errore_mm) dai FRONTI materiale<->piatto.

    Per ogni frame: profilo elev(x_int) a 0.5mm; ogni transizione sostenuta
    (2 bin sotto soglia | 2 bin sopra) e' un bordo fisico del materiale; il
    bordo maschera piu' vicino lungo X (entro +-3mm) alla stessa Y dice dove
    DOVREBBE stare. err = X_mask - X_stimata. A differenza della correlazione
    per finestre di x_int, il fronte non soffre di aliasing con il passo delle
    linee: misura la distorsione del mapping x_int->X direttamente.
    """
    out = []
    for i, (ft, fl) in enumerate(zip(table, layer)):
        ct, zt, _ = _profile(ft)
        cl, zl, yl = _profile(fl)
        if len(ct) < 5 or len(cl) < 5:
            continue
        lo, hi = ct.min(), ct.max()
        keep = (cl >= lo) & (cl <= hi) & (np.abs(cl) <= X_INT_TRUST_UM)
        cl, zl, yl = cl[keep], zl[keep], yl[keep]
        if len(cl) < 5:
            continue
        elev = -(zl - np.interp(cl, ct, zt))
        mat = elev >= thr_um
        for j in range(1, len(cl) - 2):
            if cl[j + 1] - cl[j] > 900:
                continue                       # buco dati: fronte non definito
            if mat[j - 1] == mat[j] and mat[j + 1] == mat[j + 2] \
                    and mat[j] != mat[j + 1]:
                xi = (cl[j] + cl[j + 1]) / 2.0
                X_est = C0 - xi / 1000.0 + dx
                Ye = y_rows[i] + dyb + \
                    ((yl[j] + yl[j + 1]) / 2.0 - y_ref) / 1000.0
                xs = np.arange(X_est - 3.0, X_est + 3.0, 0.05)
                mm = mask.near_at(xs, np.full(len(xs), Ye))
                tr = np.where(mm[:-1] != mm[1:])[0]
                if len(tr) == 0:
                    continue
                Xm = xs[tr[np.argmin(np.abs(xs[tr] - X_est))]]
                out.append((xi / 1000.0, float(Xm - X_est)))
    return out


def _fit_distortion(pairs, min_edges=10, min_span=8.0):
    """Correzione di distorsione err(x_int) [mm] dai fronti, quadratica robusta.

    La distorsione della lama e' una proprieta' FISSA del sensore (lente +
    triangolazione): sui dati reali i fronti delle due bande cadono sulla
    stessa curva (misurato: da -2.0mm a x_int=-14 a +0.3mm a x_int=+6). Fit su
    MEDIANE per finestre di 2mm (robusto agli accoppiamenti fronte-bordo
    sbagliati), pesato dal numero di fronti. Nessuna estrapolazione: fuori dal
    range coperto la correzione e' bloccata al valore del bordo.
    Ritorna dict {"coef","lo","hi","rms","n"} o None se dati insufficienti.
    """
    if len(pairs) < min_edges:
        return None
    xi = np.array([p[0] for p in pairs])
    er = np.array([p[1] for p in pairs])
    good = np.abs(er) <= 2.5
    xi, er = xi[good], er[good]
    if len(xi) < min_edges or xi.max() - xi.min() < min_span:
        return None
    wx, wy, wn = [], [], []
    for lo in np.arange(np.floor(xi.min()), xi.max() + 2.0, 2.0):
        m = (xi >= lo) & (xi < lo + 2.0)
        # le code estreme della lama (|xi|>15) sono sottocampionate: i fronti
        # li' sono pochi e sbilanciati, non li lasciamo tirare il fit
        if m.sum() >= 2 and abs(lo + 1.0) <= 15.0:
            wx.append(lo + 1.0)
            wy.append(np.median(er[m]))
            wn.append(m.sum())
    if len(wx) < 4:
        return None
    wx, wy, wn = np.array(wx), np.array(wy), np.array(wn)
    keep = np.ones(len(wx), dtype=bool)
    coef, rms = None, None
    for _ in range(3):                  # refit robusto: 2 passate di scarto
        if keep.sum() < 4:
            return None
        coef = np.polyfit(wx[keep], wy[keep], 2, w=np.sqrt(wn[keep]))
        res = np.polyval(coef, wx) - wy
        rms = float(np.sqrt(np.mean(res[keep] ** 2)))
        new_keep = np.abs(res) <= max(1.5 * rms, 0.35)
        if (new_keep == keep).all():
            break
        keep = new_keep
    if rms is None or rms > 0.45:
        return None                     # curva non coerente: meglio niente
    if wx[keep].max() - wx[keep].min() < min_span:
        return None
    return {"coef": coef.tolist(), "lo": float(wx[keep].min()),
            "hi": float(wx[keep].max()), "rms": rms, "n": int(len(xi)),
            "windows": int(keep.sum())}


def _dist_eval(dist, xi_mm):
    """Valuta err(x_int) con clamp del dominio (niente estrapolazione)."""
    if dist is None:
        return np.zeros(np.shape(xi_mm))
    x = np.clip(np.asarray(xi_mm, dtype=np.float64), dist["lo"], dist["hi"])
    return np.polyval(np.asarray(dist["coef"]), x)


def _register_z(bins, X, Y, mask, n_frames):
    """dz per frame dai bin 'far' (piatto nudo in entrambi gli scan).

    dz cattura la deriva sistematica tabella->strato (espansione termica del
    piatto: il piatto era freddo per la tabella e caldo dopo la stampa; misurata
    ~-50um su questi dati). Per-frame se il frame ha >= 4 bin far (media' via
    la componente lenta in Y), altrimenti si usa il globale.
    Ritorna (dz_per_frame, dz_globale, sigma_cella, n_far).
    """
    far = mask.far_at(X, Y)
    d = bins["zl"] - bins["zt"]
    if far.sum() < 8:
        return np.zeros(n_frames), 0.0, 0.0, int(far.sum())
    dz_g = float(np.median(d[far]))
    sig = float(robust_sigma(d[far]))
    dz_f = np.full(n_frames, dz_g)
    fr = bins["frame"].astype(np.int64)
    for i in range(n_frames):
        m = far & (fr == i)
        if m.sum() >= 4:
            dz_f[i] = np.median(d[m])
    return dz_f, dz_g, sig, int(far.sum())


# ------------------------------------------------------------------ analisi ---

def _tail_weight(x_int_um):
    """Peso di affidabilita' della posizione nella lama: 1 al centro, scende
    linearmente nelle code sottocampionate (>12mm), 0.2 all'estremo."""
    a = np.abs(np.asarray(x_int_um, dtype=np.float64))
    w = np.ones_like(a)
    t = a > 12000
    w[t] = np.clip(1.0 - (a[t] - 12000) / 7000.0 * 0.8, 0.2, 1.0)
    return w


def _flood_clusters(cells):
    """Cluster 8-connessi su celle {(row, col)}. Ritorna liste di celle.
    Poche centinaia di celle -> BFS puro Python e' piu' che sufficiente."""
    todo = set(cells)
    out = []
    while todo:
        seed = todo.pop()
        comp = [seed]
        stack = [seed]
        while stack:
            r, c = stack.pop()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nb = (r + dr, c + dc)
                    if nb in todo:
                        todo.remove(nb)
                        comp.append(nb)
                        stack.append(nb)
        out.append(comp)
    return out


def analyze(bands, segs_all, layer_h_mm, y0_mm, y_step_mm, n_frames,
            w_line=W_LINE_DEFAULT, elev_bias_margin_um=None,
            stray_ring_mm=None):
    """Analisi completa multi-passata.

    bands: [{"table": frames, "layer": frames, "x_nozzle": mm}, ...]
    segs_all: edge-list COMPLETA (non clippata) dei segmenti layer-1 in mm (M,4)
    layer_h_mm: altezza layer dal gcode
    y0_mm, y_step_mm, n_frames: geometria della sweep (frame i -> y0 + i*step)
    stray_ring_mm: se la edge-list e' INCOMPLETA (es. skirt/brim clippati via),
        distanza minima dai segmenti perche' materiale fuori maschera conti
        come 'inatteso' (es. 4.0: ignora l'anello dove vive la skirt). None =
        edge-list completa, si usa FAR_MM.

    Ritorna dict con verdict in {"GOOD","BAD","UNCERTAIN"}, reasons, warnings,
    metrics, e la mappa celle fusa.
    """
    LH = layer_h_mm * 1000.0
    y_rows = y0_mm + np.arange(n_frames) * y_step_mm
    R = {"verdict": "UNCERTAIN", "reasons": [], "warnings": [], "metrics": {},
         "bands": [], "cells": {}}

    # ---- guardie di acquisizione (il "mai piu' OK su scan vuoto") ----
    for bi, b in enumerate(bands):
        for nm in ("table", "layer"):
            frames = b[nm]
            npts = sum(len(f) for f in frames)
            n_empty = sum(1 for f in frames if len(f) == 0)
            if npts == 0:
                R["reasons"].append("scan %s banda %d VUOTO" % (nm, bi))
            elif n_empty > len(frames) // 2:
                R["reasons"].append("scan %s banda %d: %d/%d frame vuoti"
                                    % (nm, bi, n_empty, len(frames)))
    if R["reasons"]:
        return R

    mask = GcodeMask(segs_all, w_line=w_line, r_stray=stray_ring_mm)

    # ---- bin per banda ----
    band_bins = []
    for bi, b in enumerate(bands):
        bins = _band_bins(b["table"], b["layer"], b["x_nozzle"], y_rows)
        if len(bins["frame"]) < 20:
            R["reasons"].append("banda %d: %d bin utilizzabili (<20)"
                                % (bi, len(bins["frame"])))
            continue
        band_bins.append((bi, b, bins))
    if not band_bins:
        return R

    # ---- registrazione congiunta (un solo dx,dy rigido + jitter dy/banda) ----
    y_ref = float(np.median(np.concatenate([bb[2]["yl"] for bb in band_bins])))
    Y_list = [_bin_Y(bb[2], y_rows, y_ref) for bb in band_bins]
    dx, dys, corr_pool, corrs = _register_joint(
        [bb[2] for bb in band_bins], Y_list, mask)
    R["metrics"]["reg_corr_pooled"] = corr_pool

    # ---- distorsione della lama g(x_int): calibrata dai fronti ----
    # (proprieta' fissa del sensore; sui dati reali ~2.3mm di escursione tra
    # x_int -14 e +6. Senza correggerla, l'overlap fonde letture della stessa
    # X macchina prese da punti-lama diversi => celle miste piatto/materiale.)
    pairs = []
    for (bi, b, bins), Yb, dyb in zip(band_bins, Y_list, dys):
        pairs += _edge_residuals(b["table"], b["layer"], b["x_nozzle"] - C_NOM,
                                 dx, dyb, y_rows, y_ref, mask)
    dist = _fit_distortion(pairs)
    R["metrics"]["distortion"] = dist
    R["metrics"]["n_edges"] = len(pairs)
    ux = 0.5 if dist is not None else 1.0     # incertezza X residua per cella
    if dist is None:
        R["warnings"].append(
            "distorsione lama non calibrabile (%d fronti): mapping X con "
            "incertezza +-1mm, classificazione bordi allargata" % len(pairs))

    band_out = []
    for (bi, b, bins), Yb, dyb, corrb in zip(band_bins, Y_list, dys, corrs):
        Yreg = Yb + dyb
        Xreg = bins["xc"] + dx + _dist_eval(dist, bins["x_int"] / 1000.0)
        dz_f, dz_g, sig, n_far = _register_z(bins, Xreg, Yreg, mask, n_frames)
        band_out.append({"i": bi, "bins": bins, "Y": Yreg, "X": Xreg,
                         "dx": dx, "dy": dyb, "corr": corrb, "dz_f": dz_f,
                         "dz_g": dz_g, "sigma": sig, "n_far": n_far,
                         "C_eff": b["x_nozzle"] - C_NOM + dx})
    R["bands"] = [{k: v for k, v in bo.items() if k not in ("bins", "Y", "X")}
                  for bo in band_out]

    # guardie di registrazione
    if corr_pool < REG_MIN_CORR:
        R["reasons"].append(
            "registrazione inaffidabile (corr pooled %.2f < %.2f): mappa "
            "non allineabile al gcode" % (corr_pool, REG_MIN_CORR))
    for bo in band_out:
        if bo["corr"] < REG_MIN_CORR:
            R["warnings"].append(
                "banda %d: correlazione registrazione bassa (%.2f)"
                % (bo["i"], bo["corr"]))
        if bo["n_far"] < 8:
            R["warnings"].append(
                "banda %d: solo %d bin di piatto nudo per dz (uso dz=%.0f)"
                % (bo["i"], bo["n_far"], bo["dz_g"]))
    if R["reasons"]:
        # la registrazione fallisce anche quando NON c'e' struttura da
        # allineare: strato assente o completamente schiacciato. Diagnosi
        # indicativa dal delta globale (mediana su tutti i bin, gia' al netto
        # del dz termico stimato sui bin lontani dai segmenti).
        try:
            eg = float(np.median([np.median(
                -(bo["bins"]["zl"] - bo["bins"]["zt"] - bo["dz_g"]))
                for bo in band_out]))
            R["metrics"]["elev_global_um"] = eg
            if abs(eg) < 0.35 * LH:
                R["reasons"].append(
                    "indizio: delta globale %.0f um contro layer atteso "
                    "%.0f um -> possibile strato ASSENTE o completamente "
                    "schiacciato" % (eg, LH))
        except Exception:
            pass
        return R

    # sigma di misura per cella: la peggiore tra le bande (conservativo)
    sigma = max(bo["sigma"] for bo in band_out)
    sigma = max(sigma, 15.0)     # pavimento: mai fidarsi di sigma ~0

    # ---- celle in coordinate macchina registrate + fusione bande ----
    # chiave cella: (frame_row, colonna_x_globale). L'overlap tra bande finisce
    # sulla stessa chiave e viene fuso a media pesata (peso = n punti *
    # affidabilita' posizione lama). Prima della fusione stimo il disaccordo
    # sistematico tra bande nell'overlap e lo compenso simmetricamente su dz.
    cell_acc = {}
    for bo in band_out:
        bins = bo["bins"]
        fr = bins["frame"].astype(np.int64)
        Xr = bo["X"]
        Yr = bo["Y"]
        elev = -((bins["zl"] - bins["zt"]) - bo["dz_f"][fr])
        wgt = bins["nl"] * _tail_weight(bins["x_int"])
        col = np.floor(Xr / XBIN_MM).astype(np.int64)
        for k in range(len(fr)):
            key = (int(fr[k]), int(col[k]))
            cell_acc.setdefault(key, []).append(
                (bo["i"], float(elev[k]), float(wgt[k]),
                 float(Xr[k]), float(Yr[k])))

    # disaccordo tra bande nell'overlap
    if len(band_out) >= 2:
        diffs = []
        for key, lst in cell_acc.items():
            bs = {e[0] for e in lst}
            if len(bs) >= 2:
                by_band = {}
                for e in lst:
                    by_band.setdefault(e[0], []).append(e[1])
                vals = [np.mean(v) for v in by_band.values()]
                diffs.append(vals[0] - vals[1])
        if len(diffs) >= 6:
            d_ov = float(np.median(diffs))
            R["metrics"]["overlap_diff_um"] = d_ov
            R["metrics"]["overlap_cells"] = len(diffs)
            if abs(d_ov) > max(3 * sigma, 100.0):
                R["warnings"].append(
                    "overlap tra bande discorde: %.0f um (sigma %.0f)"
                    % (d_ov, sigma))
            # compensazione simmetrica del bias z tra le due bande
            corr_b = {band_out[0]["i"]: -d_ov / 2.0,
                      band_out[1]["i"]: +d_ov / 2.0}
            for key, lst in cell_acc.items():
                cell_acc[key] = [(b_, e + corr_b.get(b_, 0.0), w_, x_, y_)
                                 for (b_, e, w_, x_, y_) in lst]

    # fusione a media pesata
    cells = {}
    for key, lst in cell_acc.items():
        wsum = sum(e[2] for e in lst)
        elev = sum(e[1] * e[2] for e in lst) / wsum
        xm = sum(e[3] * e[2] for e in lst) / wsum
        ym = sum(e[4] * e[2] for e in lst) / wsum
        cells[key] = {"elev": elev, "w": wsum, "X": xm, "Y": ym}

    # ---- classificazione celle ----
    keys = list(cells.keys())
    Xc = np.array([cells[k]["X"] for k in keys])
    Yc = np.array([cells[k]["Y"] for k in keys])
    E = np.array([cells[k]["elev"] for k in keys])
    # footprint allargato: una cella e' "interna" solo se la maschera resta
    # piena anche spostando X/Y dell'incertezza residua di mapping (+-ux in X,
    # +-0.3mm in Y) - cosi' i bordi delle feature non generano falsi
    # 'mancante' per quantizzazione della sweep o residuo di distorsione.
    ifrac = mask.interior_frac(Xc, Yc, wx=XBIN_MM + 2 * ux, wy=y_step_mm + 0.6)
    on_edge = (ifrac > 0.2) & (ifrac < INTERIOR_FRAC)
    interior = ifrac >= INTERIOR_FRAC
    off = mask.far_at(Xc, Yc)
    off_stray = mask.far_stray_at(Xc, Yc)

    n_int = int(interior.sum())
    if n_int < 30:
        R["reasons"].append("solo %d celle interne alla maschera (<30): "
                            "oggetto troppo piccolo o scan fuori zona" % n_int)
        return R

    elev_med = float(np.median(E[interior]))
    theta_det = max(3.5 * sigma, 0.4 * max(elev_med, 0.0))
    theta_warp = max(4.0 * sigma, 0.6 * LH)

    material = E >= theta_det
    plate_lv = np.abs(E) < theta_det
    below = E <= -max(3.5 * sigma, 150.0)

    # copertura dello scan rispetto alla maschera (celle attese mai misurate)
    # (colonne x della griglia dentro il range scandito, righe = frame)
    have = set(keys)
    expected, unscanned = 0, 0
    col_lo = int(math.floor(Xc.min() / XBIN_MM))
    col_hi = int(math.floor(Xc.max() / XBIN_MM))
    for i in range(n_frames):
        yr = float(np.median(Yc[np.array([k[0] for k in keys]) == i])) \
            if any(k[0] == i for k in keys) else (y_rows[i])
        for c in range(col_lo, col_hi + 1):
            xc_ = (c + 0.5) * XBIN_MM
            if mask.interior_frac(np.array([xc_]), np.array([yr]),
                                  XBIN_MM + 2 * ux,
                                  y_step_mm + 0.6)[0] >= INTERIOR_FRAC:
                expected += 1
                if (i, c) not in have:
                    unscanned += 1
    unscanned_frac = unscanned / max(1, expected)

    # ---- metriche ----
    cov = float(material[interior].mean())
    missing_keys = [keys[j] for j in range(len(keys))
                    if interior[j] and plate_lv[j]]
    warp_keys = [keys[j] for j in range(len(keys))
                 if (interior[j] or on_edge[j]) and
                 E[j] >= elev_med + theta_warp]
    stray_keys = [keys[j] for j in range(len(keys))
                  if off_stray[j] and E[j] >= max(theta_det, 0.5 * LH)]
    cell_area = XBIN_MM * y_step_mm

    def clus(keys_):
        cl = _flood_clusters(keys_)
        out = []
        for comp in cl:
            ee = [cells[k]["elev"] for k in comp]
            out.append({
                "area_mm2": len(comp) * cell_area,
                "cells": len(comp),
                "x_mm": (min(cells[k]["X"] for k in comp),
                         max(cells[k]["X"] for k in comp)),
                "y_mm": (min(cells[k]["Y"] for k in comp),
                         max(cells[k]["Y"] for k in comp)),
                "elev_med_um": float(np.median(ee)),
                "elev_max_um": float(np.max(ee)),
            })
        return sorted(out, key=lambda c: -c["area_mm2"])

    miss_cl = clus(missing_keys)
    warp_cl = clus(warp_keys)
    stray_cl = clus(stray_keys)

    iqr = float(np.percentile(E[interior], 75) - np.percentile(E[interior], 25))
    m = R["metrics"]
    m.update({
        "sigma_cell_um": sigma,
        "elev_med_um": elev_med,
        "elev_iqr_um": iqr,
        "theta_det_um": theta_det,
        "theta_warp_um": theta_warp,
        "n_cells": len(keys),
        "n_interior": n_int,
        "coverage": cov,
        "missing_frac": float(len(missing_keys)) / n_int,
        "missing_clusters": miss_cl[:5],
        "warp_clusters": warp_cl[:5],
        "stray_clusters": stray_cl[:5],
        "below_plate_frac": float(below.mean()),
        "unscanned_frac": unscanned_frac,
        "cell_area_mm2": cell_area,
        "dz_thermal_um": {bo["i"]: bo["dz_g"] for bo in band_out},
        "elev_over_LH": elev_med / LH if LH else None,
    })

    # z-offset suggerito: elev_med - LH = errore apparente di gap ugello.
    # ATTENZIONE bias di riflettivita' (bianco su nero: il piatto scuro legge
    # ~basso) -> azionabile solo oltre il margine; il margine di default e'
    # il bias osservato su uno strato BUONO (qui ~ +0.25LH), sovrascrivibile
    # con la baseline del profilo filamento.
    bias_margin = elev_bias_margin_um if elev_bias_margin_um is not None \
        else 0.35 * LH
    z_err = elev_med - LH
    m["z_offset_hint_mm"] = round(-z_err / 1000.0, 3) \
        if abs(z_err) > bias_margin else 0.0

    # ---- verdetto ----
    reasons = []
    A_MIN = 3 * cell_area          # ~4.2 mm2: sotto, non distinguibile da
                                   # rumore locale/dropout, sopra, difetto vero
    if elev_med < 0.35 * LH:
        reasons.append("strato troppo SOTTILE: %.0f um vs layer %.0f um "
                       "(z-offset troppo basso / flusso insufficiente)"
                       % (elev_med, LH))
    if elev_med > 2.5 * LH:
        reasons.append("strato troppo ALTO: %.0f um vs layer %.0f um "
                       "(z-offset troppo alto: adesione a rischio)"
                       % (elev_med, LH))
    big_miss = [c for c in miss_cl if c["area_mm2"] >= A_MIN]
    if big_miss or m["missing_frac"] > 0.10:
        for c in big_miss[:3]:
            reasons.append("materiale MANCANTE: %.0f mm2 in X%.0f-%.0f "
                           "Y%.0f-%.0f" % (c["area_mm2"], c["x_mm"][0],
                                           c["x_mm"][1], c["y_mm"][0],
                                           c["y_mm"][1]))
        if m["missing_frac"] > 0.10 and not big_miss:
            reasons.append("materiale mancante diffuso: %.0f%% delle celle"
                           % (100 * m["missing_frac"]))
    # warp che conta = rischio collisione col nozzle al layer 2 (che passa a
    # quota elev_med + LH): curl esteso oltre +1 LH, oppure picco oltre +2 LH
    # anche se piccolo. Sotto: bump di cucitura (misurati +0.2mm su 2-3 celle
    # anche su uno strato PERFETTO), non warping.
    big_warp = [c for c in warp_cl
                if (c["area_mm2"] >= 4 * cell_area
                    and c["elev_max_um"] >= elev_med + max(theta_warp, 1.0 * LH))
                or (c["area_mm2"] >= 2 * cell_area
                    and c["elev_max_um"] >= elev_med + 2.0 * LH)]
    for c in big_warp[:3]:
        reasons.append("SOLLEVAMENTO: %.0f mm2 a X%.0f-%.0f Y%.0f-%.0f, "
                       "picco +%.2f mm sopra lo strato"
                       % (c["area_mm2"], c["x_mm"][0], c["x_mm"][1],
                          c["y_mm"][0], c["y_mm"][1],
                          (c["elev_max_um"] - elev_med) / 1000.0))
    big_stray = [c for c in stray_cl if c["area_mm2"] >= A_MIN]
    for c in big_stray[:2]:
        reasons.append("materiale INATTESO fuori maschera: %.0f mm2 a "
                       "X%.0f-%.0f Y%.0f-%.0f" % (c["area_mm2"],
                       c["x_mm"][0], c["x_mm"][1], c["y_mm"][0], c["y_mm"][1]))

    if cov < 0.80 and not reasons:
        reasons.append("copertura bassa: %.0f%% delle celle attese ha "
                       "materiale" % (100 * cov))

    warn = R["warnings"]
    if m["below_plate_frac"] > 0.05:
        warn.append("%.0f%% celle sotto il piatto: riflessi residui, "
                    "qualita' dati da verificare" % (100 * m["below_plate_frac"]))
    if unscanned_frac > 0.15:
        warn.append("%.0f%% della maschera attesa NON scandito (finestra "
                    "Y/X corta: aggiungere margine)" % (100 * unscanned_frac))
    if iqr > max(6 * sigma, 0.8 * LH):
        warn.append("uniformita' scarsa: IQR %.0f um" % iqr)

    R["cells"] = cells
    if reasons:
        R["verdict"] = "BAD"
        R["reasons"] = reasons
    else:
        R["verdict"] = "GOOD"
    return R


def format_report(R):
    L = ["VERDETTO: %s" % R["verdict"]]
    for r in R["reasons"]:
        L.append("  ! %s" % r)
    for w in R["warnings"]:
        L.append("  ~ %s" % w)
    m = R.get("metrics", {})
    if m:
        L.append("  elev mediana %.0f um (%.2fx layer)  IQR %.0f  sigma %.0f"
                 % (m.get("elev_med_um", 0), m.get("elev_over_LH", 0) or 0,
                    m.get("elev_iqr_um", 0), m.get("sigma_cell_um", 0)))
        L.append("  copertura %.1f%%  mancante %.1f%%  celle %d (interne %d)"
                 % (100 * m.get("coverage", 0), 100 * m.get("missing_frac", 0),
                    m.get("n_cells", 0), m.get("n_interior", 0)))
        if "reg_corr_pooled" in m:
            L.append("  registrazione pooled corr %.3f" % m["reg_corr_pooled"])
        d = m.get("distortion")
        if d:
            L.append("  distorsione lama: err(xi)=%.4f xi^2 %+.3f xi %+.2f mm "
                     "su [%.0f,%.0f], rms %.2f, %d fronti"
                     % (d["coef"][0], d["coef"][1], d["coef"][2],
                        d["lo"], d["hi"], d["rms"], m.get("n_edges", 0)))
        if "overlap_diff_um" in m:
            L.append("  overlap bande: %.0f um su %d celle"
                     % (m["overlap_diff_um"], m["overlap_cells"]))
        L.append("  dz termico per banda: %s" %
                 {k: "%.0f" % v for k, v in m.get("dz_thermal_um", {}).items()})
        L.append("  z-offset suggerito: %+.3f mm" % m.get("z_offset_hint_mm", 0))
        L.append("  non scandito: %.0f%%" % (100 * m.get("unscanned_frac", 0)))
    for b in R.get("bands", []):
        L.append("  banda %d: dx %+.2f dy %+.2f corr %.3f dz %.0f um "
                 "sigma %.0f n_far %d (C_eff %.2f)"
                 % (b["i"], b["dx"], b["dy"], b["corr"], b["dz_g"],
                    b["sigma"], b["n_far"], b["C_eff"]))
    return "\n".join(L)
