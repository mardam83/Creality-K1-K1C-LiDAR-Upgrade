#!/usr/bin/env python3
# lidar_firstlayer.py - Analisi PRIMO STRATO col LiDAR, STANDALONE (Fase 3).
# Gira SULLA STAMPANTE. Replica il flusso di default del display (catturato via
# strace il 2026-07-15): scan del piatto nudo sulla finestra dell'oggetto ->
# stampa primo layer (SDCARD_PRINT_FILE FIRST_FLOOR_PRINT=True, auto-pausa) ->
# ri-scan della stessa traiettoria -> get_first_floor_detect_new -> result 0/1.
#
# Uso:
#   python3 lidar_firstlayer.py --gcode NOMEFILE.gcode [--factory-level]
#                               [--frames N] [--resume|--cancel] [--dry-run]
#
# Dettagli replicati dall'originale:
#  - stessa traiettoria per tabella e primo strato (condizione del confronto);
#  - finestra scan = bbox estrusioni layer1 + offset laser (X centro +41.0, Y +23.2);
#  - Z3; l'originale fa un frame ogni ~2.68mm, noi ogni 1.4mm (Y_PITCH_MM);
#  - gcode_reduction generato dal gcode (estremi dei segmenti di estrusione, um,
#    sentinella INT32_MAX) e passato come gcode_path: OBBLIGATORIO (code 102 se manca);
#  - chiavi del comando: scan1_path/scan2_path/gcode_path;
#  - PRINT_PREPARED prima di SDCARD_PRINT_FILE (salta il rilivellamento nello START_PRINT).
# In PIU' rispetto all'originale: appena la stampa e' in pausa azzeriamo il flag
# FIRST_FLOOR_PRINT (endpoint pause_resume/set_print_first_layer), altrimenti la
# stampa si riferma a OGNI cambio layer e il --resume non completa mai il pezzo.
# Campionamento DETERMINISTICO: trigger temporali calcolati dal profilo di moto
# reale (latenza + accelerazione), cosi' il frame i cade davvero a y0+i*passo.
# Laser: di default seriale DIRETTA (k1lidar.DriverLaser, --backend middleware per
# il metodo storico); verdetto deciso da C2 (fl_analyze.py), Creality informativo.

import json, time, argparse, os, re, sys, math

from .session import (PC, GCODES, ROOT, add_common_args, resolve_temps,
                      filament_key, kl, kl_query, kl_rpc, wait_state, creality_cooldown,
                      park, heaters_off, laser_up, self_cal, apply_exposure, acquire_lock,
                      release_lock, LOCK, archive_run, load_calib, save_calib, load_frames,
                      factory_level, echo, say_cmd, linear_sweep)
from .i18n import T
from . import gcode_meta as GM

# offset finestra laser rispetto alle coordinate oggetto (ricavati dalla cattura:
# scan X152.07 per bbox X101.07-121.03 -> centro+41.0; scan Y86.37-180.15 per
# bbox Y63.17-156.95 -> +23.2 su entrambi gli estremi)
LASER_DX_CENTER = 41.0      # costante di POSIZIONAMENTO usata da Creality.
                            # NON e' il centro ottico e NON va usata per mappare
                            # i punti: la relazione x_interna->X macchina e' una
                            # DISTORSIONE (~3mm di escursione, identica tra le
                            # bande), che fl_analyze calibra in-run dai fronti.
LASER_DX_PLAN = 36.0        # centro UTILE della lama (X_nozzle - 36): e' qui che
                            # sta il plateau di densita'. Pianificando con 41 la
                            # banda lavora spostata di ~5mm e spreca il lato denso.
LASER_DY = 23.2             # offset Y di posizionamento (come l'originale).
                            # L'offset ottico EFFETTIVO e' ~25.0 (dy di
                            # registrazione -1.75mm misurato su entrambe le
                            # bande): lo assorbe SCAN_MARGIN_Y, cosi' restiamo
                            # compatibili con la geometria di fabbrica.
SCAN_MARGIN_Y = 3.0         # margine per lato sulla finestra Y. Serve a (a)
                            # coprire i ~1.8mm di scarto DY_vero-DY_nominale
                            # (senza, ~10% della maschera non viene scandito) e
                            # (b) garantire righe di piatto NUDO agli estremi,
                            # che servono a stimare il dz termico per frame.
Y_PITCH_MM = 1.4             # passo Y obiettivo tra due frame. L'originale usa
                             # ~2.68mm (35 frame su 93.78mm); qui piu' fitto
                             # perche' la cella di C2 e' lunga un passo: a 1.4mm
                             # il difetto minimo rilevabile e' ~4mm2. Prima il
                             # numero di frame era fissato a min 15 e col margine
                             # Y il passo saliva a 1.84mm (~7mm2). La velocita' di
                             # sweep si adatta da sola al tempo di cattura.
MIN_FRAMES = 15
SCAN_Z = 3.0
Y_LIMIT = 231.0              # fine corsa Y utile (stesso limite del check finestra)
# Copertura X: la lama del laser copre solo ~31mm utili (misurato), quindi un
# oggetto piu' largo va scansionato in piu' PASSATE X affiancate. Non si possono
# fondere in un unico scan: l'analizzatore usa le x ASSOLUTE del sensore e una
# deriva >5mm da' anomalia spuria (verificato sul detector). Quindi N passate
# INDIPENDENTI, ognuna col gcode ritagliato alla sua banda, e verdetto = OR.
LAMA_UTILE = 26.0           # larghezza utile REALE (mm). Il plateau di densita'
                            # misurato e' [-12,+10] (~22mm) con spalle decenti
                            # fino a +-15 e code inservibili oltre +-17. I 31mm
                            # precedenti includevano le code sotto-campionate.
BAND_OVERLAP = 10.0         # sovrapposizione NOMINALE (obiettivo): assorbe spalle
                            # deboli (+-12..17), registrazione (+-0.5) e
                            # distorsione residua.
BAND_OVERLAP_MIN = 8.0      # sovrapposizione MINIMA ACCETTABILE, ed e' questa a
                            # decidere il NUMERO di bande. Misurato: 8-10mm
                            # restano corretti. Usare il valore nominale per il
                            # conteggio e' troppo severo e costa una passata in
                            # piu': su un oggetto da 34.8mm chiedere >=10mm da
                            # n=3 (sovrapp. reale 14.4mm), mentre n=2 basta e
                            # avanza (sovrapp. reale 8.6mm, copertura 43.4mm).

def resolve_gcode(name):
    """Path del gcode, tollerante sui nomi con SPAZI.

    Klipper spezza i parametri delle macro sugli spazi, quindi
    `LIDAR_FIRSTLAYER_CHECK FILE=faba v1.gcode` non puo' arrivare intero: chi
    chiama codifica gli spazi come '+' o '%20'. Qui proviamo, in ordine: il nome
    cosi' com'e', le due decodifiche, e infine un match univoco case-insensitive
    sui file presenti (utile quando il nome e' parziale).
    """
    cands = [name]
    if "+" in name: cands.append(name.replace("+", " "))
    if "%" in name:
        try:
            from urllib.parse import unquote
            cands.append(unquote(name))
        except Exception: pass
    for c in cands:
        p = c if os.path.isabs(c) else os.path.join(GCODES, c)
        if os.path.exists(p): return p
    # ultimo tentativo: match univoco per prefisso/sottostringa
    try: files = os.listdir(GCODES)
    except OSError: files = []
    for c in cands:
        base = os.path.basename(c).lower()
        hits = [f for f in files if f.lower() == base] \
            or [f for f in files if f.lower().startswith(base)] \
            or [f for f in files if base in f.lower()]
        if len(hits) == 1:
            print("    (nome risolto: %r -> %r)" % (name, hits[0]))
            return os.path.join(GCODES, hits[0])
        if len(hits) > 1:
            raise SystemExit("nome ambiguo %r, corrispondenze: %s" % (name, hits[:5]))
    raise SystemExit("gcode non trovato: %r (cercato in %s)" % (name, GCODES))

def clear_first_layer_flag():
    """Azzera il flag FIRST_FLOOR_PRINT di virtual_sdcard.

    Senza questo la stampa si RIFERMA a OGNI layer: work_handler valuta
    `if self.print_first_layer and self.count_G1 >= 20:` a ogni riga di
    cambio-layer e chiama FIRST_FLOOR_PAUSE; `print_first_layer` non viene
    mai azzerato a stampa in corso (solo in cmd_SDCARD_PRINT_FILE e a fine
    stampa) e `count_G1` si riarma da solo (azzerato a ogni giro di
    work_handler, risale a 20 in poche righe).

    L'endpoint webhooks `pause_resume/set_print_first_layer` (pause_resume.py)
    azzera SIA `print_first_layer` SIA `first_layer_stop`: e' il meccanismo
    che usa il display, quindi niente patch a klipper ne' override di macro.
    """
    try:
        kl_rpc("pause_resume/set_print_first_layer")
    except Exception as e:
        print("    ATTENZIONE: non ho potuto azzerare il flag primo strato (%s):" % e)
        print("    la stampa si rifermera' a ogni cambio layer.")
        return False
    # Verifica: `print_first_layer` non e' esposto in get_status, ma l'endpoint
    # azzera anche `first_layer_stop` (che la pausa ha appena messo a True),
    # quindi serve da proxy. Se resta True l'endpoint non ha fatto effetto.
    st = kl_query({"virtual_sdcard": ["first_layer_stop"]}).get("virtual_sdcard", {})
    if st.get("first_layer_stop") is True:
        print("    ATTENZIONE: flag primo strato ancora attivo dopo l'azzeramento:")
        print("    la stampa si rifermera' a ogni cambio layer.")
        return False
    return True

G1RE = re.compile(r"^G[01]\s+([^;]*)")
def first_layer_segments(path):
    """Segmenti di ESTRUSIONE del primo layer: lista di (x0,y0,x1,y1) in mm
    (regola del gcode_reduction originale: solo G1 con E>0, niente travel)."""
    segs = []
    x = y = z = None; layer_z = None
    with open(path) as f:
        for line in f:
            m = G1RE.match(line)
            if not m: continue
            vals = {}
            for p in m.group(1).split():
                try: vals[p[0]] = float(p[1:])
                except (ValueError, IndexError): pass
            px, py = x, y
            if "X" in vals: x = vals["X"]
            if "Y" in vals: y = vals["Y"]
            if "Z" in vals: z = vals["Z"]
            if "E" in vals and vals["E"] > 0 and None not in (px, py, x, y):
                if layer_z is None and z is not None: layer_z = z
                if layer_z is not None and z is not None and z > layer_z + 0.15:
                    break  # inizio secondo layer
                segs.append((px, py, x, y))
    if not segs: raise SystemExit("nessuna estrusione trovata nel primo layer di %s" % path)
    return segs, (layer_z or 0.2)

def first_layer_bbox(segs):
    xs = [v for s in segs for v in (s[0], s[2])]
    ys = [v for s in segs for v in (s[1], s[3])]
    return min(xs), min(ys), max(xs), max(ys)

def write_gcode_reduction(segs, layer_z, out_path, xband=None):
    """Genera il gcode_reduction come master-server: per ogni segmento di
    estrusione i due estremi [x*1000,y*1000,z*1000] (i vertici interni di un
    percorso continuo risultano doppi = edge-list), sentinella INT32_MAX in coda.

    xband=(bx0,bx1): tiene solo i segmenti la cui X ricade nella banda (per il
    multi-passata: ogni passata copre una striscia X e va confrontata col gcode
    di QUELLA striscia). Un segmento e' incluso se almeno un estremo e' in banda."""
    zi = int(round(layer_z * 1000))
    pts = []
    for x0, y0, x1, y1 in segs:
        if xband is not None:
            bx0, bx1 = xband
            if not (bx0 <= x0 <= bx1 or bx0 <= x1 <= bx1):
                continue
        pts.append([int(round(x0 * 1000)), int(round(y0 * 1000)), zi])
        pts.append([int(round(x1 * 1000)), int(round(y1 * 1000)), zi])
    pts.append([2147483647, 0, 0])
    with open(out_path, "w") as f: json.dump(pts, f)
    return len(pts)

def band_plan(x0, x1):
    """Divide il bbox X [x0,x1] in bande da coprire con una passata ciascuna.
    Ritorna lista di dict {x_nozzle, center, bx0, bx1}. Per oggetti <= lama
    ritorna 1 banda = comportamento della passata singola (invariato)."""
    W = x1 - x0
    # Il numero di bande lo decide la sovrapposizione MINIMA accettabile, non
    # quella nominale: le bande sono equispaziate su W, quindi la sovrapposizione
    # reale e' LAMA_UTILE - W/n e cresce al crescere di n. Chiedere >=10mm invece
    # di >=8mm fa scattare una passata in piu' senza guadagno di copertura.
    step = LAMA_UTILE - BAND_OVERLAP_MIN
    n = 1 if W <= LAMA_UTILE else int(math.ceil(W / step))
    half = LAMA_UTILE / 2.0 + BAND_OVERLAP / 2.0
    bands = []
    for k in range(n):
        c = x0 + W * (2 * k + 1) / (2.0 * n)          # centro banda in X macchina
        bands.append({
            # posizionamento sul centro UTILE della lama, non su LASER_DX_CENTER
            "x_nozzle": c + LASER_DX_PLAN,
            "center": c,
            "bx0": max(x0, c - half),
            "bx1": min(x1, c + half),
        })
    return bands

SAT_UM = 32700           # |z|>=32700 = INT16_MAX/MIN = "nessuna lettura" del laser
UNDER_UM = 300           # punto piu' basso del piano di oltre 0.3mm = impossibile

def _median(v):
    v = sorted(v)
    return v[len(v) // 2] if v else None

def frame_reference(table_path):
    """Mediana Z per-frame dalla tabella (piano nudo) = quota del piatto in ogni
    posizione della sweep. E' il riferimento contro cui si giudica lo scan dello
    strato: serve al filtro anti-riflesso e all'analisi dei delta."""
    try:
        d = json.load(open(table_path))
    except Exception:
        return {}
    ref = {}
    for k, fr in d.items():
        i = int("".join(c for c in k if c.isdigit()) or -1)
        ref[i] = _median([p[2] for p in fr])
    return ref

def filter_reflections(frames, label, ref_med=None):
    """Toglie SOLO i punti fisicamente impossibili per un primo strato.

    Scoperta chiave (dimostrata facendo girare il detector originale su dati reali
    e su ~40 varianti): l'analizzatore Creality NON controlla presenza ne' altezza
    del materiale. Reagisce SOLO a punti FUORI BANDA dentro un frame -> basta un
    frame contaminato per result=1. Sulla plastica appena stampata il laser
    produce due tipi di artefatto (mai sul piatto nudo):
      - 'fantasma' a z>0 (~58mm sopra il piatto): riflesso;
      - punti 'sotto il piatto' (~1.5-2.3mm sotto il piano): riflesso/rifrazione;
    piu' i saturati a +-32767 ('nessuna lettura').
    Con questi in un frame la mediana finiva nel gruppo sbagliato -> verdetto
    falso. VALIDATO sul detector VERO del middleware: filtrando, Corpo1 e
    ref_20260716 passano da result=1 a result=0, scartando 22%/5% dei punti.

    UNA sola soglia fisica cattura entrambi gli artefatti senza accecare il
    detector: un punto piu' BASSO del piano (piu' positivo di tab+0.3mm) non puo'
    essere primo strato (ghost e sotto-piatto lo sono entrambi). Il MATERIALE
    (anche sollevato: piu' ALTO del piano, z piu' negativo) viene sempre
    CONSERVATO -> uno strato davvero difettoso resta bocciato.

    ref_med: mediane per-frame della tabella (dict idx->z). Senza (per lo scan
    della tabella stessa) si usa la mediana globale dei frame come piano.
    """
    if ref_med is None:
        g = _median([p[2] for fr in frames for p in fr])
        ref_med = {i: g for i in range(len(frames))}
    out = []; dropped = 0
    for i, fr in enumerate(frames):
        rf = ref_med.get(i)
        keep = [p for p in fr
                if abs(p[2]) < SAT_UM and (rf is None or p[2] <= rf + UNDER_UM)]
        dropped += len(fr) - len(keep)
        out.append(keep)
    if dropped:
        tot = sum(len(fr) for fr in frames)
        print("  [%s] filtro riflessi: scartati %d/%d punti (%.1f%%) fuori dal piano"
              % (label, dropped, tot, 100.0 * dropped / tot))
        vuoti = sum(1 for fr in out if not fr)
        if vuoti:
            print("  [%s] ATTENZIONE: %d frame senza punti dopo il filtro" % (label, vuoti))
    return out, dropped

def save_scan_speed(table_path, f):
    """Memorizza la velocita' usata per la tabella, accanto al .temp.
    Serve a --scan-only, che riusa una tabella acquisita in un run precedente e
    deve campionare il primo strato nelle stesse posizioni."""
    try:
        with open(table_path + ".speed", "w") as fh: fh.write(str(int(f)))
    except Exception: pass

def load_scan_speed(table_path):
    try:
        with open(table_path + ".speed") as fh: return int(fh.read().strip())
    except Exception: return None

SCAN_STATS = {}     # label -> punti/scartati/vuoti, per il manifest e le calibrazioni

def scan_window(L, save_path, label, key_prefix, x, y0, y1, n_frames,
                mode="cont", speed=1200, force_speed=None, ref_med=None):
    """Scan della finestra col backend laser L; ritorna la velocita' (F) usata.

    - "cont" (default, come l'originale): UNA passata lineare continua; la Y di
      ogni frame viene dalla traiettoria vera di klipper (session.linear_sweep),
      cosi' il frame i cade davvero a y0+i*step.
    - "step": n_frames passi Y a testa ferma (solo diagnostica).

    `force_speed` (F): velocita' della tabella riusata per lo strato. Non e' piu'
    indispensabile (i frame sono scelti per posizione), ma tiene identiche le
    condizioni dei due scan.
    """
    step = (y1 - y0) / (n_frames - 1)
    print("  [%s] X%.2f Y%.2f->%.2f, %d frame (%.2fmm/frame, %s)"
          % (label, x, y0, y1, n_frames, step, mode))
    kl("G90\nG0 Z%.1f F600\nG0 X%.2f Y%.2f F12000\nM400" % (SCAN_Z, x, y0))
    L.open_scan()

    def grab(retry=3):
        for _ in range(retry):
            pc = L.grab()
            if pc: return pc
            time.sleep(0.03)
        return []

    frames = []; empty = 0
    v = (force_speed or speed) / 60.0
    if mode == "step":
        for i in range(n_frames):
            kl("G0 X%.2f Y%.3f F9000\nM400" % (x, y0 + i * step))
            pc = grab()
            if not pc: empty += 1
            frames.append(pc)
    else:
        # Passata lineare continua sincronizzata con klipper (session.linear_sweep):
        # il vecchio metodo a trigger temporali sbagliava la posizione appena la
        # passata superava ~2 s di moto (klipper trattiene la risposta del G0).
        # Tabella e strato campionano comunque le STESSE Y (frame scelto per
        # posizione), anche se la velocita' effettiva differisse.
        frames, info = linear_sweep(L, x, y0, y1, n_frames, force_speed or speed, label,
                                    y_max=Y_LIMIT)
        empty = info["empty"]
        v = info["speed_F"] / 60.0
    L.close_scan()
    raw_pts = sum(len(fr) for fr in frames)
    frames, ghosts = filter_reflections(frames, label, ref_med)
    data = {"%s%d" % (key_prefix, i): fr for i, fr in enumerate(frames)}
    with open(save_path, "w") as f: json.dump(data, f)
    npts = sum(len(fr) for fr in frames)
    print("  [%s] %d frame (%d vuoti), %d punti -> %s"
          % (label, len(frames), empty, npts, os.path.basename(save_path)))
    SCAN_STATS[label] = {"points": npts, "raw_points": raw_pts, "dropped": ghosts,
                         "empty_frames": empty, "frames": len(frames),
                         "speed_F": int(round(v * 60))}

    # GUARDIA CRITICA: uno scan senza punti (laser spento/non sbloccato, open_flow
    # rifiutato) NON deve mai arrivare all'analizzatore: get_first_floor_detect_new
    # su dati vuoti risponde code 0 / result 0 = "primo strato OK" -> falso positivo,
    # la stampa proseguirebbe senza alcun controllo reale. Meglio fallire rumorosamente.
    if npts == 0:
        raise RuntimeError(
            "scan '%s' VUOTO (%d/%d frame senza punti): laser non attivo. "
            "Verdetto non calcolabile - NON mi fido di un result su dati vuoti."
            % (label, empty, len(frames)))
    if empty > len(frames) // 2:
        raise RuntimeError(
            "scan '%s' inaffidabile: %d frame vuoti su %d" % (label, empty, len(frames)))
    if empty:
        print("  [%s] ATTENZIONE: %d frame vuoti (scan comunque utilizzabile)" % (label, empty))
    return int(round(v * 60))     # velocita' usata: il secondo scan deve riusarla

def analyze_c2(bands, tab_paths, scan_paths, segs, layer_h, sy0, sy1, n, w_line):
    """Analisi QUANTITATIVA nostra (C2): mappa di altezza vs piano, difetti
    localizzati in mm, verdetto GOOD/BAD/UNCERTAIN.

    Sostituisce il verdetto binario di Creality, che su bande parziali sbaglia
    sistematicamente (misurato: 2 falsi positivi su uno strato perfetto) perche'
    reagisce a QUALUNQUE punto fuori banda in un frame senza mai controllare
    presenza o altezza del materiale.

    `segs` sono i segmenti COMPLETI del layer 1 (non ritagliati alla banda):
    cosi' la maschera include la skirt e non serve `stray_ring_mm`."""
    import numpy as np
    if ROOT not in sys.path:
        sys.path.insert(0, ROOT)
    import fl_analyze as fa
    bl = []
    for k, b in enumerate(bands):
        bl.append({"table": load_frames(tab_paths[k]),
                   "layer": load_frames(scan_paths[k]),
                   "x_nozzle": b["x_nozzle"]})
    segs_mm = np.array([[s[0], s[1], s[2], s[3]] for s in segs], dtype=np.float64)
    step = (sy1 - sy0) / float(n - 1) if n > 1 else 0.0
    # ATTENZIONE: analyze() vuole la Y del primo frame in coordinate OGGETTO
    # (dove sta il materiale), non la Y del NOZZLE. Il laser guarda LASER_DY
    # piu' indietro, quindi col nozzle a sy0 si osserva il piano a sy0-LASER_DY.
    # Passare sy0 grezzo sfalsa tutto di 23.2mm e la registrazione fallisce
    # (corr ~0 -> UNCERTAIN su ogni run). Il residuo lo assorbe la registrazione.
    y0_obs = sy0 - LASER_DY
    return fa.analyze(bl, segs_mm, layer_h, y0_obs, step, n, w_line=w_line), fa

def build_parser():
    ap = argparse.ArgumentParser(description="Analisi primo strato col LiDAR (standalone)")
    ap.add_argument("--gcode", required=True, help="nome file in %s (o path assoluto)" % GCODES)
    ap.add_argument("--factory-level", action="store_true",
                    help="prep completa: CX_ROUGH_G28+NOZZLE_CLEAR+ACCURATE_G28+CHECK_BED_MESH")
    add_common_args(ap)
    ap.add_argument("--frames", type=int, default=0,
                    help="frame per scan (0 = auto, passo %.1fmm)" % Y_PITCH_MM)
    ap.add_argument("--scan-mode", default="cont", choices=["cont", "step"],
                    help="cont = sweep continua con trigger di posizione (come l'originale); step = passi a testa ferma")
    ap.add_argument("--scan-speed", type=int, default=1200, help="feedrate della sweep in mode cont (originale: F1200)")
    ap.add_argument("--no-selfcal", action="store_true")
    ap.add_argument("--scan-only", action="store_true",
                    help="NON stampa e non livella: scansiona il primo strato GIA' sul piatto e "
                         "da' il verdetto, riusando la tabella del piatto nudo di un run precedente")
    ap.add_argument("--resume", action="store_true", help="riprendi la stampa se result=0 (ok)")
    ap.add_argument("--cancel", action="store_true", help="annulla la stampa a fine analisi (come fa il display su result=1)")
    ap.add_argument("--dry-run", action="store_true",
                    help="niente stampa: doppio scan del piatto nudo + detect (test idraulica)")
    ap.add_argument("--no-c2", action="store_true",
                    help="NON eseguire l'analisi quantitativa C2 (torna a decidere col solo "
                         "detector Creality; sconsigliato: sbaglia sulle bande parziali)")
    ap.add_argument("--line-width", type=float, default=None,
                    help="larghezza linea del primo strato in mm, usata da C2 per la maschera "
                         "'dove DEVE esserci materiale' (default: letta dal gcode, altrimenti 0.5)")
    ap.add_argument("--on-bad", default="pause", choices=["pause", "cancel"],
                    help="con --resume, cosa fare se il primo strato e' DIFETTOSO (default pausa)")
    ap.add_argument("--remember", action="store_true",
                    help="salva --exposure come esposizione del filamento (--filament)")
    return ap


def main():
    a = build_parser().parse_args()
    return run(a)


def run(a, L=None, prepared=False):
    """Controllo del primo strato. Con `L` (laser gia' acceso) e `prepared=True` gira
    dentro un job piu' grande (avvio stampa dopo la taratura PA): niente lock, ne'
    livellamento, ne' self-cal, ne' spegnimento del laser, a carico del chiamante.
    Ritorna {"verdict": GOOD|BAD|UNCERTAIN|n/d, "creality": ..., "c2": ...}."""
    own = L is None
    hot, bed, tsrc = resolve_temps(a)
    fil = filament_key(a, hot)
    calib = load_calib(fil)
    print("== Analisi primo strato col LiDAR ==  %s: ugello %d C / piatto %d C | filamento %s | laser: %s"
          % (tsrc, hot, bed, fil, a.backend))

    gpath = resolve_gcode(a.gcode)
    echo(T("fl_start", file=os.path.basename(gpath)))
    if a.line_width is None:
        try:
            a.line_width = GM.read(gpath).get("first_layer_line_width") or 0.5
        except Exception:
            a.line_width = 0.5
        print("  larghezza linea primo strato: %.2f mm (dal gcode)" % a.line_width)
    segs, lz = first_layer_segments(gpath)
    x0, y0, x1, y1 = first_layer_bbox(segs)
    nmoves = len(segs)
    sy0, sy1 = y0 + LASER_DY - SCAN_MARGIN_Y, y1 + LASER_DY + SCAN_MARGIN_Y
    n = a.frames or max(MIN_FRAMES, int(math.ceil((sy1 - sy0) / Y_PITCH_MM)) + 1)

    # Copertura X: se l'oggetto e' piu' largo della lama, N passate affiancate.
    bands = band_plan(x0, x1)
    print("  oggetto: bbox X%.2f-%.2f Y%.2f-%.2f (layer Z%.2f, %d estrusioni)" % (x0, x1, y0, y1, lz or -1, nmoves))
    print("  finestra Y: %.2f->%.2f (%d frame) | larghezza X %.1fmm -> %d passata/e"
          % (sy0, sy1, n, x1 - x0, len(bands)))
    for k, b in enumerate(bands):
        print("    banda %d: nozzle X%.2f, copre X%.1f-%.1f" % (k, b["x_nozzle"], b["bx0"], b["bx1"]))
        if b["x_nozzle"] > 231 or sy1 > 231:
            raise SystemExit("finestra scan fuori corsa (X%.1f Y%.1f)" % (b["x_nozzle"], sy1))

    # path per-banda (indicizzati): tabella, scan strato, gcode ritagliato
    tab_paths  = [PC + "/scan_table_%d.temp" % k for k in range(len(bands))]
    scan_paths = [PC + "/scan_layer_%d.temp" % k for k in range(len(bands))]
    gred_paths = [PC + "/gcode_red_%d.temp" % k for k in range(len(bands))]
    for k, b in enumerate(bands):
        write_gcode_reduction(segs, lz, gred_paths[k], xband=(b["bx0"], b["bx1"]))

    if a.scan_only:
        missing = [p for p in tab_paths if not os.path.exists(p)]
        if missing:
            raise SystemExit(
                "--scan-only richiede le tabelle del piatto NUDO di un run precedente,\n"
                "  mancano: %s. Vanno acquisite PRIMA di stampare l'oggetto." % ", ".join(missing))


    # Un solo script alla volta sul laser (prima il primo strato non aveva lock:
    # lanciato insieme a lidar_pa.py si sarebbero rubati la seriale).
    if own and not acquire_lock():
        raise SystemExit("ERRORE: un altro script LiDAR sta usando il laser (%s)." % LOCK)

    res = {}; c2 = None; printing = False; vexp = None; fans_saved = None
    # Tutto dentro il try: [1] puo' accendere i riscaldatori, quindi un errore
    # PRIMA della stampa deve spegnerli (lo faceva gia' lidar_pa.py, qui mancava).
    try:
        print("\n[1] preparazione")
        if prepared:
            # Stesso job della taratura PA: livellamento gia' fatto. Rifarlo vorrebbe
            # dire tastare gli angoli col pattern PA gia' stampato sul bordo.
            print("    livellamento gia' fatto in questo job: lo salto")
        elif a.scan_only:
            prep_scan_only()
        elif a.factory_level:
            factory_level(hot, bed)
        else:
            homed = kl_query({"toolhead": ["homed_axes"]}).get("toolhead", {}).get("homed_axes", "")
            if "z" not in homed.lower():
                print("    non homata: G28")
                kl("G28")
        if not a.dry_run and not a.scan_only:
            kl("M140 S%d" % bed)          # scalda il piatto durante gli scan (come l'originale)

        if own:
            print("\n[2] alimentazione + sblocco laser")
            L = laser_up(a.backend)
            if L is None:
                raise RuntimeError("laser non disponibile (ne' driver ne' middleware)")
            vexp = apply_exposure(L, a, calib)
            if not a.no_selfcal:
                self_cal(L)

        # velocita' per-banda: riusata tra tabella e strato della STESSA banda,
        # cosi' i due scan campionano nelle stesse Y (l'auto-riduzione sceglieva
        # valori diversi -> sfasamento; misurato F873 vs F694).
        v_used = [None] * len(bands)

        # [3] TABELLE (una per banda), tutte a piatto nudo
        if a.scan_only:
            print("\n[3] tabelle: riuso quelle esistenti (%d banda/e)" % len(bands))
            for k in range(len(bands)):
                v_used[k] = load_scan_speed(tab_paths[k])
        else:
            print("\n[3] scan TABELLE piatto nudo (%d passata/e)" % len(bands))
            for k, b in enumerate(bands):
                lbl = "tabella" if len(bands) == 1 else "tabella b%d" % k
                v_used[k] = scan_window(L, tab_paths[k], lbl, "table", b["x_nozzle"],
                                        sy0, sy1, n, a.scan_mode, a.scan_speed)
                save_scan_speed(tab_paths[k], v_used[k])

        # riferimento del piano per il filtro anti-riflesso, per banda
        refs = [frame_reference(tab_paths[k]) for k in range(len(bands))]

        def scan_all_layers(tag):
            for k, b in enumerate(bands):
                lbl = "primo strato" if len(bands) == 1 else "%s b%d" % (tag, k)
                scan_window(L, scan_paths[k], lbl, "scan", b["x_nozzle"], sy0, sy1, n,
                            a.scan_mode, a.scan_speed, force_speed=v_used[k], ref_med=refs[k])

        if a.scan_only:
            print("\n[4-5] scan del primo strato GIA' presente (nessuna stampa)")
            scan_all_layers("strato")
        elif a.dry_run:
            print("\n[4] --dry-run: secondo scan (piatto nudo) al posto della stampa")
            scan_all_layers("scan")
        else:
            print("\n[4] stampa primo layer (FIRST_FLOOR_PRINT=True, auto-pausa)")
            kl("PRINT_PREPARED")      # lo START_PRINT del gcode salta il rilivellamento
            kl('SDCARD_PRINT_FILE FILENAME="%s" FIRST_FLOOR_PRINT=True' % os.path.basename(gpath))
            printing = True
            st = wait_state(("printing",), 600)
            print("    stato: %s" % st)
            st = wait_state(("paused",), 2400)
            if st != "paused":
                raise RuntimeError("la stampa non e' andata in pausa (stato %s)" % st)
            # Subito: il flag va spento ORA, altrimenti la stampa si riferma a
            # ogni cambio layer (vale anche se poi si riprende dal display).
            if clear_first_layer_flag():
                print("    flag FIRST_FLOOR_PRINT azzerato (niente ri-pausa ai layer successivi)")
            # Ventole da ripristinare alla ripresa: il raffreddamento pre-scan le
            # spegne, e FIRST_FLOOR_RESUME rimette solo P2. Lo slicer accende la
            # ventola del pezzo (P0) proprio prima del cambio layer 1->2, cioe' prima
            # della pausa: senza ripristino il resto del pezzo si stampa SENZA
            # raffreddamento (successo il 2026-11-06, stampa da buttare).
            fans_saved = save_fans()
            print("    ventole al momento della pausa: %s" % fans_saved)
            print("    primo layer stampato, in pausa. Raffreddo (step Creality: fan max + M104 140 + park)...")
            creality_cooldown()

            # MASTER-SERVER SPEGNE IL LASER QUANDO PARTE LA STAMPA (chiama
            # /usr/bin/laser_power.sh off in "print ready end"): la porta seriale
            # sparisce e va riaperta, altrimenti lo scan esce VUOTO.
            print("\n[4b] rialimento il laser (master-server lo spegne a inizio stampa)")
            if not L.up():
                raise RuntimeError("laser non ripristinato dopo la stampa: scan impossibile")
            if vexp is not None:
                L.set_exposure(vexp)

            print("\n[5] scan PRIMO STRATO (%d passata/e, stessa Y e velocita')" % len(bands))
            scan_all_layers("strato")
            kl("FIRST_FLOOR_PAUSE_POSITION")   # ri-parcheggio come l'originale

        # [6] ANALISI C2 (nostra, quantitativa): DECIDE lei.
        if not a.no_c2:
            print("\n[6] analisi quantitativa C2 (mappa di altezza)")
            try:
                t0 = time.time()
                c2, fa = analyze_c2(bands, tab_paths, scan_paths, segs, lz,
                                    sy0, sy1, n, a.line_width)
                print(fa.format_report(c2))
                print("    (%.1fs)" % (time.time() - t0))
            except Exception as e:
                # Non fatale, ma il verdetto resta indeterminato: senza C2 non
                # si riprende in automatico (vedi decisione finale).
                print("    C2 FALLITA: %s" % e)
                c2 = {"verdict": "UNCERTAIN",
                      "reasons": ["analisi C2 fallita: %s" % e], "metrics": {}}

        # [6b] detector Creality, un detect per banda (chiavi reali: scan1=tabella,
        # scan2=strato, gcode=reduction RITAGLIATO alla banda), verdetto = OR.
        # Solo informativo: si archivia accanto a C2 per confrontarli nel tempo.
        # Col driver diretto richiede di ridare per un attimo la porta al middleware.
        print("\n[6b] detector Creality per banda (informativo)")
        result = 0; any_fail = False
        for k in range(len(bands)):
            rk = L.detect_first_floor(tab_paths[k], scan_paths[k], gred_paths[k])
            ck, res_k = rk.get("code"), rk.get("result")
            print("    banda %d: %s" % (k, json.dumps(rk)))
            if ck != 0:
                any_fail = True
            elif res_k == 1:
                result = 1
        res = {"code": (0 if not any_fail else -1), "result": result}
    except BaseException:
        if not printing:
            # niente stampa in corso: non lasciare la stampante calda
            heaters_off()
        raise
    finally:
        if own:
            print("\n[7] chiusura laser - garantita")
            if L is not None:
                L.shutdown()
            release_lock()

        # ARCHIVIO: nel finally di proposito. Un run che ABORTISCE e' il caso
        # piu' prezioso (e' cosi' che si perde un difetto vero), quindi i dati
        # vanno salvati anche quando qualcosa e' andato storto.
        mode = ("scan-only" if a.scan_only else "dry-run" if a.dry_run else "print")
        archive_run({
            "stamp":      time.strftime("%Y%m%d_%H%M%S"),
            "tag":        re.sub(r"[^A-Za-z0-9_.-]", "_", os.path.basename(gpath))[:40],
            "mode":       mode,
            "gcode":      os.path.basename(gpath),
            "material":   a.material, "filament": fil, "hotend": hot, "bed": bed,
            "backend":    (L.name if L is not None else a.backend),
            "exposure":   vexp,
            "layer_h_mm": lz,
            "bbox_mm":    {"x0": x0, "x1": x1, "y0": y0, "y1": y1},
            "n_moves":    nmoves,
            # tutto cio' che serve a ri-analizzare i .temp OFFLINE (fl_analyze):
            "scan_y0_mm":   sy0,
            "scan_y1_mm":   sy1,
            "n_frames":     n,
            "y_step_mm":    (sy1 - sy0) / float(n - 1) if n > 1 else 0.0,
            "bands":        bands,
            "segs_mm":      [list(sg) for sg in segs],   # gcode completo del layer 1
            "scan_speeds":  [load_scan_speed(p) for p in tab_paths],
            "scan_stats":   SCAN_STATS,
            "laser_dx_center": LASER_DX_CENTER, "laser_dx_plan": LASER_DX_PLAN,
            "laser_dy": LASER_DY, "scan_margin_y": SCAN_MARGIN_Y, "scan_z": SCAN_Z,
            "lama_utile": LAMA_UTILE, "band_overlap": BAND_OVERLAP,
            "line_width": a.line_width,
            # ENTRAMBI i verdetti: serve a confrontarli run dopo run finche' C2
            # non ha abbastanza storia da poter mandare in pensione l'altro.
            "verdict_creality": res,
            "verdict_c2":  (None if c2 is None else
                            {"verdict": c2.get("verdict"),
                             "reasons": c2.get("reasons"),
                             "metrics": c2.get("metrics")}),
        }, tab_paths + scan_paths + gred_paths + [p + ".speed" for p in tab_paths])

    code, result = res.get("code"), res.get("result")
    cre = ("OK" if result == 0 else "ANOMALO") if code == 0 else "fallita(code %s)" % code
    c2v = (c2 or {}).get("verdict", "n/d")
    m = (c2 or {}).get("metrics", {}) or {}

    # Calibrazioni del filamento: la baseline di altezza misurata su uno strato
    # BUONO (dipende dalla riflettivita' del filamento: 1.61x PLA bianco, 1.7x
    # nero) e la quota di riflessi scartati, utile per scegliere l'esposizione.
    if c2v == "GOOD" and not a.dry_run:
        lay = [v for k, v in SCAN_STATS.items() if not k.startswith("tabella")]
        ghost = (sum(v["dropped"] for v in lay) / float(max(1, sum(v["raw_points"] for v in lay)))
                 if lay else None)
        prev = calib.get("elev_over_LH")
        save_calib(fil, elev_over_LH=m.get("elev_over_LH"),
                   ghost_frac=(round(ghost, 4) if ghost is not None else None),
                   exposure=(a.exposure if (a.remember and a.exposure) else None))
        print("    calibrazione '%s' aggiornata: altezza %.2fx layer%s"
              % (fil, m.get("elev_over_LH") or 0.0,
                 "" if prev is None else " (prima %.2fx)" % prev))

    echo(T("fl_result", verdict=c2v))
    # Altezza rispetto alle stampe BUONE precedenti dello stesso filamento: il rapporto
    # misurato/nominale dipende dal filamento (riflettivita'), quindi va confrontato con
    # il suo storico, non con 1.0. Solo informativo: non cambia il verdetto.
    base = calib.get("elev_over_LH"); cur = m.get("elev_over_LH")
    if base and cur and not a.dry_run:
        dev = (cur - base) / base
        if abs(dev) >= 0.30:
            echo(T("fl_height", pct="%+.0f" % (100 * dev), hint="%+.2f" % (m.get("z_offset_hint_mm") or 0.0)))
    print("\n" + "=" * 62)
    print(">>> PRIMO STRATO: %s <<<" % c2v)
    print("    C2 (decide):      %s" % c2v)
    print("    Creality (info):  %s" % cre)
    if c2v != "n/d" and ((c2v == "GOOD") != (code == 0 and result == 0)):
        # Divergenza attesa e gia' misurata (Creality sbaglia su bande parziali):
        # va comunque segnalata, perche' e' il dato che serve a validare C2.
        print("    NB: i due verdetti DIVERGONO - caso da conservare e rivedere.")
    for r in (c2 or {}).get("reasons", []):
        print("    - %s" % r)
    print("=" * 62)

    # Eco in CONSOLE KLIPPER (macro _LIDAR_SAY: M118 non esiste sulla K1): cosi'
    # l'esito e i difetti localizzati compaiono in Fluidd/display senza ssh.
    try:
        msg = ["LiDAR primo strato: %s (Creality: %s)" % (c2v, cre)]
        if m.get("coverage") is not None:
            msg.append("  copertura %.1f%%  mancante %.1f%%" % (
                100.0 * m["coverage"], 100.0 * m.get("missing_frac", 0.0)))
        if m.get("elev_med_um") is not None:
            msg.append("  altezza mediana %.0fum (%.2fx layer)" % (
                m["elev_med_um"], m.get("elev_over_LH") or 0.0))
        for r in (c2 or {}).get("reasons", [])[:6]:
            msg.append("  - %s" % r)
        for line in msg:
            kl(say_cmd(line.replace("\n", " ")), t=20)
    except Exception as e:
        print("    (eco in console non riuscito: %s)" % e)

    if not a.dry_run and not a.scan_only:
        st = kl_query({"print_stats": ["state"]}).get("print_stats", {}).get("state", "?")
        if st == "paused":
            # DECIDE C2. UNCERTAIN non riprende MAI in automatico.
            if a.cancel or (c2v == "BAD" and (a.on_bad == "cancel" or not a.resume)):
                print("    annullo la stampa (C2: BAD)." if c2v == "BAD"
                      else "    annullo la stampa (--cancel).")
                kl_rpc("pause_resume/cancel")
                heaters_off()
                echo(T("fl_cancel"))
            elif a.resume and c2v == "GOOD":
                # Il raffreddamento pre-scan ha portato l'ugello a g28_ext_temp
                # (~140C): riprendere cosi' fa fallire la stampa con "Extrude below
                # minimum temp". FIRST_FLOOR_RESUME non scalda: M109 prima.
                print("    C2 GOOD: ripristino temperatura di stampa (M109 S%d)..." % hot)
                kl("M109 S%d" % hot, t=600)
                restore_fans(fans_saved)
                print("    riprendo la stampa (FIRST_FLOOR_RESUME).")
                echo(T("fl_resume"))
                kl("FIRST_FLOOR_RESUME")
            else:
                # BAD + --resume NON annulla: finche' C2 non e' validato sul campo
                # non si distrugge una stampa per un verdetto forse sbagliato.
                why = {"UNCERTAIN": "verdetto INCERTO: dati insufficienti per decidere",
                       "BAD": "verdetto BAD, ma con --resume non annullo da solo: "
                              "controlla il pezzo (usa --cancel per annullare in automatico)",
                       }.get(c2v, "nessun --resume/--cancel")
                print("    stampa lasciata in PAUSA (%s)." % why)
                print("    decidi tu: resume dal display, oppure pause_resume/cancel.")
                echo(T("fl_pause", why=c2v))
    else:
        # niente stampa in ballo (--dry-run/--scan-only): libera il piatto,
        # SENZA M84 (il display chiederebbe di rifare l'home)
        if own:
            park()
            heaters_off()
    return {"verdict": c2v, "creality": cre, "c2": c2}


FANS = (0, 2)       # P0 = ventola del pezzo, P2 = ventola laterale (P1 = camera, non la tocchiamo)


def save_fans():
    """Valori (0..1) delle ventole P0/P2 letti dagli output_pin di Klipper."""
    objs = {"output_pin fan%d" % p: ["value"] for p in FANS}
    objs["gcode_macro PRINTER_PARAM"] = ["fan0_min", "fan2_min"]
    st = kl_query(objs)
    par = st.get("gcode_macro PRINTER_PARAM", {})
    out = {}
    for p in FANS:
        out[p] = {"value": float(st.get("output_pin fan%d" % p, {}).get("value") or 0.0),
                  "min": float(par.get("fan%d_min" % p) or 0.0)}
    return out


def restore_fans(saved):
    """Rimette le ventole come al momento della pausa. La macro M106 della K1
    rimappa S in [fanN_min, 255]: per ritrovare lo stesso valore serve la sua
    inversa, la stessa formula che FIRST_FLOOR_RESUME usa per P2."""
    if not saved:
        print("    ATTENZIONE: stato ventole non salvato, non le ripristino")
        return
    for p, f in sorted(saved.items()):
        v, fmin = f["value"], f["min"]
        s = 0 if v <= 0 else max(0.0, min(255.0, (v * 255.0 - fmin) * 255.0 / (255.0 - fmin)))
        kl("M106 P%d S%d" % (p, int(round(s))))
        print("    ventola P%d ripristinata (valore %.2f -> M106 S%d)" % (p, v, int(round(s))))


def prep_scan_only():
    """--scan-only: l'oggetto e' gia' sul piatto. Niente livellamento (muoverebbe lo
    Z e invaliderebbe il confronto con la tabella gia' acquisita) e niente stampa."""
    print("    --scan-only: riuso la tabella esistente, non stampo e non livello")
    homed = kl_query({"toolhead": ["homed_axes"]}).get("toolhead", {}).get("homed_axes", "")
    if "z" in homed.lower():
        return
    # RIFARE LO ZERO Z QUI E' DELICATO: su questa stampante lo zero si stabilisce
    # TOCCANDO IL PIATTO CON L'UGELLO (prtouch) e il PEZZO E' SUL PIATTO:
    #  (1) la punta sporca di plastica SOLIDIFICATA falsa lo zero di decine di um;
    #  (2) il punto di sonda puo' capitare SULL'OGGETTO -> zero sbagliato in modo
    #      NON uniforme (C2 rimuove un offset globale, non una sonda sul pezzo).
    # Mitigazione: scaldare a g28_ext_temp e PULIRE la punta prima di sondare.
    print("    ATTENZIONE: assi non homati e PEZZO SUL PIATTO.")
    print("    Lo zero Z va rifatto toccando il piatto: pulisco prima la punta.")
    try:
        g28t = kl_query({"custom_macro": ["g28_ext_temp"]}).get("custom_macro", {}).get("g28_ext_temp")
        if g28t:
            print("    riscaldo a %sC e pulisco l'ugello..." % g28t)
            kl("M109 S%d" % int(g28t), t=600)
            kl("CX_NOZZLE_CLEAR", t=600)
        else:
            print("    g28_ext_temp non leggibile: salto la pulizia")
    except Exception as e:
        print("    pulizia ugello non riuscita (%s): proseguo comunque" % e)
    kl("G28")
    try: kl("M104 S0")
    except Exception: pass
    print("    NOTA: lo zero Z e' stato RIFATTO: C2 rimuove lo scostamento globale del")
    print("    piano, quindi i difetti LOCALI restano validi; l'altezza ASSOLUTA no.")
    print("    Per evitarlo: non spegnere i motori tra la stampa e lo scan.")


if __name__ == "__main__":
    main()
