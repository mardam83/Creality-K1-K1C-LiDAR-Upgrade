# k1lidar/store.py - calibrazioni persistenti PER FILAMENTO.
#
# File JSON fuori dalla cartella del toolkit (un reinstall/aggiornamento non lo
# tocca): /usr/data/lidar_calib.json
#   {"filaments": {"<chiave>": {"pa": 0.03, "pa_confidence": ..., "hotend": 220,
#                               "elev_over_LH": 1.31, "exposure": ..., "history": {...}}}}
#
# La chiave e' il nome del PROFILO FILAMENTO dello slicer (filament_settings_id nel
# gcode) normalizzato + la temperatura dell'ugello, es.
#   "Creality HF Generic Speed PLA - Copia" a 220 C -> "Creality_HF_Generic_Speed_PLA_Copia_T220".
# Due filamenti diversi devono avere due profili diversi nello slicer: e' il nome del
# profilo che li distingue. Il nome originale si salva in "name" (per gli elenchi).
import json, os, re, time

CALIB = "/usr/data/lidar_calib.json"
HISTORY_LEN = 20


def normalize_key(name):
    """Nome leggibile -> chiave stabile: lettere, cifre e punto; ogni altra sequenza -> '_'."""
    k = re.sub(r"[^A-Za-z0-9.]+", "_", str(name or "").strip()).strip("_.")
    return k[:60] or "default"


def _load_db():
    try:
        with open(CALIB) as f:
            db = json.load(f)
        return db if isinstance(db, dict) else {}
    except Exception:
        return {}


def _write_db(db):
    tmp = CALIB + ".tmp"
    with open(tmp, "w") as f:
        json.dump(db, f, indent=2, sort_keys=True)
    os.rename(tmp, CALIB)          # scrittura atomica


def load_calib(key):
    """Calibrazioni salvate per il filamento `key` (dict, vuoto se nessuna)."""
    return _load_db().get("filaments", {}).get(normalize_key(key), {})


def save_calib(key, **values):
    """Aggiorna le calibrazioni del filamento `key`. I valori None vengono ignorati.
    Tiene uno storico breve per ogni grandezza (serve a vedere se un valore deriva)."""
    try:
        db = _load_db()
        fil = db.setdefault("filaments", {}).setdefault(normalize_key(key), {})
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        for k, v in values.items():
            if v is None:
                continue
            fil[k] = v
            if isinstance(v, (int, float)) and not isinstance(v, bool):   # storico solo dei numeri
                hist = fil.setdefault("history", {}).setdefault(k, [])
                hist.append([stamp, v])
                del hist[:-HISTORY_LEN]
        fil["updated"] = stamp
        _write_db(db)
        return True
    except Exception as e:
        print("    ATTENZIONE: calibrazione non salvata (%s)" % e)
        return False


def get_pa(key, legacy=None):
    """PA salvato per il filamento, o None. `legacy` = chiave alternativa da provare
    (es. il nome senza temperatura, usato dalle versioni precedenti)."""
    v = load_calib(key).get("pa")
    if v is None and legacy:
        v = load_calib(legacy).get("pa")
    try:
        return float(v) if v is not None else None
    except (TypeError, ValueError):
        return None


def list_filaments():
    """[(chiave, dict)] ordinati per nome."""
    return sorted(_load_db().get("filaments", {}).items())


def forget(key):
    """Cancella le calibrazioni di un filamento. Ritorna True se esisteva."""
    db = _load_db()
    fil = db.get("filaments", {})
    k = normalize_key(key)
    if k not in fil:
        return False
    del fil[k]
    _write_db(db)
    return True
