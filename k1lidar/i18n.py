# k1lidar/i18n.py - messaggi per l'utente (console Klipper / Fluidd / display) in EN e IT.
#
# Lingua: variabile d'ambiente K1LIDAR_LANG ("en" | "it"), che il componente Moonraker
# imposta dal registro `lidar_lang`. Default inglese. I log tecnici dettagliati restano
# nella lingua in cui sono scritti nel codice.
import os

MSG = {
    "pa_start": ("LiDAR PA calibration: filament {fil}, nozzle {hot}C, bed {bed}C",
                 "Taratura PA LiDAR: filamento {fil}, ugello {hot}C, piatto {bed}C"),
    "level": ("LiDAR: homing and bed leveling...", "LiDAR: homing e livellamento piatto..."),
    "laser": ("LiDAR: laser on, self-calibration...", "LiDAR: laser acceso, auto-calibrazione..."),
    "scan_table": ("LiDAR: scanning bare bed...", "LiDAR: scansione piatto nudo..."),
    "print_pattern": ("LiDAR: printing PA test pattern...", "LiDAR: stampa del pattern PA..."),
    "cooldown": ("LiDAR: cooling nozzle before scanning...", "LiDAR: raffreddo l'ugello prima dello scan..."),
    "scan_lines": ("LiDAR: scanning printed lines...", "LiDAR: scansione delle linee stampate..."),
    "pa_result": ("LiDAR PA result: {pa} (80% range {lo}-{hi}), confidence {conf}",
                  "Risultato PA LiDAR: {pa} (intervallo 80% {lo}-{hi}), confidenza {conf}"),
    "conf_high": ("high", "alta"),
    "conf_low": ("LOW", "BASSA"),
    "pa_fine": ("LiDAR: fine pass around {pa} (step 0.001)...",
                "LiDAR: passata fine attorno a {pa} (passo 0.001)..."),
    "pa_saved": ("PA {pa} saved for '{fil}' and applied", "PA {pa} salvato per '{fil}' e applicato"),
    "pa_low_conf": ("WARNING: low confidence - the PA was saved, but consider repeating "
                    "(clean plate, or FINE=1)",
                    "ATTENZIONE: confidenza bassa - il PA e' stato salvato, ma valuta di ripetere "
                    "(piatto pulito, oppure FINE=1)"),
    "pa_fail": ("LiDAR PA calibration FAILED: {err}", "Taratura PA LiDAR FALLITA: {err}"),
    "pa_reuse": ("LiDAR: using saved PA {pa} for '{fil}' (no test print)",
                 "LiDAR: uso il PA salvato {pa} per '{fil}' (nessun pattern)"),
    "pa_new": ("LiDAR: filament '{fil}' not calibrated yet: running PA calibration",
               "LiDAR: filamento '{fil}' non ancora tarato: eseguo la taratura PA"),
    "slicer_pa": ("WARNING: the slicer sets its own PA ({pa}) in this gcode and will override ours",
                  "ATTENZIONE: lo slicer imposta un suo PA ({pa}) in questo gcode e sovrascrivera' il nostro"),
    "fl_start": ("LiDAR first layer check: {file}", "Controllo primo strato LiDAR: {file}"),
    "fl_result": ("LiDAR first layer: {verdict}", "Primo strato LiDAR: {verdict}"),
    "fl_height": ("LiDAR: first layer {pct}% vs. the usual height for this filament "
                  "(suggested z-offset change {hint} mm, info only)",
                  "LiDAR: primo strato {pct}% rispetto all'altezza abituale di questo filamento "
                  "(z-offset suggerito {hint} mm, solo informativo)"),
    "fl_resume": ("LiDAR: first layer OK, resuming print", "LiDAR: primo strato OK, riprendo la stampa"),
    "fl_pause": ("LiDAR: print left PAUSED ({why}) - resume or cancel from the screen",
                 "LiDAR: stampa in PAUSA ({why}) - riprendi o annulla dallo schermo"),
    "fl_cancel": ("LiDAR: first layer BAD, print cancelled", "LiDAR: primo strato DIFETTOSO, stampa annullata"),
    "job_print": ("LiDAR: starting print {file}", "LiDAR: avvio la stampa {file}"),
    "pa_noname": ("NOTE: no FILAMENT or GCODE given: the PA is saved as '{key}', which your "
                  "slicer profiles will NOT find. Use GCODE=<a file sliced with the profile> "
                  "or FILAMENT=<exact slicer profile name>.",
                  "NOTA: senza FILAMENT ne' GCODE il PA viene salvato come '{key}', che i "
                  "profili del tuo slicer NON troveranno. Usa GCODE=<un file fatto con quel "
                  "profilo> oppure FILAMENT=<nome esatto del profilo nello slicer>."),
    "pa_set": ("PA {pa} saved for profile '{fil}' at {hot}C", "PA {pa} salvato per il profilo '{fil}' a {hot}C"),
    "list_head": ("LiDAR - calibrated filament profiles:", "LiDAR - profili filamento tarati:"),
    "list_empty": ("LiDAR: no filament profile calibrated yet", "LiDAR: nessun profilo filamento tarato"),
    "manual": ("set by hand", "impostato a mano"),
    "busy": ("LiDAR: another operation is running", "LiDAR: c'e' gia' un'operazione in corso"),
    "error": ("LiDAR ERROR: {err}", "ERRORE LiDAR: {err}"),
}


def lang():
    v = (os.environ.get("K1LIDAR_LANG") or "en").lower()
    return "it" if v.startswith("it") else "en"


def T(key, **kw):
    en, it = MSG[key]
    s = it if lang() == "it" else en
    try:
        return s.format(**kw)
    except (KeyError, IndexError):
        return s
