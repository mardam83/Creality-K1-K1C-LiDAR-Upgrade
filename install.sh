#!/bin/sh
# install.sh - install the K1 LiDAR toolkit (PA calibration + first layer check).
#
# Run ON THE PRINTER as root. Easiest way, from a PC: install-from-pc.bat (Windows) or
# install-from-pc.sh (Linux/macOS), which copy the toolkit over SSH and run this script
# through update.sh. Or on the printer, from the folder you cloned:
#   sh /usr/data/k1-lidar/install.sh [--with-hook] [--lang it]
#
# Options:
#   --with-hook   LiDAR features at every print start (wraps SDCARD_PRINT_FILE)
#   --no-hook     remove the print start hook
#   --lang en|it  language of the messages (default: en)
#   --check       only check the prerequisites, change nothing
#   --force       install even on an untested printer model (NOT recommended)
#
# Safe to run again (after a firmware update, or to repair): it is idempotent.
# Config and Moonraker component are SYMLINKS into this folder, so an update via
# Fluidd/Mainsail (update manager, git pull) updates them too.
set -e

DIR="$(cd "$(dirname "$0")" && pwd)"
CFGDIR=/usr/data/printer_data/config
PRINTER_CFG="$CFGDIR/printer.cfg"
MOON_CFG="$CFGDIR/moonraker.conf"
VARS_FILE="$CFGDIR/lidar_vars.cfg"
SYSCFG=/usr/data/creality/userdata/config/system_config.json
LASER_INFO=/usr/data/creality/userdata/config/laser_info.json
STAMP=$(date +%Y%m%d_%H%M%S)
TESTED_FW="1.3.5.22"

WITH_HOOK=0; NO_HOOK=0; LANG_SET=""; CHECK_ONLY=0; FORCE=0
while [ $# -gt 0 ]; do
    case "$1" in
        --with-hook) WITH_HOOK=1 ;;
        --no-hook) NO_HOOK=1 ;;
        --lang) LANG_SET="$2"; shift ;;
        --check) CHECK_ONLY=1 ;;
        --force) FORCE=1 ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "unknown option: $1"; exit 1 ;;
    esac
    shift
done

ok()   { echo "  [ok]   $1"; }
warn() { echo "  [WARN] $1"; }
err()  { echo "  [ERR]  $1"; FAIL=1; }
FAIL=0

json_get() {   # $1 file, $2 python expression on d
    python3 -c "import json,sys; d=json.load(open('$1')); print($2)" 2>/dev/null || true
}

echo "== K1 LiDAR toolkit: checks =="
[ "$(id -u)" = "0" ] && ok "root" || err "run as root"

MODEL=$(json_get "$SYSCFG" "d['device_info']['model_str']")
if [ "$MODEL" = "CR-K1" ]; then
    ok "printer: Creality K1 ($MODEL)"
elif [ "$FORCE" = "1" ]; then
    warn "printer '$MODEL' is NOT tested (only CR-K1): continuing because of --force"
else
    err "printer '${MODEL:-unknown}' is not supported (only Creality K1 with the LiDAR module)"
fi
if [ -f "$LASER_INFO" ] && grep -q '"laser"' "$LASER_INFO"; then
    ok "LiDAR module declared (laser_info.json)"
else
    err "no LiDAR module declared in $LASER_INFO"
fi
FW=$(json_get /usr/data/creality/userdata/config/system_version.json "d.get('sys_version','?')")
[ "$FW" = "$TESTED_FW" ] && ok "firmware $FW (tested)" || warn "firmware ${FW:-?}: tested only on $TESTED_FW"

[ -S /tmp/klippy_uds ] && ok "Klipper running" || err "Klipper socket /tmp/klippy_uds missing"
MOON_COMP=""
for d in /usr/data/moonraker/moonraker/moonraker/components /usr/data/moonraker/moonraker/components; do
    [ -d "$d" ] && MOON_COMP="$d" && break
done
[ -n "$MOON_COMP" ] && ok "Moonraker components: $MOON_COMP" \
    || err "Moonraker not found (install it with the Creality Helper-Script)"
[ -f "$MOON_CFG" ] && ok "moonraker.conf" || err "$MOON_CFG missing (a firmware update may have deleted it)"
python3 -c 'import numpy' 2>/dev/null && ok "numpy" || err "numpy missing in python3"
python3 -c 'import termios, select' 2>/dev/null && ok "serial access (direct laser driver)" \
    || warn "termios missing: only the middleware backend will work"
[ -x /usr/bin/laser_power.sh ] && ok "laser_power.sh (laser power on GPIO)" \
    || warn "/usr/bin/laser_power.sh missing: the laser may not power on (old firmware?)"
pidof cx_ai_middleware >/dev/null 2>&1 && ok "cx_ai_middleware (fallback + Creality detector)" \
    || warn "cx_ai_middleware not running: no fallback backend"
grep -rq "CX_PRINT_LEVELING_CALIBRATION" /usr/share/klipper/klippy/extras/ 2>/dev/null \
    && ok "factory leveling macros (CX_*)" || warn "CX_* leveling macros not found"

# overlay full -> Klipper can't save its config. pip's cache filled it twice on our K1.
FREE=$(df -k / | awk 'NR==2{print $4}')
if [ "$FREE" -lt 5120 ] && [ -d /root/.cache/pip ] && [ "$CHECK_ONLY" = "0" ]; then
    rm -rf /root/.cache/pip; ok "root filesystem was full: removed pip cache"
else
    ok "root filesystem: ${FREE}KB free"
fi

# install.sh riavvia Klipper e Moonraker: mai durante una stampa o un job LiDAR
if [ "$CHECK_ONLY" = "0" ] && [ -f "$DIR/update.sh" ] && BUSY=$(sh "$DIR/update.sh" --busy-check); then
    err "$BUSY: install when it is finished"
fi

if [ "$FAIL" = "1" ]; then
    echo; echo "Prerequisites NOT met: fix the [ERR] lines above."; exit 1
fi
if [ "$CHECK_ONLY" = "1" ]; then
    echo; echo "--check: everything OK, nothing changed."; exit 0
fi

echo; echo "== Python self-tests =="
( cd "$DIR" && python3 laser_driver.py --selftest >/dev/null ) && ok "laser driver" \
    || { err "laser_driver.py self-test failed"; exit 1; }
( cd "$DIR" && python3 -c 'import k1lidar.job, k1lidar.pa, k1lidar.firstlayer' ) && ok "k1lidar modules" \
    || { err "k1lidar modules do not import"; exit 1; }
( cd "$DIR" && python3 -c 'import sys; sys.path.insert(0, "algo"); import pa_phys' ) && ok "PA analyzer" \
    || { err "algo/pa_phys.py does not import"; exit 1; }

echo; echo "== Klipper config =="
BACKED_UP=0
backup() {
    [ "$BACKED_UP" = "1" ] && return
    cp "$PRINTER_CFG" "$PRINTER_CFG.k1lidar_bak_$STAMP"; BACKED_UP=1
    ok "backup printer.cfg -> printer.cfg.k1lidar_bak_$STAMP"
}
has_include() { grep -q "^\[include $1\]" "$PRINTER_CFG"; }
add_include() {
    has_include "$1" && { ok "[include $1] present"; return; }
    backup
    LAST=$(grep -n '^\[include ' "$PRINTER_CFG" | tail -1 | cut -d: -f1)
    if [ -n "$LAST" ]; then sed -i "${LAST}a [include $1]" "$PRINTER_CFG"; else sed -i "1i [include $1]" "$PRINTER_CFG"; fi
    ok "[include $1] added"
}
del_include() {
    has_include "$1" || return 0
    backup; sed -i "/^\[include $(echo "$1" | sed 's/\./\\./g')\]/d" "$PRINTER_CFG"
    ok "[include $1] removed"
}
link_cfg() {
    if [ -L "$CFGDIR/$1" ] && [ "$(readlink "$CFGDIR/$1")" = "$DIR/config/$1" ]; then
        ok "$1 -> toolkit"
    else
        [ -e "$CFGDIR/$1" ] && mv "$CFGDIR/$1" "$CFGDIR/$1.k1lidar_bak_$STAMP"
        ln -s "$DIR/config/$1" "$CFGDIR/$1"; ok "$1 linked"
    fi
}

# migration from the previous (private) version of the toolkit
for old in lidar_buttons.cfg lidar_print_hook.cfg lidar_firstlayer_check.cfg; do del_include "$old"; done

link_cfg k1lidar.cfg
add_include k1lidar.cfg
# [save_variables]: only one allowed in Klipper. Use ours only if there is no other
# INCLUDED one (old files of our previous version don't count once un-included).
OTHER_SV=""
for f in $(grep -l '^\[save_variables\]' "$CFGDIR"/*.cfg 2>/dev/null || true); do
    b=$(basename "$f")
    case "$b" in k1lidar_vars.cfg|lidar_buttons.cfg|*k1lidar_bak*) continue ;; esac
    if [ "$b" = "printer.cfg" ] || has_include "$b"; then OTHER_SV="$b"; fi
done
if [ -n "$OTHER_SV" ]; then
    ok "existing [save_variables] in $OTHER_SV: using it"
    del_include k1lidar_vars.cfg
else
    link_cfg k1lidar_vars.cfg
    add_include k1lidar_vars.cfg
fi
if [ "$WITH_HOOK" = "1" ]; then
    link_cfg k1lidar_hook.cfg; add_include k1lidar_hook.cfg
elif [ "$NO_HOOK" = "1" ]; then
    del_include k1lidar_hook.cfg
elif has_include k1lidar_hook.cfg; then
    ok "print start hook installed (--no-hook to remove)"
fi

if [ -n "$LANG_SET" ]; then
    case "$LANG_SET" in en|it) ;; *) err "--lang: en or it"; exit 1 ;; esac
    python3 - "$VARS_FILE" "$LANG_SET" <<'PY'
import sys, os
path, lang = sys.argv[1], sys.argv[2]
lines = open(path).read().splitlines() if os.path.exists(path) else ["[Variables]"]
lines = [l for l in lines if not l.startswith("lidar_lang")]
if "[Variables]" not in lines:
    lines.insert(0, "[Variables]")
lines.insert(lines.index("[Variables]") + 1, "lidar_lang = '%s'" % lang)
open(path, "w").write("\n".join(lines) + "\n")
PY
    ok "language: $LANG_SET"
fi

echo; echo "== Moonraker =="
# migration: old component
if [ -e "$MOON_COMP/lidar_calibration.py" ]; then rm -f "$MOON_COMP/lidar_calibration.py"; ok "old component lidar_calibration removed"; fi
if [ -L "$MOON_COMP/k1lidar.py" ] && [ "$(readlink "$MOON_COMP/k1lidar.py")" = "$DIR/moonraker/k1lidar.py" ]; then
    ok "component k1lidar -> toolkit"
else
    rm -f "$MOON_COMP/k1lidar.py"; ln -s "$DIR/moonraker/k1lidar.py" "$MOON_COMP/k1lidar.py"; ok "component k1lidar linked"
fi
ORIGIN=""
if [ -d "$DIR/.git" ]; then ORIGIN=$(cd "$DIR" && git remote get-url origin 2>/dev/null || true); fi
cp "$MOON_CFG" "$MOON_CFG.k1lidar_bak_$STAMP"
python3 - "$MOON_CFG" "$DIR" "$ORIGIN" <<'PY'
import re, sys
path, d, origin = sys.argv[1:4]
txt = open(path).read()
def drop(section, t):
    # toglie una sezione [section] fino alla prossima sezione
    return re.sub(r"(?ms)^\[%s\][^\[]*" % re.escape(section), "", t)
txt = drop("lidar_calibration", txt)
txt = drop("k1lidar", txt)
txt = drop("update_manager k1-lidar", txt)
txt = txt.rstrip() + "\n\n[k1lidar]\npath: %s\n" % d
if origin:
    txt += ("\n[update_manager k1-lidar]\ntype: git_repo\nchannel: dev\npath: %s\norigin: %s\n"
            "primary_branch: main\nmanaged_services: klipper moonraker\n" % (d, origin))
open(path, "w").write(txt)
PY
ok "moonraker.conf: [k1lidar]$( [ -n "$ORIGIN" ] && echo " + [update_manager k1-lidar]" )"

echo; echo "== Restart =="
if [ -x /etc/init.d/S55klipper_service ]; then /etc/init.d/S55klipper_service restart >/dev/null 2>&1 && ok "Klipper restarted"
else python3 "$DIR/klctl.py" restart >/dev/null 2>&1 && ok "Klipper restarted (FIRMWARE_RESTART)" || warn "restart Klipper by hand"; fi
# Il "restart" di Creality fa stop + 1 s + start, ma Moonraker impiega di piu' a
# chiudersi: lo start trova il vecchio processo vivo e non fa nulla -> Moonraker
# resta spento. Quindi: stop, attesa vera della chiusura, start staccato dal terminale.
restart_moonraker() {
    S=/etc/init.d/S56moonraker_service
    [ -x "$S" ] || { warn "restart Moonraker by hand"; return; }
    $S stop >/dev/null 2>&1 || true
    i=0; while [ -f /var/run/moonraker.pid ] && kill -0 "$(cat /var/run/moonraker.pid 2>/dev/null)" 2>/dev/null && [ $i -lt 30 ]; do sleep 1; i=$((i+1)); done
    setsid $S start >/dev/null 2>&1 < /dev/null
    sleep 2
    if kill -0 "$(cat /var/run/moonraker.pid 2>/dev/null)" 2>/dev/null; then ok "Moonraker restarted"
    else warn "Moonraker did not restart: run $S start"; fi
}
restart_moonraker

echo
echo "Done. In Fluidd/Mainsail -> Macros (LIDAR_HELP prints a short guide):"
echo "  LIDAR_PA_CALIBRATE GCODE=\"a file sliced with the filament profile\"   (find the PA, no print)"
echo "  LIDAR_FLOW MODE=1 / LIDAR_FIRSTLAYER ON=1           (before every print, needs --with-hook)"
echo "  LIDAR_STATUS, LIDAR_PA_LIST, LIDAR_PA_SET, LIDAR_CANCEL"
