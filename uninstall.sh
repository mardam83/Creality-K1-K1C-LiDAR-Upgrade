#!/bin/sh
# uninstall.sh - remove the K1 LiDAR toolkit from Klipper and Moonraker.
#   sh uninstall.sh [--purge]
# Keeps your calibrations (/usr/data/lidar_calib.json) and settings (lidar_vars.cfg)
# unless --purge is given. The toolkit folder itself is not deleted.
set -e
CFGDIR=/usr/data/printer_data/config
PRINTER_CFG="$CFGDIR/printer.cfg"
MOON_CFG="$CFGDIR/moonraker.conf"
STAMP=$(date +%Y%m%d_%H%M%S)
PURGE=0; [ "$1" = "--purge" ] && PURGE=1

cp "$PRINTER_CFG" "$PRINTER_CFG.k1lidar_uninst_$STAMP"
for f in k1lidar.cfg k1lidar_vars.cfg k1lidar_hook.cfg; do
    sed -i "/^\[include $(echo "$f" | sed 's/\./\\./g')\]/d" "$PRINTER_CFG"
    [ -L "$CFGDIR/$f" ] && rm -f "$CFGDIR/$f"
done
echo "  [ok] Klipper config cleaned (backup printer.cfg.k1lidar_uninst_$STAMP)"

for d in /usr/data/moonraker/moonraker/moonraker/components /usr/data/moonraker/moonraker/components; do
    [ -L "$d/k1lidar.py" ] && rm -f "$d/k1lidar.py"
done
if [ -f "$MOON_CFG" ]; then
    cp "$MOON_CFG" "$MOON_CFG.k1lidar_uninst_$STAMP"
    python3 - "$MOON_CFG" <<'PY'
import re, sys
p = sys.argv[1]; t = open(p).read()
for s in ("k1lidar", "update_manager k1-lidar"):
    t = re.sub(r"(?ms)^\[%s\][^\[]*" % re.escape(s), "", t)
open(p, "w").write(t.rstrip() + "\n")
PY
    echo "  [ok] moonraker.conf cleaned"
fi
if [ "$PURGE" = "1" ]; then
    rm -f /usr/data/lidar_calib.json "$CFGDIR/lidar_vars.cfg"
    echo "  [ok] calibrations and settings deleted"
fi
[ -x /etc/init.d/S55klipper_service ] && /etc/init.d/S55klipper_service restart >/dev/null 2>&1
if [ -x /etc/init.d/S56moonraker_service ]; then
    /etc/init.d/S56moonraker_service stop >/dev/null 2>&1 || true
    i=0; while [ -f /var/run/moonraker.pid ] && kill -0 "$(cat /var/run/moonraker.pid 2>/dev/null)" 2>/dev/null && [ $i -lt 30 ]; do sleep 1; i=$((i+1)); done
    setsid /etc/init.d/S56moonraker_service start >/dev/null 2>&1 < /dev/null
fi
echo "Removed. The toolkit folder and the LiDAR archive (/usr/data/lidar_archive) are still there."
