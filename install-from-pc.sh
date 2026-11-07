#!/bin/sh
# install-from-pc.sh - install or update the K1 LiDAR toolkit on the printer, from a PC
# (Linux, macOS, Windows Git Bash). Needs only ssh and tar; asks the root password once.
#
#   sh install-from-pc.sh PRINTER_IP [install options]
#
#   first install:   sh install-from-pc.sh 192.168.1.xx --with-hook --lang it
#   update:          sh install-from-pc.sh 192.168.1.xx
#
# Options (passed to install.sh on the printer):
#   --with-hook   LiDAR features at every print start   --no-hook   remove them
#   --lang en|it  message language                        --force     untested model
# An update keeps hook, language, settings and calibrations: no options needed.
# The printer must be idle (no print running). Root password: the one shown on the
# printer screen when you enable root access (Settings -> Root account).
set -e
cd "$(dirname "$0")"

HOST="$1"
if [ -z "$HOST" ] || [ "$HOST" = "-h" ] || [ "$HOST" = "--help" ]; then
    sed -n '2,16p' "$0"; exit 1
fi
shift
for o in "$@"; do
    case "$o" in
        --with-hook|--no-hook|--force|--lang|en|it) ;;
        *) echo "unknown option: $o"; exit 1 ;;
    esac
done

command -v ssh >/dev/null 2>&1 || { echo "ssh not found"; exit 1; }
command -v tar >/dev/null 2>&1 || { echo "tar not found"; exit 1; }
[ -f MANIFEST ] || { echo "MANIFEST missing: run me from the toolkit folder"; exit 1; }

LIST=$(mktemp)
trap 'rm -f "$LIST"' EXIT
grep -v '^#' MANIFEST | grep -v '^[[:space:]]*$' | tr -d '\r' > "$LIST"
while read -r f; do
    [ -e "$f" ] || { echo "missing file: $f"; exit 1; }
done < "$LIST"

STAGE=/usr/data/k1-lidar.new
REMOTE="rm -rf $STAGE && mkdir -p $STAGE && tar xzf - -C $STAGE && sh $STAGE/update.sh --apply $STAGE $*"

echo "== K1 LiDAR toolkit -> root@$HOST =="
echo "(the root password is asked once)"
tar czf - --exclude=__pycache__ --exclude='*.pyc' -T "$LIST" \
    | ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10 "root@$HOST" "$REMOTE"
