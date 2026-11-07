#!/bin/sh
# update.sh - install or update the K1 LiDAR toolkit ON THE PRINTER (run as root).
#
#   sh /usr/data/k1-lidar/update.sh                 download the latest version and install it
#   sh /usr/data/k1-lidar/update.sh --rollback      go back to the version before the last update
#   sh update.sh --apply DIR [install.sh options]   install the toolkit unpacked in DIR
#                                                   (used by install-from-pc.sh / .bat)
#
# Options passed on to install.sh: --with-hook, --no-hook, --lang en|it, --force.
# An update keeps your settings (hook, language, LIDAR_FLOW...) and calibrations: they
# live outside the toolkit folder (/usr/data/lidar_calib.json, printer_data/config).
#
# How: the new version is unpacked next to the old one, the folders are swapped
# (/usr/data/k1-lidar.old = previous version) and install.sh is run. If install.sh
# fails, the previous version is put back automatically.
# Refuses to run while a print or a LiDAR job is running (it restarts Klipper).
#
# Download source: K1LIDAR_URL (tar.gz of the repo), default below. If the folder is a
# git checkout, "git pull" is used instead.
set -e

DEST="${K1LIDAR_DEST:-/usr/data/k1-lidar}"
OLD="$DEST.old"
STAGE="$DEST.new"
URL="${K1LIDAR_URL:-https://github.com/<user>/k1-lidar/archive/refs/heads/main.tar.gz}"

ok()  { echo "  [ok]   $1"; }
die() { echo "  [ERR]  $1"; exit 1; }

[ "$(id -u)" = "0" ] || die "run as root"

printer_busy() {
    # stampa in corso o in pausa -> non si riavvia Klipper sotto i piedi
    [ -f /tmp/k1lidar.lock ] && { echo "a LiDAR job is running (LIDAR_CANCEL to stop it)"; return 0; }
    [ -S /tmp/klippy_uds ] || return 1
    python3 - <<'PY' && return 0 || return 1
import json, socket, sys
try:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.settimeout(5)
    s.connect("/tmp/klippy_uds")
    s.sendall(json.dumps({"id": 1, "method": "objects/query",
                          "params": {"objects": {"print_stats": ["state"]}}}).encode() + b"\x03")
    buf = b""
    while not buf.endswith(b"\x03"):
        c = s.recv(4096)
        if not c:
            break
        buf += c
    st = json.loads(buf.rstrip(b"\x03").decode())["result"]["status"]["print_stats"]["state"]
except Exception:
    sys.exit(1)              # Klipper non risponde: non blocchiamo
if st in ("printing", "paused"):
    print("the printer is %s" % st)
    sys.exit(0)
sys.exit(1)
PY
}

check_idle() {
    if MSG=$(printer_busy); then die "$MSG: try again when it is finished"; fi
}

fix_eol() {   # file scaricati su Windows con fine riga CRLF: la shell non li digerisce
    find "$1" -type f \( -name '*.sh' -o -name '*.py' -o -name '*.cfg' \) -exec sed -i 's/\r$//' {} +
}

swap_in() {   # $1 = cartella con la nuova versione
    [ -f "$1/install.sh" ] && [ -d "$1/k1lidar" ] || die "$1 does not contain the toolkit"
    fix_eol "$1"
    if [ -d "$DEST" ]; then
        rm -rf "$OLD"
        mv "$DEST" "$OLD"
    fi
    mv "$1" "$DEST"
    ok "new version in $DEST$( [ -d "$OLD" ] && echo " (previous one in $OLD)" )"
}

run_install() {
    if sh "$DEST/install.sh" "$@"; then
        echo; echo "Update/installation finished."
        return 0
    fi
    echo
    if [ -d "$OLD" ]; then
        echo "  [ERR]  install.sh failed: putting the previous version back"
        rm -rf "$STAGE"; mv "$DEST" "$STAGE"; mv "$OLD" "$DEST"
        sh "$DEST/install.sh" >/dev/null 2>&1 && ok "previous version restored" \
            || echo "  [ERR]  restore failed too: run sh $DEST/install.sh"
        echo "         the failed version is in $STAGE"
    fi
    exit 1
}

download() {
    rm -rf "$STAGE" /tmp/k1lidar_dl; mkdir -p /tmp/k1lidar_dl
    echo "== download $URL =="
    if command -v curl >/dev/null 2>&1; then
        curl -fL --retry 2 -o /tmp/k1lidar_dl/src.tgz "$URL" || die "download failed (curl)"
    else
        wget -O /tmp/k1lidar_dl/src.tgz "$URL" || die "download failed (wget)"
    fi
    mkdir -p /tmp/k1lidar_dl/x
    tar xzf /tmp/k1lidar_dl/src.tgz -C /tmp/k1lidar_dl/x || die "the download is not a valid tar.gz"
    # l'archivio di GitHub ha una cartella in cima (k1-lidar-main/)
    TOP=$(find /tmp/k1lidar_dl/x -maxdepth 2 -name install.sh | head -1)
    [ -n "$TOP" ] || die "install.sh not found in the download"
    mkdir -p "$STAGE"
    cp -a "$(dirname "$TOP")/." "$STAGE/"
    rm -rf /tmp/k1lidar_dl
    ok "downloaded"
}

case "$1" in
    --apply)
        SRC="$2"; [ -n "$SRC" ] || die "--apply DIR"
        shift 2
        check_idle
        echo "== installing from $SRC =="
        if [ "$(cd "$SRC" && pwd)" != "$DEST" ]; then
            swap_in "$SRC"
        fi
        run_install "$@"
        ;;
    --rollback)
        [ -d "$OLD" ] || die "no previous version ($OLD missing)"
        check_idle
        rm -rf "$STAGE"; mv "$DEST" "$STAGE"; mv "$OLD" "$DEST"; mv "$STAGE" "$OLD"
        ok "previous version back in $DEST (the newer one is now in $OLD)"
        run_install
        ;;
    --busy-check)            # usato da install.sh: exit 0 (con messaggio) se occupata
        if MSG=$(printer_busy); then echo "$MSG"; exit 0; fi
        exit 1
        ;;
    -h|--help)
        sed -n '2,21p' "$0"
        ;;
    *)
        check_idle
        if [ -d "$DEST/.git" ] && command -v git >/dev/null 2>&1; then
            echo "== git pull =="
            ( cd "$DEST" && git pull --ff-only ) || die "git pull failed"
            run_install "$@"
        else
            case "$URL" in *"<user>"*) die "set K1LIDAR_URL (download address of the toolkit)" ;; esac
            download
            swap_in "$STAGE"
            run_install "$@"
        fi
        ;;
esac
