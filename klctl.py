#!/usr/bin/env python3
# klctl.py - mini client del socket API Klipper (/tmp/klippy_uds).
#   python3 klctl.py info                 -> stato ("ready" / "error") e messaggio
#   python3 klctl.py script "G28"         -> invia gcode e attende
#   python3 klctl.py restart              -> FIRMWARE_RESTART
import socket, json, sys, time

SOCK = "/tmp/klippy_uds"

def call(obj, timeout=120):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    s.connect(SOCK)
    s.sendall(json.dumps(obj).encode() + b"\x03")
    buf = b""
    try:
        while b"\x03" not in buf:
            c = s.recv(1 << 16)
            if not c:
                break
            buf += c
    except Exception:
        pass
    s.close()
    for fr in buf.split(b"\x03"):
        if not fr.strip():
            continue
        try:
            m = json.loads(fr)
        except Exception:
            continue
        if m.get("id") == obj.get("id"):
            return m
    return None

def info():
    r = call({"id": 1, "method": "info"})
    if not r:
        return "no-response", ""
    res = r.get("result", {})
    return res.get("state", "?"), res.get("state_message", "")

def main():
    if len(sys.argv) < 2:
        print("uso: info | script <gcode> | restart"); return
    cmd = sys.argv[1]
    if cmd == "info":
        st, msg = info(); print("state:", st); print("msg:", msg.strip()[:400])
    elif cmd == "script":
        r = call({"id": 2, "method": "gcode/script", "params": {"script": sys.argv[2]}})
        print(json.dumps(r))
    elif cmd == "restart":
        try:
            call({"id": 3, "method": "gcode/script", "params": {"script": "FIRMWARE_RESTART"}}, timeout=5)
        except Exception as e:
            print("restart inviato (connessione caduta, normale):", e)
        # attesa ripartenza + polling stato
        for i in range(30):
            time.sleep(2)
            try:
                st, msg = info()
                print("  [%2ds] state=%s" % (i * 2, st))
                if st in ("ready", "error", "shutdown"):
                    if msg.strip():
                        print("  msg:", msg.strip()[:400])
                    return
            except Exception:
                print("  [%2ds] socket non pronto..." % (i * 2))
    else:
        print("comando sconosciuto")

if __name__ == "__main__":
    main()
