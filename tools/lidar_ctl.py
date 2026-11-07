#!/usr/bin/env python3
# lidar_ctl.py - controllo diretto del LiDAR Creality K1 via cx_ai_middleware
# Gira SULLA STAMPANTE (python3). Bypassa la UI: sblocca il gate del middleware
# con `ubus call laser set_state {laser_plugged:1}` e parla il protocollo socket.
#
# Uso:
#   python3 lidar_ctl.py version                 # versione modulo laser
#   python3 lidar_ctl.py scan [file.json] [N]    # N nuvole di punti -> salva JSON
#   python3 lidar_ctl.py exposure                # esposizione corrente
#   python3 lidar_ctl.py raw '{"control":"..."}' # comando grezzo
#   python3 lidar_ctl.py session                 # solo sblocco+chiudi (test code 0)
#
# Protocollo: socket unix /tmp/ai_server_uds, JSON UTF-8 + terminatore 0x03.
# Richiesta  {"control":"<cmd>", <params>}      Risposta {"control":..,"result":{"code":0,..}}
# code 0=OK, 101=driver non init (manca sblocco), 103=mancano dati di scansione.

import socket, json, time, sys, subprocess, os

SOCK = "/tmp/ai_server_uds"
LOG  = "/usr/data/creality/userdata/log/cx_ai_middleware.log"
PORT = "/dev/serial/by-id/creality-laser"
POWER = "/usr/bin/laser_power.sh"

def _ubus(arg):
    subprocess.run(["ubus", "call", "laser", "set_state", arg],
                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)

# Una sola connessione per tutta la sessione: un socket nuovo per comando esaurisce
# gli fd del middleware e lo impicca (successo piu' volte, vedi lidar_pa.py).
_sock = None
_buf = b""

def _connect(timeout):
    global _sock, _buf
    _sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    _sock.settimeout(timeout)
    _sock.connect(SOCK)
    _buf = b""

def ai(obj, timeout=6.0):
    """Invia un comando control e ritorna il dict di risposta."""
    global _sock, _buf
    for tentativo in (0, 1):
        try:
            if _sock is None:
                _connect(timeout)
            _sock.settimeout(timeout)
            _sock.sendall(json.dumps(obj).encode() + b"\x03")
            while b"\x03" not in _buf:
                c = _sock.recv(1 << 20)
                if not c:
                    raise ConnectionError("socket chiuso dal middleware")
                _buf += c
            msg, _, _buf = _buf.partition(b"\x03")
            return json.loads(msg.decode("utf-8", "replace"))
        except socket.timeout:
            return {"control": obj.get("control"), "result": {"code": -1, "err": "timeout"}}
        except (OSError, ConnectionError):
            close_sock()
            if tentativo:
                raise

def close_sock():
    global _sock
    if _sock is not None:
        try: _sock.close()
        except Exception: pass
    _sock = None

def unlock():
    """Sblocca il gate: alimenta il laser (GPIO PA14) se la seriale non c'e',
    poi il middleware inizializza il driver seriale."""
    if not os.path.exists(PORT) and os.path.exists(POWER):
        subprocess.run(["sh", POWER, "on"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        for _ in range(40):
            if os.path.exists(PORT):
                break
            time.sleep(0.25)
    _ubus('{"laser_plugged":1}')
    time.sleep(2)

def lock():
    """Chiusura pulita: spegne emissione, rilascia il driver e l'alimentazione."""
    try: ai({"control": "set_laser_close"})
    except Exception: pass
    close_sock()
    _ubus('{"laser_plugged":0}')
    if os.path.exists(POWER):
        subprocess.run(["sh", POWER, "off"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def open_laser():
    """Stessa sequenza di lidar_pa.py: senza open_flow il modulo risponde
    a get_point_cloud con stato 4 e nessun punto."""
    ai({"control": "set_laser_open"})
    ai({"control": "set_laser_coor", "coor": 1})
    ai({"control": "set_point_cloud_level", "level": 2})
    ai({"control": "set_take_over_model", "takeOverModel": 1})
    ai({"control": "open_flow"})

def close_flow():
    """Va sempre fatto: con take_over lasciato a 1 la calibrazione da display va in errore."""
    for c in ({"control": "close_flow"},
              {"control": "set_take_over_model", "takeOverModel": 0}):
        try: ai(c)
        except Exception: pass

def get_point_cloud():
    return ai({"control": "get_point_cloud"})

# ---------------- CLI ----------------
def main(argv):
    if not argv:
        print(__doc__); return
    cmd = argv[0]
    unlock()
    try:
        if cmd == "version":
            print(json.dumps(ai({"control": "get_laser_version"})))
        elif cmd == "exposure":
            print(json.dumps(ai({"control": "get_laser_exposure"})))
        elif cmd == "session":
            r = ai({"control": "get_laser_version"})
            print("sblocco OK" if r.get("result", {}).get("code") == 0 else "ancora bloccato", r)
        elif cmd == "raw":
            print(json.dumps(ai(json.loads(argv[1]))))
        elif cmd == "scan":
            out = argv[1] if len(argv) > 1 else "pointcloud.json"
            n   = int(argv[2]) if len(argv) > 2 else 1
            frames = []
            try:
                open_laser(); time.sleep(0.5)
                for i in range(n):
                    r = get_point_cloud()
                    pc = r.get("result", {}).get("point_cloud", [])
                    frames.append(pc)
                    print("scan %d/%d: %d punti (code %s)" %
                          (i + 1, n, len(pc), r.get("result", {}).get("code")))
                    time.sleep(0.1)
            finally:
                close_flow()
            with open(out, "w") as f:
                json.dump(frames if n > 1 else frames[0], f)
            print("salvato:", out)
        else:
            print("comando sconosciuto:", cmd); print(__doc__)
    finally:
        lock()

if __name__ == "__main__":
    main(sys.argv[1:])
