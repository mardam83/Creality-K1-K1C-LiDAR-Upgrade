#!/usr/bin/env python3
# laser_driver.py - driver diretto del modulo laser (LiDAR) della Creality K1.
#
# Parla col modulo sulla sua porta seriale (protocollo a frame "AF FF" con checksum a
# 16 bit, documentato osservando il traffico tra stampante e modulo). Solo stdlib
# (termios); pyserial opzionale. Verificato sull'hardware (fw stampante 1.3.5.22,
# modulo 1_0_13): stessi frame del software di serie, nessuna trasformazione dei punti.
#
# Uso (sulla stampante):
#   sh /usr/bin/laser_power.sh on
#   ubus call laser set_state '{"laser_plugged":0}'     # il software di serie rilascia la porta
#   python3 laser_driver.py info
#   python3 laser_driver.py scan [N] [out.temp]           # N frame -> {"scan0": [[x,y,z],...], ...}
#   python3 laser_driver.py get exposure|gain|level|coor
#   python3 laser_driver.py set exposure|gain|level|coor|pcmode|takeover VAL
#   python3 laser_driver.py roi XMIN XMAX
#   python3 laser_driver.py laser on|off   /   light on|off
#   python3 laser_driver.py raw CMD [HEXPAYLOAD]          # es. raw 0x03 (solo comandi di lettura/acquisizione)
# Opzioni: --port PATH (default /dev/serial/by-id/creality-laser), --baud N, -v (dump frame con
#   timestamp monotono)
#
# Su PC (nessun hardware):
#   python laser_driver.py --selftest

import json
import os
import select
import struct
import sys
import threading
import time

DEFAULT_PORT = "/dev/serial/by-id/creality-laser"
DEFAULT_BAUD = 1500000

MAGIC = b"\xAF\xFF"
SRC_HOST = 0x01                  # host -> modulo
SRC_MODULE = 0x02                # modulo -> host (verificato in ricezione)

# --- codici comando ------------------------------------------------------------------
CMD_OPEN_FLOW       = 0x01
CMD_CLOSE_FLOW      = 0x02
CMD_GET_VERSION     = 0x03
CMD_GET_POINT_CLOUD = 0x04
CMD_CALI_STEP       = 0x05
CMD_SET_PC_MODE     = 0x06
CMD_SET_PC_LEVEL    = 0x08
CMD_GET_PC_LEVEL    = 0x09
CMD_SET_ROI         = 0x0A
CMD_SET_TAKE_OVER   = 0x0C
CMD_SET_COOR        = 0x0E
CMD_GET_COOR        = 0x0F
CMD_MODULE_RESTART  = 0x11
CMD_LASER_ONOFF     = 0x13
CMD_FILL_LIGHT      = 0x14
CMD_SET_EXPOSURE    = 0x15
CMD_GET_EXPOSURE    = 0x16
CMD_SET_GAIN        = 0x17
CMD_GET_GAIN        = 0x18
CMD_GET_SN          = 0x1D
CMD_INIT_STATUS     = 0x55
CMD_CREATE_IR_IMAGE = 0x56
CMD_GET_IR_CHUNK    = 0x57

ROI_MAX = 1600                   # validazione host di SET_ROI (0x640)

INIT_ERR_BITS = {0x01: "IR sensor I2C fail", 0x02: "laser starter I2C fail",
                 0x04: "IR sensor mipi fail", 0x08: "Algorithm init fail"}

MAX_PAYLOAD = 0xFFFF - 2         # limite del campo length
MAX_RX_LEN = 0x2800              # length massima accettata in ricezione (buffer Creality 10240 B)
MAX_RX_BUF = 0x10000             # tetto del buffer di ricezione
MAX_POINTS = 1600                # buffer fw point_cloud_get 0x2580 B = 1600 punti x 6 B
POINT_BYTES = 6

# Comandi che possono danneggiare/bloccare il modulo (aggiornamento firmware, reset,
# calibrazione, watchdog): la CLI `raw` li rifiuta sempre, come gli ID sconosciuti.
DANGEROUS_CMDS = {0x05, 0x10, 0x11, 0x12, 0x19, 0x1A, 0x58, 0x59, 0xAA, 0xAB}
SAFE_CMDS = {0x01, 0x02, 0x03, 0x04, 0x06, 0x08, 0x09, 0x0A, 0x0C, 0x0E, 0x0F, 0x13, 0x14,
             0x15, 0x16, 0x17, 0x18, 0x1D, 0x55, 0x56, 0x57}


class LaserError(Exception):
    pass


class FrameError(LaserError):
    pass


class LaserTimeout(LaserError):
    pass


class ModuleStatusError(LaserError):
    def __init__(self, cmd, status):
        LaserError.__init__(self, "cmd 0x%02X: stato modulo %d (0x%02X)" % (cmd, status, status))
        self.cmd, self.status = cmd, status


# --- frame -----------------------------------------------------------------------
def checksum(data):
    """Complemento a due della somma a 16 bit."""
    return (-sum(data)) & 0xFFFF


def encode_frame(cmd, payload=b"", src=SRC_HOST):
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload troppo lungo")
    head = MAGIC + struct.pack("<HBB", len(payload) + 2, src, cmd & 0xFF) + payload
    return head + struct.pack("<H", checksum(head))


def decode_frame(buf, expect_cmd=None, expect_src=SRC_MODULE, strict=False, max_len=MAX_RX_LEN):
    """Cerca e valida un frame in `buf`.

    Ritorna (cmd, payload, end) con `end` = indice dopo il frame, oppure None se non c'e'
    (ancora) nessun frame valido e completo.

    Modalita' normale (strict=False): un candidato con length impossibile, src/cmd diversi o
    checksum errato e' trattato come rumore: si riparte dal byte successivo al suo AF, cosi'
    un falso header non oscura un frame valido che lo segue. Un candidato incompleto non
    blocca la ricerca di candidati successivi.
    strict=True: src/cmd/checksum errati sollevano FrameError (usato dai test).
    """
    i = 0
    while True:
        i = buf.find(MAGIC, i)
        if i < 0:
            return None
        if len(buf) < i + 6:
            return None
        length, src, cmd = struct.unpack_from("<HBB", buf, i + 2)
        if length < 2 or length > max_len:
            i += 1          # length impossibile: AF FF casuale nel rumore
            continue
        if expect_src is not None and src != expect_src:
            if strict:
                raise FrameError("frame source fail (src=0x%02X)" % src)
            i += 1
            continue
        if expect_cmd is not None and cmd != expect_cmd:
            if strict:
                raise FrameError("frame cmd fail (atteso 0x%02X, ricevuto 0x%02X)" % (expect_cmd, cmd))
            i += 1
            continue
        end = i + 4 + length + 2
        if len(buf) < end:
            i += 1          # incompleto: potrebbe essere un falso header, guarda oltre
            continue
        body = buf[i:i + 4 + length]
        (ck,) = struct.unpack_from("<H", buf, i + 4 + length)
        if ck != checksum(body):
            if strict:
                raise FrameError("data check fail (ck=0x%04X atteso 0x%04X)" % (ck, checksum(body)))
            i += 1
            continue
        return cmd, bytes(buf[i + 6:i + 4 + length]), end


# --- decoder dei payload ------------------------------------------------------------
def u24(v):
    if not 0 <= v <= 0xFFFFFF:
        raise ValueError("valore fuori da 24 bit: %r" % v)
    return struct.pack("<I", v)[:3]


def parse_u24(p, off=0):
    if len(p) < off + 3:
        raise LaserError("risposta troppo corta (%d byte)" % len(p))
    return p[off] | p[off + 1] << 8 | p[off + 2] << 16


def parse_point_cloud(payload):
    """payload[0]=stato; poi int16 LE con segno, a triple (x,y,z)."""
    if not payload:
        raise LaserError("frame di profilo vuoto")
    if payload[0] != 0:
        raise ModuleStatusError(CMD_GET_POINT_CLOUD, payload[0])
    data = len(payload) - 1
    if data % POINT_BYTES:
        raise LaserError("point cloud: %d byte di dati non multiplo di %d" % (data, POINT_BYTES))
    n_pts = data // POINT_BYTES
    if n_pts > MAX_POINTS:
        raise LaserError("point cloud: %d punti > massimo %d" % (n_pts, MAX_POINTS))
    vals = struct.unpack_from("<%dh" % (3 * n_pts), payload, 1)
    return [list(vals[3 * k:3 * k + 3]) for k in range(n_pts)]


def u8(v, name="valore"):
    """Rifiuta (non maschera) valori fuori da 0..255."""
    if not isinstance(v, int) or isinstance(v, bool) or not 0 <= v <= 255:
        raise ValueError("%s: serve un intero 0..255 (ricevuto %r)" % (name, v))
    return v


# --- trasporti -----------------------------------------------------------------------
class PosixSerial(object):
    """Seriale raw 8N1, CREAD|CLOCAL, VMIN=1 VTIME=1 (come il software di serie)."""

    def __init__(self, port, baud=DEFAULT_BAUD):
        import termios
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            if not os.isatty(self.fd):
                raise LaserError("%s non e' un terminale" % port)
            speed = getattr(termios, "B%d" % baud, None)
            if speed is None:
                raise LaserError("baud %d non supportato da termios" % baud)
            cc = [0] * len(termios.tcgetattr(self.fd)[6])   # memset 0 come l'originale
            cc[termios.VMIN] = 1
            cc[termios.VTIME] = 1
            attrs = [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0, speed, speed, cc]
            termios.tcflush(self.fd, termios.TCIFLUSH)
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
            import fcntl
            fcntl.fcntl(self.fd, fcntl.F_SETFL, 0)          # torna bloccante, come l'originale
        except Exception:
            os.close(self.fd)
            raise

    def write(self, data):
        n = os.write(self.fd, data)
        if n != len(data):
            raise LaserError("write device error (%d/%d)" % (n, len(data)))

    def read(self, n, timeout):
        r, _, _ = select.select([self.fd], [], [], timeout)
        return os.read(self.fd, n) if r else b""

    def flush_input(self):
        import termios
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class PySerial(object):
    def __init__(self, port, baud=DEFAULT_BAUD):
        import serial  # noqa
        self.s = serial.Serial(port, baud, bytesize=8, parity="N", stopbits=1, timeout=0,
                               rtscts=False, xonxoff=False)

    def write(self, data):
        self.s.write(data)

    def read(self, n, timeout):
        self.s.timeout = timeout
        return self.s.read(max(1, min(n, self.s.in_waiting or 1)))

    def flush_input(self):
        self.s.reset_input_buffer()

    def close(self):
        self.s.close()


def open_transport(port, baud=DEFAULT_BAUD):
    try:
        __import__("termios")
    except ImportError:          # niente termios (es. Windows): serve pyserial
        return PySerial(port, baud)
    return PosixSerial(port, baud)


# --- driver ------------------------------------------------------------------------
class LaserModule(object):
    """API equivalente ai comandi socket del middleware, ma diretta sulla seriale."""

    def __init__(self, transport, timeout=8.0, verbose=False):
        self.t = transport
        self.timeout = timeout           # middleware: 800 x 10 ms
        self.verbose = verbose
        self.flush_before_tx = True      # il middleware non lo fa: disattivabile dopo la validazione
        self._lock = threading.Lock()    # una sola transazione alla volta sulla seriale

    # -- trasporto
    def transact(self, cmd, payload=b"", timeout=None):
        """Invia un comando e restituisce il payload della risposta (stesso cmd, src=0x02)."""
        with self._lock:
            return self._transact(cmd, payload, timeout)

    def _transact(self, cmd, payload, timeout):
        frame = encode_frame(cmd, payload)
        if self.flush_before_tx:
            try:
                self.t.flush_input()
            except Exception:
                pass
        if self.verbose:
            sys.stderr.write("%.3f TX %s\n" % (time.monotonic(), frame.hex()))
        self.t.write(frame)
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        buf = b""
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                raise LaserTimeout("recv timeout cmd 0x%02X (ricevuti %d byte)" % (cmd, len(buf)))
            chunk = self.t.read(4096, min(left, 0.05))
            if chunk:
                buf += chunk
                if len(buf) > MAX_RX_BUF:
                    buf = buf[-MAX_RX_BUF:]      # memoria limitata: tieni la coda
                res = decode_frame(buf, expect_cmd=cmd)
                if res is not None:
                    if self.verbose:
                        sys.stderr.write("%.3f RX %s\n" % (time.monotonic(), buf[:res[2]].hex()))
                    return res[1]

    def _status(self, cmd, payload=b""):
        p = self.transact(cmd, payload)
        if not p:
            raise LaserError("cmd 0x%02X: risposta vuota" % cmd)
        if p[0] != 0:
            raise ModuleStatusError(cmd, p[0])
        return p

    # -- init (stessa sequenza del software di serie)
    def init(self, set_coor=True):
        p = self.transact(CMD_INIT_STATUS)
        if not p or p[0] != 0:
            bits = p[0] if p else 0xFF
            errs = [t for b, t in INIT_ERR_BITS.items() if bits & b]
            raise LaserError("module init fail 0x%02X %s" % (bits, errs))
        try:
            info = {"version": self.version()}
        except LaserTimeout:
            raise LaserError("il modulo risponde all'init (0x55) ma non alla lettura della versione "
                             "(0x03): firmware del modulo diverso da quello provato (1_0_13)?")
        try:
            info["sn"] = self.serial_number()
        except LaserError as e:
            info["sn"] = None
            info["sn_err"] = str(e)
        if set_coor:
            self.set_coor(1)         # l'init del middleware lo fa sempre
        return info

    def version(self):
        p = self.transact(CMD_GET_VERSION)
        if len(p) < 3:
            raise LaserError("versione: risposta corta %r" % p)
        return "%d_%d_%d" % (p[0], p[1], p[2])

    def serial_number(self):
        p = self.transact(CMD_GET_SN)
        if len(p) < 13 or p[0] != 0:
            raise LaserError("SN non disponibile (%r)" % p)
        return p[1:13].hex().upper()   # [status][12 byte SN]

    def _value(self, cmd, n=1):
        """Getter con valore diretto (nessun byte di stato): richiede almeno n byte."""
        p = self.transact(cmd)
        if len(p) < n:
            raise LaserError("cmd 0x%02X: risposta troppo corta (%d byte)" % (cmd, len(p)))
        return p

    # -- acquisizione
    def open_flow(self):
        self._status(CMD_OPEN_FLOW)

    def close_flow(self):
        self._status(CMD_CLOSE_FLOW)

    def get_point_cloud(self):
        return parse_point_cloud(self.transact(CMD_GET_POINT_CLOUD))

    def set_level(self, level):
        self._status(CMD_SET_PC_LEVEL, bytes([u8(level, "level")]))

    def get_level(self):
        return self._value(CMD_GET_PC_LEVEL)[0]

    def set_coor(self, coor):
        self._status(CMD_SET_COOR, bytes([u8(coor, "coor")]))

    def get_coor(self):
        return self._value(CMD_GET_COOR)[0]

    def set_take_over(self, mode):
        self._status(CMD_SET_TAKE_OVER, bytes([u8(mode, "takeover")]))

    def set_pc_mode(self, mode):
        self._status(CMD_SET_PC_MODE, bytes([u8(mode, "pcmode")]))

    def set_roi(self, x_min, x_max):
        if not (0 <= x_min <= x_max <= ROI_MAX):
            raise ValueError("ROI: serve 0 <= x_min <= x_max <= %d" % ROI_MAX)
        self._status(CMD_SET_ROI, struct.pack("<HH", x_min, x_max))

    # -- parametri sensore
    def set_exposure(self, v):
        self._status(CMD_SET_EXPOSURE, u24(v))

    def get_exposure(self):
        return parse_u24(self.transact(CMD_GET_EXPOSURE))

    def set_gain(self, v):
        self._status(CMD_SET_GAIN, u24(v))

    def get_gain(self):
        return parse_u24(self.transact(CMD_GET_GAIN))

    # -- attuatori / servizio
    def laser(self, on):
        self._status(CMD_LASER_ONOFF, b"\x01" if on else b"\x00")

    def fill_light(self, on):
        self._status(CMD_FILL_LIGHT, b"\x01" if on else b"\x00")

    def cali_step(self, step):
        if step not in (1, 2):
            raise ValueError("step deve essere 1 o 2")
        self._status(CMD_CALI_STEP, bytes([step]))

    def module_restart(self):
        self._status(CMD_MODULE_RESTART)

    def ir_image(self, img_type, resolution, path):
        p = self._status(CMD_CREATE_IR_IMAGE, bytes([u8(img_type, "tipo"), u8(resolution, "risoluzione")]))
        size = parse_u24(p, 1)
        got = 0
        ok = False
        try:
            with open(path, "wb") as f:
                while got < size:
                    q = self._status(CMD_GET_IR_CHUNK, b"\x00")
                    if len(q) <= 1:
                        raise LaserError("IR: chunk vuoto a %d/%d" % (got, size))
                    chunk = q[1:size - got + 1]          # tronca l'ultimo blocco alla dimensione residua
                    f.write(chunk)
                    got += len(chunk)
            ok = got == size
        finally:
            try:                                          # chiusura trasferimento (0x57, modo 2)
                self.transact(CMD_GET_IR_CHUNK, b"\x02")
            except LaserError:
                pass
            if not ok:
                try:
                    os.remove(path)
                except OSError:
                    pass
        return size

    def close(self):
        self.t.close()


# --- selftest (senza hardware) -----------------------------------------------------------
class FakeModule(object):
    """Simula il modulo sul trasporto: risponde secondo la specifica, con rumore e frammentazione."""

    def __init__(self):
        self.rx = b""
        self.out = b""
        self.state = {"exposure": 0x01F4, "gain": 0x10, "level": 0, "coor": 0, "flow": False}
        self.pc = [[-5000, 0, -12], [0, 0, 3], [7321, 0, -80]]
        self.ir = bytes(range(256)) * 9   # 2304 byte

    def _reply(self, cmd, payload):
        self.out += b"\x00\x13\xAF"          # rumore prima dell'header (anche un AF isolato)
        self.out += encode_frame(cmd, payload, src=SRC_MODULE)

    def write(self, data):
        self.rx += data
        r = decode_frame(self.rx, expect_src=SRC_HOST)
        assert r is not None, "frame host incompleto"
        cmd, p, end = r
        self.rx = self.rx[end:]
        s = self.state
        if cmd == CMD_INIT_STATUS:
            self._reply(cmd, b"\x00")
        elif cmd == CMD_GET_VERSION:
            self._reply(cmd, b"\x01\x00\x0d")
        elif cmd == CMD_GET_SN:
            self._reply(cmd, bytes([0, 0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 1, 2, 3, 4, 5, 6, 7]))   # [status][12 B SN]
        elif cmd in (CMD_SET_COOR, CMD_SET_PC_LEVEL):
            s["coor" if cmd == CMD_SET_COOR else "level"] = p[0]
            self._reply(cmd, b"\x00")
        elif cmd == CMD_GET_COOR:
            self._reply(cmd, bytes([s["coor"]]))
        elif cmd == CMD_GET_PC_LEVEL:
            self._reply(cmd, bytes([s["level"]]))
        elif cmd == CMD_SET_EXPOSURE:
            s["exposure"] = parse_u24(p)
            self._reply(cmd, b"\x00")
        elif cmd == CMD_GET_EXPOSURE:
            self._reply(cmd, u24(s["exposure"]))
        elif cmd == CMD_SET_GAIN:
            s["gain"] = parse_u24(p)
            self._reply(cmd, b"\x00")
        elif cmd == CMD_GET_GAIN:
            self._reply(cmd, u24(s["gain"]))
        elif cmd == CMD_OPEN_FLOW:
            s["flow"] = True
            self._reply(cmd, b"\x00")
        elif cmd == CMD_CLOSE_FLOW:
            s["flow"] = False
            self._reply(cmd, b"\x00")
        elif cmd == CMD_GET_POINT_CLOUD:
            if not s["flow"]:
                self._reply(cmd, b"\x05")
            else:
                body = b"".join(struct.pack("<3h", *pt) for pt in self.pc)
                self._reply(cmd, b"\x00" + body)
        elif cmd == CMD_SET_ROI:
            self._reply(cmd, b"\x00" if struct.unpack("<HH", p)[1] <= ROI_MAX else b"\x09")
        elif cmd == CMD_CREATE_IR_IMAGE:
            self._ir_pos = 0
            self._reply(cmd, b"\x00" + u24(len(self.ir)))
        elif cmd == CMD_GET_IR_CHUNK and p[:1] == b"\x02":
            self.ir_closed = True
            self._reply(cmd, b"\x00")
        elif cmd == CMD_GET_IR_CHUNK:
            chunk = self.ir[self._ir_pos:self._ir_pos + 0x7FF]
            self._ir_pos += len(chunk)
            self._reply(cmd, b"\x00" + chunk)
        else:
            self._reply(cmd, b"\x00")

    def read(self, n, timeout):
        k = min(n, 5)                     # frammenta la risposta
        d, self.out = self.out[:k], self.out[k:]
        return d

    def flush_input(self):
        pass

    def close(self):
        pass


def selftest():
    ok = [0]

    def check(cond, what):
        if not cond:
            raise AssertionError("FALLITO: " + what)
        ok[0] += 1
        print("  ok  " + what)

    print("[1] frame e checksum")
    f = encode_frame(CMD_GET_VERSION)
    check(f == bytes.fromhex("afff02000103 4cfe".replace(" ", "")), "GET_VERSION = AF FF 02 00 01 03 4C FE")
    check(sum(f[:-2]) + struct.unpack("<H", f[-2:])[0] & 0xFFFF == 0, "somma frame + checksum = 0 mod 2^16")
    f = encode_frame(CMD_SET_EXPOSURE, u24(0x012345))
    check(f[:6] == b"\xAF\xFF\x05\x00\x01\x15" and f[6:9] == b"\x45\x23\x01", "SET_EXPOSURE: length=N+2 LE, u24 LE")
    f = encode_frame(CMD_SET_ROI, struct.pack("<HH", 100, 1500))
    check(f[6:10] == b"\x64\x00\xdc\x05", "SET_ROI: x_min/x_max u16 LE")
    big = bytes(range(256)) * 20
    f = encode_frame(CMD_GET_POINT_CLOUD, big, src=SRC_MODULE)
    r = decode_frame(b"rumore" + f + b"coda", expect_cmd=CMD_GET_POINT_CLOUD)
    check(r is not None and r[1] == big and r[2] == 6 + len(f), "round-trip 5120 byte con rumore prima/dopo")
    check(decode_frame(f[:-1]) is None, "frame troncato -> None (attendi altri byte)")
    bad = bytearray(f)
    bad[100] ^= 1
    try:
        decode_frame(bytes(bad), strict=True)
        check(False, "checksum errato rilevato (strict)")
    except FrameError:
        check(True, "checksum errato rilevato (strict)")
    try:
        decode_frame(encode_frame(3, b"\x01\x00\x0d", src=SRC_HOST), strict=True)
        check(False, "src host rifiutato in ricezione (strict)")
    except FrameError:
        check(True, "src host rifiutato in ricezione (strict)")
    try:
        decode_frame(encode_frame(4, b"\x00", src=SRC_MODULE), expect_cmd=3, strict=True)
        check(False, "cmd diverso rifiutato (strict)")
    except FrameError:
        check(True, "cmd diverso rifiutato (strict)")
    good = encode_frame(CMD_GET_POINT_CLOUD, b"\x00" + bytes(range(18)), src=SRC_MODULE)
    check(decode_frame(bad[:6] + b"\x00" + good, expect_cmd=CMD_GET_POINT_CLOUD)[0] == CMD_GET_POINT_CLOUD,
          "resync: frame con checksum errato seguito da frame valido")
    fake = b"\xAF\xFF\x40\x00\x02\x04"           # falso header: length 64, incompleto
    r = decode_frame(fake + good, expect_cmd=CMD_GET_POINT_CLOUD)
    check(r is not None and r[1] == b"\x00" + bytes(range(18)), "resync: falso header lungo non oscura il frame valido")
    r = decode_frame(b"\xAF\xFF\xFF\xFF\x02\x04" + good, expect_cmd=CMD_GET_POINT_CLOUD)
    check(r is not None, "resync: length enorme scartata subito")
    check(decode_frame(good + b"\xAF", expect_cmd=CMD_GET_POINT_CLOUD)[2] == len(good), "AF finale non disturba")

    print("[2] payload")
    pl = b"\x00" + struct.pack("<6h", -1, 2, -32768, 32767, 0, -60)
    check(parse_point_cloud(pl) == [[-1, 2, -32768], [32767, 0, -60]], "point cloud: int16 LE con segno, triple")
    for bad_pl, what in ((pl + b"\x01", "byte dispari"), (pl + b"\x01\x02", "mezzo punto"), (b"", "vuoto"),
                         (b"\x00" + bytes(6 * (MAX_POINTS + 1)), "troppi punti")):
        try:
            parse_point_cloud(bad_pl)
            check(False, "point cloud malformata rifiutata: " + what)
        except ModuleStatusError:
            check(False, "point cloud malformata rifiutata: " + what)
        except LaserError:
            check(True, "point cloud malformata rifiutata: " + what)
    check(len(parse_point_cloud(b"\x00")) == 0, "point cloud con zero punti ammessa")
    try:
        u8(256)
        check(False, "u8 rifiuta 256")
    except ValueError:
        check(True, "u8 rifiuta 256")
    try:
        parse_point_cloud(b"\x05")
        check(False, "stato point cloud != 0 -> errore")
    except ModuleStatusError as e:
        check(e.status == 5, "stato point cloud != 0 -> errore")
    check(parse_u24(b"\x45\x23\x01") == 0x012345, "u24 LE")

    print("[3] driver su modulo simulato")
    fm = FakeModule()
    m = LaserModule(fm, timeout=1.0)
    info = m.init()
    check(info["version"] == "1_0_13", "init: 0x55 -> versione 1_0_13")
    check(m.get_coor() == 1, "init imposta coor=1 (come LaserDriveInit)")
    check(info["sn"] == "A1B2C3D4E501020304050607", "SN = 12 byte dopo lo status")
    m.set_exposure(1234)
    check(m.get_exposure() == 1234, "set/get esposizione u24")
    m.set_gain(77)
    check(m.get_gain() == 77, "set/get guadagno u24")
    m.set_level(2)
    check(m.get_level() == 2, "set/get point cloud level")
    m.set_roi(0, 1600)
    try:
        m.set_roi(10, 1601)
        check(False, "ROI > 1600 rifiutata lato host")
    except ValueError:
        check(True, "ROI > 1600 rifiutata lato host")
    try:
        m.get_point_cloud()
        check(False, "get_point_cloud a flow chiuso -> stato modulo")
    except ModuleStatusError:
        check(True, "get_point_cloud a flow chiuso -> stato modulo")
    m.open_flow()
    check(m.get_point_cloud() == [[-5000, 0, -12], [0, 0, 3], [7321, 0, -80]], "get_point_cloud con risposta frammentata")
    m.close_flow()
    import tempfile
    tmp = os.path.join(tempfile.gettempdir(), "laser_driver_selftest_ir.bin")
    n = m.ir_image(1, 1, tmp)
    with open(tmp, "rb") as fh:
        check(n == 2304 and fh.read() == bytes(range(256)) * 9, "immagine IR a blocchi 0x57, ultimo blocco troncato")
    check(getattr(fm, "ir_closed", False), "trasferimento IR chiuso con 0x57 modo 2")
    os.remove(tmp)

    class Silent(FakeModule):
        def read(self, n, timeout):
            time.sleep(min(timeout, 0.01))
            return b""
    try:
        LaserModule(Silent(), timeout=0.1).version()
        check(False, "timeout senza risposta")
    except LaserTimeout:
        check(True, "timeout senza risposta")

    print("SELFTEST OK: %d verifiche (senza hardware)" % ok[0])


# --- CLI ---------------------------------------------------------------------------
def main(argv):
    port, baud, verbose = DEFAULT_PORT, DEFAULT_BAUD, False
    args = []
    it = iter(argv)
    for a in it:
        if a == "--port":
            port = next(it)
        elif a == "--baud":
            baud = int(next(it))
        elif a == "-v":
            verbose = True
        elif a == "--selftest":
            selftest()
            return 0
        else:
            args.append(a)
    if not args:
        with open(os.path.abspath(__file__)) as fh:     # l'help e' il commento in testa al file
            print(fh.read().split("\nimport", 1)[0])
        return 1
    cmd = args[0]
    if cmd == "raw":                      # controllo PRIMA di aprire la seriale
        rcmd = int(args[1], 0)
        if rcmd not in SAFE_CMDS:
            kind = "PERICOLOSO" if rcmd in DANGEROUS_CMDS else "SCONOSCIUTO"
            print("raw 0x%02X: comando %s, rifiutato (solo comandi di lettura/acquisizione)."
                  % (rcmd, kind))
            return 2
    m = LaserModule(open_transport(port, baud), verbose=verbose)
    try:
        if cmd == "raw":
            payload = bytes.fromhex(args[2]) if len(args) > 2 else b""
            print(m.transact(rcmd, payload).hex())
            return 0
        info = m.init()
        if cmd == "info":
            info.update(exposure=m.get_exposure(), gain=m.get_gain(), level=m.get_level(), coor=m.get_coor())
            print(json.dumps(info, indent=1))
        elif cmd == "scan":
            n = int(args[1]) if len(args) > 1 else 1
            out = args[2] if len(args) > 2 else "scan.temp"
            m.set_level(2)
            m.set_take_over(1)
            m.open_flow()
            frames = {}
            try:
                for i in range(n):
                    t0 = time.time()
                    try:
                        pc = m.get_point_cloud()
                    except ModuleStatusError as e:
                        print("frame %d: %s" % (i, e))
                        pc = []
                    frames["scan%d" % i] = pc
                    print("frame %d: %d punti (%.0f ms)" % (i, len(pc), (time.time() - t0) * 1000))
            finally:
                m.close_flow()
            with open(out, "w") as f:
                json.dump(frames, f)
            print("salvato", out)
        elif cmd == "get":
            print({"exposure": m.get_exposure, "gain": m.get_gain, "level": m.get_level,
                   "coor": m.get_coor}[args[1]]())
        elif cmd == "set":
            {"exposure": m.set_exposure, "gain": m.set_gain, "level": m.set_level, "coor": m.set_coor,
             "pcmode": m.set_pc_mode, "takeover": m.set_take_over}[args[1]](int(args[2], 0))
            print("ok")
        elif cmd == "roi":
            m.set_roi(int(args[1]), int(args[2]))
            print("ok")
        elif cmd in ("laser", "light"):
            (m.laser if cmd == "laser" else m.fill_light)(args[1] == "on")
            print("ok")
        else:
            print("comando sconosciuto:", cmd)
            return 1
    finally:
        m.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
