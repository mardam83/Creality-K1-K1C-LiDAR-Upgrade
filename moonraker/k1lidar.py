# k1lidar.py - componente Moonraker del toolkit LiDAR per Creality K1.
#
# Collega Klipper (pulsanti/macro e hook di avvio stampa) agli script che usano il
# laser. Le macro chiamano questo componente con `action_call_remote_method`, quindi
# non serve gcode_shell_command.
#
# moonraker.conf:
#   [k1lidar]
#   path: /usr/data/k1-lidar          # cartella del toolkit (default: quella di questo file)
#
# Remote method (dalle macro in config/k1lidar.cfg):
#   k1lidar_print(filename)                     avvio stampa con flusso/primo strato
#   k1lidar_pa(nozzle, bed, filament, material, fine, start, step, gcode)   taratura PA a pulsante
#   k1lidar_set(pa, gcode, filament, nozzle)    PA di un profilo impostato a mano
#   k1lidar_cancel()  k1lidar_list()  k1lidar_forget(filament, nozzle)
#   k1lidar_answer(choice)                      risposta alla domanda di avvio stampa
#
# Domanda prima della stampa (registro lidar_ask=1): k1lidar_print apre una finestra in
# Fluidd/Mainsail (righe "// action:prompt_..." in console, via _LIDAR_SAY) e aspetta
# la risposta per lidar_ask_timeout secondi; senza risposta usa i registri salvati.
# HTTP (gli stessi nomi valgono come JSON-RPC sul websocket):
#   GET  /machine/k1lidar/status      registri, job in corso, ultimi risultati, log
#   GET  /machine/k1lidar/filaments   calibrazioni salvate per filamento
#   POST /machine/k1lidar/settings    {flow, firstlayer, on_bad, pa_apply, ask, ask_timeout,
#                                      lang, material, hotend, bed}
#   POST /machine/k1lidar/pa          {nozzle, bed, filament, material, fine, start, step, gcode}
#   POST /machine/k1lidar/set         {pa, gcode | filament + nozzle}
#   POST /machine/k1lidar/print       {filename}  (applica i registri)
#   POST /machine/k1lidar/cancel
#
# Lo STATO (le scelte dell'utente) sta nei registri [save_variables] di Klipper:
# l'hook di stampa li legge in jinja e le macro li scrivono. Qui si leggono e si
# scrivono solo tramite Klipper (SAVE_VARIABLE), cosi' la verita' e' una sola.
from __future__ import annotations

import asyncio
import json
import logging
import os
import re
import signal
from typing import Any, Dict, Optional

COMPONENT = "k1lidar"
MATERIALS = ("PLA", "PETG", "ABS", "TPU", "ASA")
MIN_HOTEND, MAX_HOTEND = 150, 320
MIN_BED, MAX_BED = 40, 115
CALIB = "/usr/data/lidar_calib.json"
GCODES = "/usr/data/printer_data/gcodes"
# risposte della finestra di avvio stampa
ASK_CHOICES = ("both", "pa", "fl", "none", "cancel", "default")
DEFAULTS = {"lidar_flow": 0, "lidar_firstlayer": 0, "lidar_on_bad": "pause",
            "lidar_pa_apply": 1, "lidar_lang": "en", "lidar_ask": 0, "lidar_ask_timeout": 60,
            "lidar_material": "PLA", "lidar_hotend": 0, "lidar_bed": 0}


class K1Lidar:
    def __init__(self, config) -> None:
        self.server = config.get_server()
        self.eventloop = self.server.get_event_loop()
        here = os.path.dirname(os.path.realpath(__file__))
        self.path = config.get("path", os.path.dirname(here))
        self.proc: Optional[asyncio.subprocess.Process] = None
        self.task: Optional[str] = None
        self.last: Dict[str, Any] = {}
        self.log_lines: list = []
        self.ask_future: Optional[asyncio.Future] = None

        for path, methods, cb in (
                ("/machine/k1lidar/status", ["GET"], self._h_status),
                ("/machine/k1lidar/filaments", ["GET"], self._h_filaments),
                ("/machine/k1lidar/settings", ["POST"], self._h_settings),
                ("/machine/k1lidar/pa", ["POST"], self._h_pa),
                ("/machine/k1lidar/set", ["POST"], self._h_set),
                ("/machine/k1lidar/print", ["POST"], self._h_print),
                ("/machine/k1lidar/cancel", ["POST"], self._h_cancel)):
            self.server.register_endpoint(path, methods, cb)
        for name, cb in (("k1lidar_print", self._rm_print), ("k1lidar_pa", self._rm_pa),
                         ("k1lidar_set", self._rm_set),
                         ("k1lidar_cancel", self._rm_cancel), ("k1lidar_list", self._rm_list),
                         ("k1lidar_forget", self._rm_forget), ("k1lidar_answer", self._rm_answer)):
            self.server.register_remote_method(name, cb)

    # ------------------------------------------------------------- registri
    async def _regs(self) -> Dict[str, Any]:
        regs = dict(DEFAULTS)
        try:
            res = await self._kapis().query_objects({"save_variables": None})
            saved = res.get("save_variables", {}).get("variables", {}) or {}
            regs.update({k: saved[k] for k in regs if k in saved})
        except Exception:
            logging.exception("[%s] lettura registri fallita, uso i default", COMPONENT)
        return regs

    async def _write_reg(self, key: str, value: Any) -> None:
        # SAVE_VARIABLE: klipper spacchetta i parametri con shlex e poi fa literal_eval,
        # quindi una stringa vuole il DOPPIO quoting: VALUE='"PLA"'.
        if isinstance(value, bool):
            lit = "1" if value else "0"
        elif isinstance(value, str):
            lit = "'\"%s\"'" % value
        else:
            lit = str(value)
        await self._kapis().run_gcode("SAVE_VARIABLE VARIABLE=%s VALUE=%s" % (key, lit))

    def _kapis(self):
        return self.server.lookup_component("klippy_apis")

    async def _say(self, msg: str) -> None:
        """Messaggio in console Klipper (Fluidd/Mainsail/display)."""
        try:
            for line in msg.splitlines():
                if line.strip():
                    # M118 non esiste sulla K1: macro _LIDAR_SAY (action_respond_info)
                    await self._kapis().run_gcode('_LIDAR_SAY MSG="%s"' % line.replace('"', "'")[:200])
        except Exception:
            logging.exception("[%s] messaggio in console fallito", COMPONENT)

    async def _print_state(self) -> str:
        try:
            r = await self._kapis().query_objects({"print_stats": None})
            return r.get("print_stats", {}).get("state", "unknown")
        except Exception:
            return "unknown"

    # ------------------------------------------------------------------ job
    def _busy(self) -> bool:
        return self.proc is not None and self.proc.returncode is None

    async def _run_aux(self, args: list, lang: str) -> int:
        """Comandi rapidi (elenco, PA a mano): processo a parte, non occupa il job
        principale (cosi' LIDAR_PA_LIST funziona anche durante una taratura). I
        messaggi li scrive lo script direttamente in console."""
        script = os.path.join(self.path, "lidar_job.py")
        env = dict(os.environ, K1LIDAR_LANG=str(lang or "en"), PYTHONUNBUFFERED="1")
        p = await asyncio.create_subprocess_exec(
            "python3", "-u", script, *args, cwd=self.path, env=env,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
        out, _ = await p.communicate()
        if p.returncode:
            logging.info("[%s] %s rc=%s: %s", COMPONENT, args[0], p.returncode,
                         out.decode(errors="replace")[-500:])
            tail = [l for l in out.decode(errors="replace").splitlines() if l.strip()][-1:]
            await self._say("LiDAR: %s" % (tail[0] if tail else "errore"))
        return p.returncode

    async def _run(self, task: str, args: list, lang: str) -> None:
        if self._busy():
            raise self.server.error("LiDAR: job gia' in corso (%s)" % self.task, 409)
        script = os.path.join(self.path, "lidar_job.py")
        if not os.path.exists(script):
            raise self.server.error("script non trovato: %s" % script, 404)
        env = dict(os.environ, K1LIDAR_LANG=str(lang or "en"), PYTHONUNBUFFERED="1")
        cmd = ["python3", "-u", script] + args
        logging.info("[%s] avvio %s: %s", COMPONENT, task, " ".join(cmd))
        self.task = task
        self.log_lines = []
        self.proc = await asyncio.create_subprocess_exec(
            *cmd, cwd=self.path, env=env,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
        self.server.send_event("%s:started" % COMPONENT, {"task": task})
        self.eventloop.create_task(self._reader(task))

    async def _reader(self, task: str) -> None:
        assert self.proc is not None and self.proc.stdout is not None
        try:
            async for raw in self.proc.stdout:
                line = raw.decode(errors="replace").rstrip()
                if not line:
                    continue
                self.log_lines.append(line)
                del self.log_lines[:-500]
                self.server.send_event("%s:output" % COMPONENT, {"task": task, "line": line})
                self._scrape(line)
        except Exception:
            logging.exception("[%s] lettura output fallita", COMPONENT)
        rc = await self.proc.wait()
        self.last["returncode"] = rc
        self.server.send_event("%s:finished" % COMPONENT, {"task": task, "returncode": rc,
                                                           "result": self.last})
        logging.info("[%s] %s terminato rc=%s", COMPONENT, task, rc)
        self.proc = None
        self.task = None

    def _scrape(self, line: str) -> None:
        m = re.search(r"PA OTTIMALE \(LiDAR\):\s*([0-9.]+)", line)
        if m:
            self.last["pa"] = float(m.group(1))
        m = re.search(r"(?:LiDAR PA result|Risultato PA LiDAR): ([0-9.]+) \((?:80% range|"
                      r"intervallo 80%) ([0-9.]+)-([0-9.]+)\), (?:confidence|confidenza) (\S+)", line)
        if m:
            self.last["pa_range"] = [float(m.group(2)), float(m.group(3))]
            self.last["pa_confidence"] = m.group(4).lower()
        m = re.search(r">>> PRIMO STRATO:\s*(GOOD|BAD|UNCERTAIN|n/d)", line)
        if m:
            self.last["firstlayer"] = {"verdict": m.group(1), "problems": []}
        m = re.match(r"\s*[!-]\s+(.*(?:MANCANTE|SOLLEVAMENTO|INATTESO|troppo|copertura|"
                     r"indizio|registrazione|scan).*)$", line)
        if m and isinstance(self.last.get("firstlayer"), dict):
            p = self.last["firstlayer"]["problems"]
            if m.group(1) not in p:
                p.append(m.group(1).strip())
        if ">>> JOB" in line:
            self.last["job"] = line.split(">>>", 1)[1].strip()

    # ------------------------------------------------------------ validazione
    def _temp(self, v, lo, hi, what) -> Optional[int]:
        if v in (None, "", 0, "0"):
            return None
        try:
            n = int(float(v))
        except (TypeError, ValueError):
            raise self.server.error("%s non valido: %r" % (what, v), 400)
        if not lo <= n <= hi:
            raise self.server.error("%s %d fuori dai limiti (%d-%d)" % (what, n, lo, hi), 400)
        return n

    def _gcode(self, v) -> Optional[str]:
        """Nome di un file nella cartella gcodes (come lo mostra Fluidd)."""
        v = (v or "").strip().lstrip("/")
        if not v:
            return None
        if ".." in v.split("/") or any(c in v for c in "\"`$;|&<>\n"):
            raise self.server.error("nome gcode non valido: %r" % v, 400)
        path = os.path.join(GCODES, v)
        if not os.path.isfile(path):
            raise self.server.error("gcode non trovato: %s" % v, 404)
        return v

    def _filament(self, v) -> Optional[str]:
        v = (v or "").strip()
        if len(v) > 80 or any(c in v for c in "\"'`$;|&<>\n"):
            raise self.server.error("nome filamento non valido: %r" % v, 400)
        return v or None

    # ------------------------------------------------------------ operazioni
    def _range(self, start, step):
        """START/STEP dei 32 valori del pattern (None = default del materiale)."""
        out = []
        try:
            st = float(start) if start not in (None, "") else -1.0
            sp = float(step) if step not in (None, "") else 0.0
        except (TypeError, ValueError):
            raise self.server.error("start/step non validi", 400)
        if st != -1.0:
            if not 0.0 <= st <= 0.5:
                raise self.server.error("start fuori dai limiti (0-0.5)", 400)
            out += ["--start", "%.4f" % st]
        if sp:
            if not 0.0005 <= sp <= 0.02:
                raise self.server.error("step fuori dai limiti (0.0005-0.02)", 400)
            out += ["--step", "%.4f" % sp]
        return out

    async def start_pa(self, nozzle=None, bed=None, filament=None, material=None, fine=False,
                       start=None, step=None, gcode=None):
        regs = await self._regs()
        gfile = self._gcode(gcode)
        # con un gcode materiale e temperature vengono dal gcode, salvo override espliciti
        mat = (material or ("" if gfile else regs["lidar_material"]) or "").upper()
        if mat and mat not in MATERIALS:
            raise self.server.error("materiale '%s' sconosciuto (%s)" % (mat, ", ".join(MATERIALS)), 400)
        if await self._print_state() in ("printing", "paused"):
            raise self.server.error("stampa in corso: impossibile tarare ora", 409)
        args = ["pa"]
        if mat:
            args += ["--material", mat]
        if gfile:
            args += ["--gcode", gfile]
        n = self._temp(nozzle, MIN_HOTEND, MAX_HOTEND, "ugello") or (None if gfile else self._temp(
            regs["lidar_hotend"], MIN_HOTEND, MAX_HOTEND, "ugello"))
        b = self._temp(bed, MIN_BED, MAX_BED, "piatto") or (None if gfile else self._temp(
            regs["lidar_bed"], MIN_BED, MAX_BED, "piatto"))
        if n:
            args += ["--nozzle", str(n)]
        if b:
            args += ["--bed", str(b)]
        fil = self._filament(filament)
        if fil:
            args += ["--filament", fil]
        if fine:
            args.append("--fine")
        args += self._range(start, step)
        self.last = {}
        await self._run("pa", args, regs["lidar_lang"])
        return {"started": True, "task": "pa", "args": args}

    async def set_pa(self, pa=None, gcode=None, filament=None, nozzle=None):
        try:
            v = float(pa)
        except (TypeError, ValueError):
            raise self.server.error("PA non valido: %r" % pa, 400)
        if not 0.0 <= v <= 1.0:
            raise self.server.error("PA fuori dai limiti (0-1)", 400)
        regs = await self._regs()
        args = ["set", "--pa", "%.4f" % v]
        gfile = self._gcode(gcode)
        if gfile:
            args += ["--gcode", gfile]
        fil = self._filament(filament)
        if fil:
            args += ["--filament", fil]
        n = self._temp(nozzle, MIN_HOTEND, MAX_HOTEND, "ugello")
        if n:
            args += ["--nozzle", str(n)]
        if not gfile and not (fil and n):
            raise self.server.error("serve gcode, oppure filament + nozzle", 400)
        rc = await self._run_aux(args, regs["lidar_lang"])
        return {"ok": rc == 0, "args": args}

    # ------------------------------------------------- domanda prima della stampa
    @staticmethod
    def _describe(flow: int, fl: int, it: bool) -> str:
        parts = []
        if flow:
            parts.append("taratura PA" if it else "PA calibration")
        if fl:
            parts.append("controllo primo strato" if it else "first layer check")
        return " + ".join(parts) or ("nessuna funzione LiDAR" if it else "no LiDAR feature")

    async def _prompt(self, line: str) -> None:
        # klipper tronca la riga a # ; * : fuori dal testo della finestra
        line = re.sub(r"[#;*]", "_", line).replace('"', "'")[:200]
        await self._kapis().run_gcode('_LIDAR_SAY MSG="%s"' % line)

    async def _ask(self, filename: str, regs: Dict[str, Any]) -> str:
        # Finestra in Fluidd/Mainsail; ritorna una di ASK_CHOICES ("default" = nessuna
        # risposta entro il tempo limite).
        it = str(regs["lidar_lang"]) == "it"

        def T(i, e):
            return i if it else e
        try:
            timeout = max(10, min(600, int(regs.get("lidar_ask_timeout") or 60)))
        except (TypeError, ValueError):
            timeout = 60
        saved = self._describe(int(regs["lidar_flow"]), int(regs["lidar_firstlayer"]), it)
        fut = asyncio.get_event_loop().create_future()
        self.ask_future = fut

        def btn(label, choice, color, kind="prompt_button"):
            return "action:%s %s|_LIDAR_ASK_ANSWER CHOICE=%s|%s" % (kind, label, choice, color)
        lines = [
            "action:prompt_end",
            "action:prompt_begin " + T("LiDAR - prima di stampare", "LiDAR - before printing"),
            "action:prompt_text " + os.path.basename(filename),
            "action:prompt_text " + T("Senza risposta entro %d s: %s (impostazioni salvate)",
                                      "No answer within %d s: %s (saved settings)") % (timeout, saved),
            btn(T("Taratura PA + primo strato", "PA calibration + first layer"), "both", "primary"),
            btn(T("Solo taratura PA", "PA calibration only"), "pa", "secondary"),
            btn(T("Solo controllo primo strato", "First layer check only"), "fl", "secondary"),
            btn(T("Stampa senza LiDAR", "Print without LiDAR"), "none", "secondary"),
            btn(T("Non stampare", "Do not print"), "cancel", "error", "prompt_footer_button"),
            "action:prompt_show"]
        try:
            for line in lines:
                await self._prompt(line)
            try:
                choice = await asyncio.wait_for(asyncio.shield(fut), timeout)
            except asyncio.TimeoutError:
                choice = "default"
                await self._prompt("action:prompt_end")
                await self._say(T("LiDAR: nessuna risposta, uso le impostazioni salvate",
                                  "LiDAR: no answer, using the saved settings"))
        finally:
            self.ask_future = None
        return choice

    async def start_print(self, filename: str):
        if not filename:
            raise self.server.error("filename mancante", 400)
        if self._busy() or self.ask_future is not None:
            raise self.server.error("LiDAR: job o domanda gia' in corso (%s)"
                                    % (self.task if self._busy() else "domanda"), 409)
        regs = await self._regs()
        flow, fl = int(regs["lidar_flow"]), int(regs["lidar_firstlayer"])
        if int(regs.get("lidar_ask") or 0):
            it = str(regs["lidar_lang"]) == "it"
            choice = await self._ask(filename, regs)
            if choice == "cancel":
                await self._say("LiDAR: stampa non avviata" if it else "LiDAR: print not started")
                return {"started": False, "choice": choice}
            if await self._print_state() in ("printing", "paused"):
                await self._say("LiDAR: c'e' gia' una stampa in corso" if it
                                else "LiDAR: a print is already running")
                return {"started": False, "choice": choice}
            if choice != "default":
                flow = (flow if flow in (1, 2) else 1) if choice in ("both", "pa") else 0
                fl = 1 if choice in ("both", "fl") else 0
            await self._say("LiDAR: %s" % self._describe(flow, fl, it))
        args = ["print", filename,
                "--flow", str(flow),
                "--firstlayer", str(fl),
                "--on-bad", "cancel" if regs["lidar_on_bad"] == "cancel" else "pause",
                "--pa-apply", str(int(regs["lidar_pa_apply"])),
                "--material", str(regs["lidar_material"]).upper()]
        if int(regs["lidar_hotend"]):
            args += ["--hotend", str(int(regs["lidar_hotend"]))]
        if int(regs["lidar_bed"]):
            args += ["--bed", str(int(regs["lidar_bed"]))]
        self.last = {}
        await self._run("print", args, regs["lidar_lang"])
        return {"started": True, "task": "print", "args": args}

    async def cancel(self):
        if self.ask_future is not None and not self.ask_future.done():
            self.ask_future.set_result("cancel")
            await self._prompt("action:prompt_end")
            return {"cancelled": True, "task": "ask"}
        if not self._busy():
            return {"cancelled": False}
        assert self.proc is not None
        # SIGINT: gli script hanno try/finally che spengono laser e riscaldatori.
        self.proc.send_signal(signal.SIGINT)
        try:
            await asyncio.wait_for(self.proc.wait(), timeout=30)
        except asyncio.TimeoutError:
            self.proc.terminate()
        return {"cancelled": True, "task": self.task}

    def filaments(self) -> Dict[str, Any]:
        try:
            with open(CALIB) as f:
                fil = json.load(f).get("filaments", {})
        except Exception:
            fil = {}
        return {k: {kk: vv for kk, vv in v.items() if kk != "history"} for k, v in fil.items()}

    # ------------------------------------------------------- remote method
    async def _guard(self, coro) -> None:
        """Le remote method non hanno un chiamante che legga l'errore: lo si scrive in console."""
        try:
            await coro
        except Exception as e:
            msg = getattr(e, "message", None) or str(e)
            await self._say("LiDAR: %s" % msg)

    async def _rm_print(self, filename: str = "", **_kw) -> None:
        await self._guard(self.start_print(filename))

    async def _rm_pa(self, nozzle=None, bed=None, filament=None, material=None, fine=0,
                     start=None, step=None, gcode=None, **_kw) -> None:
        await self._guard(self.start_pa(nozzle, bed, filament, material, bool(int(fine or 0)),
                                        start, step, gcode))

    async def _rm_set(self, pa=None, gcode=None, filament=None, nozzle=None, **_kw) -> None:
        await self._guard(self.set_pa(pa, gcode, filament, nozzle))

    async def _rm_answer(self, choice: str = "default", **_kw) -> None:
        c = str(choice or "default").lower()
        if self.ask_future is not None and not self.ask_future.done():
            self.ask_future.set_result(c if c in ASK_CHOICES else "default")

    async def _rm_cancel(self, **_kw) -> None:
        r = await self.cancel()
        await self._say("LiDAR: %s" % ("job interrotto" if r.get("cancelled") else "nessun job in corso"))

    async def _rm_list(self, **_kw) -> None:
        # l'elenco lo stampa lo script (stessa logica e stessa lingua dei messaggi)
        try:
            regs = await self._regs()
            await self._run_aux(["list"], regs["lidar_lang"])
        except Exception as e:
            await self._say("LiDAR: %s" % (getattr(e, "message", None) or e))

    async def _rm_forget(self, filament: str = "", nozzle=None, **_kw) -> None:
        fil = self.filaments()
        name = re.sub(r"[^A-Za-z0-9.]+", "_", str(filament or "").strip()).strip("_.")
        keys = [k for k in fil if k == name or (nozzle and k == "%s_T%d" % (name, int(nozzle)))
                or (not nozzle and k.startswith(name + "_T"))]
        if not keys:
            await self._say("LiDAR: nessuna calibrazione per '%s'" % filament)
            return
        try:
            with open(CALIB) as f:
                db = json.load(f)
            for k in keys:
                db.get("filaments", {}).pop(k, None)
            tmp = CALIB + ".tmp"
            with open(tmp, "w") as f:
                json.dump(db, f, indent=2, sort_keys=True)
            os.rename(tmp, CALIB)
            await self._say("LiDAR: dimenticato %s" % ", ".join(keys))
        except Exception as e:
            await self._say("LiDAR: errore %s" % e)

    # ------------------------------------------------------------- HTTP
    async def _h_status(self, req) -> Dict[str, Any]:
        return {"running": self._busy(), "task": self.task, "registers": await self._regs(),
                "last": self.last, "log": self.log_lines[-60:], "path": self.path}

    async def _h_filaments(self, req) -> Dict[str, Any]:
        return {"filaments": self.filaments()}

    async def _h_settings(self, req) -> Dict[str, Any]:
        args = req.get_args()
        pending = []
        if "flow" in args:
            v = req.get_int("flow")
            if v not in (0, 1, 2):
                raise self.server.error("flow: 0, 1 o 2", 400)
            pending.append(("lidar_flow", v))
        if "firstlayer" in args:
            pending.append(("lidar_firstlayer", 1 if req.get_boolean("firstlayer") else 0))
        if "pa_apply" in args:
            pending.append(("lidar_pa_apply", 1 if req.get_boolean("pa_apply") else 0))
        if "ask" in args:
            pending.append(("lidar_ask", 1 if req.get_boolean("ask") else 0))
        if "ask_timeout" in args:
            v = req.get_int("ask_timeout")
            if not 10 <= v <= 600:
                raise self.server.error("ask_timeout: 10-600 s", 400)
            pending.append(("lidar_ask_timeout", v))
        if "on_bad" in args:
            v = req.get_str("on_bad")
            if v not in ("pause", "cancel"):
                raise self.server.error("on_bad: pause o cancel", 400)
            pending.append(("lidar_on_bad", v))
        if "lang" in args:
            v = req.get_str("lang").lower()
            if v not in ("en", "it"):
                raise self.server.error("lang: en o it", 400)
            pending.append(("lidar_lang", v))
        if "material" in args:
            v = req.get_str("material").upper()
            if v not in MATERIALS:
                raise self.server.error("materiale sconosciuto", 400)
            pending.append(("lidar_material", v))
        if "hotend" in args:
            pending.append(("lidar_hotend", self._temp(req.get_int("hotend"), MIN_HOTEND, MAX_HOTEND, "ugello") or 0))
        if "bed" in args:
            pending.append(("lidar_bed", self._temp(req.get_int("bed"), MIN_BED, MAX_BED, "piatto") or 0))
        for k, v in pending:          # tutto validato prima di scrivere
            await self._write_reg(k, v)
        return await self._regs()

    async def _h_pa(self, req) -> Dict[str, Any]:
        return await self.start_pa(req.get_str("nozzle", None), req.get_str("bed", None),
                                   req.get_str("filament", None), req.get_str("material", None),
                                   req.get_boolean("fine", False), req.get_str("start", None),
                                   req.get_str("step", None), req.get_str("gcode", None))

    async def _h_set(self, req) -> Dict[str, Any]:
        return await self.set_pa(req.get_str("pa", None), req.get_str("gcode", None),
                                 req.get_str("filament", None), req.get_str("nozzle", None))

    async def _h_print(self, req) -> Dict[str, Any]:
        return await self.start_print(req.get_str("filename"))

    async def _h_cancel(self, req) -> Dict[str, Any]:
        return await self.cancel()


def load_component(config):
    return K1Lidar(config)
