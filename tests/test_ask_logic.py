#!/usr/bin/env python3
"""Domanda prima della stampa (componente Moonraker moonraker/k1lidar.py), senza stampante.

Moonraker e Klipper sono finti: si verifica che la finestra venga aperta, che ogni
risposta diventi le opzioni giuste per il job (--flow / --firstlayer), che senza
risposta valgano i registri salvati e che "Non stampare" / LIDAR_CANCEL non avviino nulla.
Uso: python tests/test_ask_logic.py
"""
import asyncio, importlib.util, os, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
spec = importlib.util.spec_from_file_location("k1lidar_moon", os.path.join(ROOT, "moonraker", "k1lidar.py"))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


class FastAsyncio(object):
    """asyncio con wait_for accorciato (il tempo limite vero e' >= 10 s)."""
    def __getattr__(self, n):
        return getattr(asyncio, n)

    @staticmethod
    def wait_for(f, timeout):
        return asyncio.wait_for(f, 0.3)


mod.asyncio = FastAsyncio()


class ServerError(Exception):
    def __init__(self, msg, code=400):
        Exception.__init__(self, msg)
        self.message = msg


class FakeApis(object):
    def __init__(self, regs, state="standby"):
        self.regs, self.state, self.gcode = regs, state, []

    async def run_gcode(self, g):
        self.gcode.append(g)

    async def query_objects(self, objs):
        if "save_variables" in objs:
            return {"save_variables": {"variables": dict(self.regs)}}
        return {"print_stats": {"state": self.state}}


class FakeServer(object):
    def __init__(self, apis):
        self.apis = apis
        self.remote = {}

    def get_event_loop(self):
        return None

    def register_endpoint(self, *a):
        pass

    def register_remote_method(self, name, cb):
        self.remote[name] = cb

    def lookup_component(self, name):
        return self.apis

    def error(self, msg, code=400):
        return ServerError(msg, code)

    def send_event(self, *a):
        pass


class FakeConfig(object):
    def __init__(self, server):
        self.server = server

    def get_server(self):
        return self.server

    def get(self, k, d=None):
        return ROOT


def make(regs, state="standby"):
    base = dict(mod.DEFAULTS, lidar_ask=1, lidar_lang="it")
    base.update(regs)
    apis = FakeApis(base, state)
    srv = FakeServer(apis)
    c = mod.K1Lidar(FakeConfig(srv))
    c.ran = []

    async def fake_run(task, args, lang):
        c.ran.append(args)
    c._run = fake_run
    return c, srv, apis


async def answer_later(srv, choice, delay=0.05):
    await asyncio.sleep(delay)
    await srv.remote["k1lidar_answer"](choice=choice)


def opt(args, name):
    return args[args.index(name) + 1]


def check(name, cond, extra=""):
    print("  %s  %s" % ("ok " if cond else "ERR", name))
    if not cond:
        print("     ", extra)
        raise SystemExit(1)


async def run_case(regs, choice, state="standby"):
    c, srv, apis = make(regs, state)
    tasks = [c.start_print("pezzo.gcode")]
    if choice is not None:
        tasks.append(answer_later(srv, choice))
    res = (await asyncio.gather(*tasks))[0]
    return c, apis, res


async def main():
    c, apis, _ = await run_case({"lidar_flow": 0, "lidar_firstlayer": 0}, "both")
    prompt = [g for g in apis.gcode if "action:prompt" in g]
    check("finestra aperta con 4 scelte + Non stampare, poi mostrata",
          any("prompt_begin" in g for g in prompt) and any("prompt_show" in g for g in prompt)
          and sum("_button " in g and "_LIDAR_ASK_ANSWER CHOICE=" in g for g in prompt) == 5,
          prompt)
    a = c.ran[0]
    check("'PA + primo strato' con flow spento -> --flow 1 --firstlayer 1",
          opt(a, "--flow") == "1" and opt(a, "--firstlayer") == "1", a)

    c, _, _ = await run_case({"lidar_flow": 2, "lidar_firstlayer": 1}, "pa")
    a = c.ran[0]
    check("'Solo PA' con flow=2 salvato -> --flow 2 --firstlayer 0",
          opt(a, "--flow") == "2" and opt(a, "--firstlayer") == "0", a)

    c, _, _ = await run_case({"lidar_flow": 1}, "fl")
    a = c.ran[0]
    check("'Solo primo strato' -> --flow 0 --firstlayer 1",
          opt(a, "--flow") == "0" and opt(a, "--firstlayer") == "1", a)

    c, _, _ = await run_case({"lidar_flow": 1, "lidar_firstlayer": 1}, "none")
    a = c.ran[0]
    check("'Stampa senza LiDAR' -> --flow 0 --firstlayer 0 (il PA salvato resta come da pa_apply)",
          opt(a, "--flow") == "0" and opt(a, "--firstlayer") == "0" and opt(a, "--pa-apply") == "1", a)

    c, apis, _ = await run_case({"lidar_flow": 2, "lidar_firstlayer": 1}, None)
    a = c.ran[0]
    check("nessuna risposta -> impostazioni salvate (--flow 2 --firstlayer 1) e finestra chiusa",
          opt(a, "--flow") == "2" and opt(a, "--firstlayer") == "1"
          and any("prompt_end" in g for g in apis.gcode[-3:]), (a, apis.gcode[-3:]))

    c, _, res = await run_case({}, "cancel")
    check("'Non stampare' -> nessun job", not c.ran and res["started"] is False)

    c, srv, _ = make({})
    r = await asyncio.gather(c.start_print("pezzo.gcode"),
                             (lambda: asyncio.sleep(0.05))(), return_exceptions=True)
    check("risposta mai data e nessun crash", not isinstance(r[0], Exception), r)

    c, srv, _ = make({})

    async def cancel_later():
        await asyncio.sleep(0.05)
        return await c.cancel()
    res, cres = await asyncio.gather(c.start_print("pezzo.gcode"), cancel_later())
    check("LIDAR_CANCEL durante la domanda -> nessun job", not c.ran and cres["task"] == "ask")

    c, srv, _ = make({})

    async def second():
        await asyncio.sleep(0.05)
        try:
            await c.start_print("altro.gcode")
            return None
        except ServerError as e:
            return e
    r1, err, _ = await asyncio.gather(c.start_print("pezzo.gcode"), second(),
                                   answer_later(srv, "none", 0.1))
    check("seconda stampa mentre la domanda e' aperta -> rifiutata", isinstance(err, ServerError)
          and len(c.ran) == 1)

    c, _, res = await run_case({}, "both", state="printing")
    check("stampa partita nel frattempo (display) -> nessun job", not c.ran and res["started"] is False)

    c, _, _ = await run_case({"lidar_ask": 0, "lidar_flow": 1, "lidar_firstlayer": 1}, None)
    a = c.ran[0]
    check("lidar_ask=0 -> nessuna finestra, registri salvati",
          opt(a, "--flow") == "1" and opt(a, "--firstlayer") == "1")
    print("OK")


if __name__ == "__main__":
    asyncio.run(main())
