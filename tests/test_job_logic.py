#!/usr/bin/env python3
"""Logica dell'orchestratore all'avvio stampa (k1lidar/job.py), senza stampante.

Klipper, laser, taratura PA e primo strato sono sostituiti da finti che registrano
le chiamate. Si verifica COSA viene deciso per ogni combinazione di impostazioni:
riuso del PA salvato, taratura dei filamenti nuovi, un solo livellamento, avvio
della stampa con il comando giusto.
Uso: python tests/test_job_logic.py
"""
import os, sys, tempfile
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from k1lidar import job, store  # noqa: E402

# gcode sintetico (nostro) con intestazione in stile OrcaSlicer: stesso filamento
# e temperature del Corpo1 usato sulla stampante di sviluppo
GCODE = os.path.join(ROOT, "tests", "data", "sample_orca.gcode")
calls = []


class FakeL(object):
    name = "fake"

    def shutdown(self):
        calls.append("laser_off")


def setup(hook=True, slicer_pa=False):
    del calls[:]
    job.kl = lambda g, t=None: calls.append(g)
    job.kl_query = lambda o, t=None: {"configfile": {"settings": (
        {"gcode_macro sdcard_print_file": {}, "gcode_macro _lidar_pa_lock": {}} if hook else {})}}
    real_read = job.GM.__dict__.get("_real_read") or job.GM.read
    job.GM._real_read = real_read
    job.GM.read = lambda p: dict(real_read(p), slicer_pa_enabled=slicer_pa, slicer_pa=0.04)
    job.echo = lambda m: calls.append("echo:" + m)
    job.log = lambda m="": None
    job.factory_level = lambda h, b: calls.append("LEVEL %d/%d" % (h, b))
    job.laser_up = lambda b: (calls.append("laser_on"), FakeL())[1]
    job.self_cal = lambda L: calls.append("selfcal")
    job.acquire_lock = lambda: True
    job.release_lock = lambda: calls.append("unlock")
    job.heaters_off = lambda: calls.append("heaters_off")
    job.park = lambda z=60: None
    job.resolve_temps = lambda ns: (ns.hotend or 220, ns.bed or 60, "x")
    job.PA.run = lambda o, L=None, prepared=False: (
        calls.append("PA_RUN fil=%s hot=%d prepared=%s selfcal=%s" % (o.fil, o.hot, prepared, not o.no_selfcal)),
        {"final_pa": 0.031})[1]
    job.FL.run = lambda a, L=None, prepared=False: (
        calls.append("FL_RUN prepared=%s on_bad=%s" % (prepared, a.on_bad)), {"verdict": "GOOD"})[1]
    job.FL.resolve_gcode = lambda n: n


def args(**kw):
    d = dict(file=GCODE, flow=0, firstlayer=0, on_bad="pause", pa_apply=1, material="PLA",
             hotend=0, bed=0, backend="driver")
    d.update(kw)
    return SimpleNamespace(**d)


def check(name, cond):
    print("  %s  %s" % ("ok " if cond else "ERR", name))
    if not cond:
        print("     chiamate:", calls)
        raise SystemExit(1)


def main():
    if not os.path.isfile(GCODE):
        print("gcode di prova assente (%s): test saltato" % GCODE)
        return 0
    tmp = tempfile.mkdtemp()
    store.CALIB = os.path.join(tmp, "calib.json")
    key = "Creality_HF_Generic_Speed_PLA_Copia_T220"

    setup()
    job.do_print(args(flow=1))
    check("filamento NUOVO + flow=1 -> livella, tara (prepared), poi stampa senza rilivellare",
          calls.count("LEVEL 220/60") == 1 and any(c.startswith("PA_RUN fil=%s" % key) and "prepared=True" in c
                                                  and "selfcal=False" in c for c in calls)
          and "PRINT_PREPARED" in calls and 'SDCARD_PRINT_FILE_BASE FILENAME="%s"' % os.path.basename(GCODE) in calls)

    store.save_calib(key, pa=0.031)
    setup()
    job.do_print(args(flow=1))
    check("filamento GIA' TARATO + flow=1 -> applica PA salvato, niente laser ne' livellamento",
          "SET_PRESSURE_ADVANCE ADVANCE=0.0310" in calls and "laser_on" not in calls
          and not any(c.startswith("LEVEL") for c in calls) and "PRINT_PREPARED" not in calls
          and any(c.startswith("SDCARD_PRINT_FILE_BASE") for c in calls))

    setup()
    job.do_print(args(flow=2))
    check("flow=2 -> tara anche se gia' tarato", any(c.startswith("PA_RUN") for c in calls))

    setup()
    job.do_print(args(flow=1, firstlayer=1, on_bad="cancel"))
    check("tarato + primo strato -> PA salvato, un livellamento, FL in sessione, niente PRINT_PREPARED qui",
          "SET_PRESSURE_ADVANCE ADVANCE=0.0310" in calls and calls.count("LEVEL 220/60") == 1
          and "FL_RUN prepared=True on_bad=cancel" in calls and not any(c.startswith("PA_RUN") for c in calls)
          and "selfcal" in calls and "unlock" in calls)

    setup()
    job.do_print(args(flow=0, firstlayer=0, pa_apply=1))
    check("tutto spento ma pa_apply=1 -> solo PA salvato e avvio normale",
          "SET_PRESSURE_ADVANCE ADVANCE=0.0310" in calls and "laser_on" not in calls)

    setup(hook=False)
    job.do_print(args(flow=0, firstlayer=0, pa_apply=0))
    check("senza hook -> SDCARD_PRINT_FILE (non _BASE), nessun PA",
          any(c.startswith('SDCARD_PRINT_FILE FILENAME=') for c in calls)
          and not any("PRESSURE_ADVANCE" in c for c in calls))

    setup()
    job.do_print(args(flow=1))
    check("flow=1 con PA salvato -> PA bloccato sulla stampa (sostituisce quello del gcode)",
          "_LIDAR_PA_LOCK PA=0.0310 ARMED=0" in calls
          and calls.index("_LIDAR_PA_LOCK PA=0.0310 ARMED=0") < next(
              i for i, c in enumerate(calls) if c.startswith("SDCARD_PRINT_FILE_BASE")))

    setup(slicer_pa=True)
    job.do_print(args(flow=2))
    check("flow=2 + PA dello slicer nel gcode -> blocca il PA appena tarato e avvisa",
          "_LIDAR_PA_LOCK PA=0.0310 ARMED=0" in calls
          and any(c.startswith("echo:") and "0.0310" in c for c in calls))

    setup()
    job.do_print(args(flow=1, firstlayer=1))
    check("flow + primo strato -> lock armato per la stampa FIRST_FLOOR_PRINT",
          "_LIDAR_PA_LOCK PA=0.0310 ARMED=1" in calls
          and calls.index("_LIDAR_PA_LOCK PA=0.0310 ARMED=1") < calls.index("FL_RUN prepared=True on_bad=pause"))

    setup(slicer_pa=True)
    job.do_print(args(flow=0, firstlayer=0, pa_apply=1))
    check("taratura spenta (solo pa_apply) -> nessun lock, vince il PA del gcode (avviso)",
          not any(c.startswith("_LIDAR_PA_LOCK") for c in calls)
          and any(c.startswith("echo:") for c in calls))

    setup(hook=False)
    job.do_print(args(flow=1))
    check("macro di lock assente -> nessun _LIDAR_PA_LOCK",
          not any(c.startswith("_LIDAR_PA_LOCK") for c in calls))

    setup()
    job.PA.run = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("scan vuoto"))
    try:
        job.do_print(args(flow=2))
    except RuntimeError:
        pass
    check("errore durante la taratura -> riscaldatori spenti, laser spento, lock rilasciato, nessuna stampa",
          "heaters_off" in calls and "laser_off" in calls and "unlock" in calls
          and not any(c.startswith("SDCARD") for c in calls))
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
