#!/usr/bin/env python3
# lidar_pa.py - taratura Pressure Advance col LiDAR (riga di comando).
# La logica sta in k1lidar/pa.py; qui solo le opzioni.
#
#   python3 lidar_pa.py --material PLA --filament "PLA arancione" --hotend 220 --bed 60 [--fine]
#
# Il PA trovato viene applicato (SET_PRESSURE_ADVANCE) e salvato per il filamento in
# /usr/data/lidar_calib.json: le stampe successive con lo stesso filamento lo riusano.
import argparse, os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from k1lidar import add_common_args, resolve_temps, filament_key  # noqa: E402
from k1lidar import pa  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="Taratura PA automatica col LiDAR")
    add_common_args(ap)
    ap.add_argument("--level", action="store_true", help="livellamento G29 invece di quello di fabbrica")
    ap.add_argument("--factory-level", action="store_true", default=True,
                    help="livellamento di FABBRICA (default)")
    ap.add_argument("--skip-print", action="store_true", help="pattern gia' sul piatto: solo scan e analisi")
    ap.add_argument("--no-selfcal", action="store_true", help="salta il self-cal del laser sull'adesivo")
    ap.add_argument("--no-apply", action="store_true", help="non applicare il PA trovato")
    ap.add_argument("--apply", action="store_true", help="(compatibilita': il PA si applica di default)")
    ap.add_argument("--fine", action="store_true",
                    help="seconda passata a passo 0.001 attorno al risultato (secondo pattern a +30 mm)")
    ap.add_argument("--start", type=float, help="primo PA del pattern (default per materiale)")
    ap.add_argument("--step", type=float, help="passo fra i 32 nodi (default 0.002)")
    ap.add_argument("--remember", action="store_true", help="salva --exposure per il filamento")
    ap.add_argument("--scan-mode", default="cont", choices=["cont", "step"])
    ap.add_argument("--scan-speed", type=int, default=400)
    ap.add_argument("--frames", type=int, default=170)
    a = ap.parse_args()
    hot, bed, _src = resolve_temps(a)
    o = pa.Opts(material=a.material, fil=filament_key(a, hot), hot=hot, bed=bed,
                frames=a.frames, scan_mode=a.scan_mode, scan_speed=a.scan_speed,
                no_selfcal=a.no_selfcal, factory_level=not a.level, level=a.level,
                skip_print=a.skip_print, apply=not a.no_apply, fine=a.fine,
                start=a.start, step=a.step, exposure=a.exposure, backend=a.backend,
                remember=a.remember)
    try:
        res = pa.run(o)
    except Exception as e:
        from k1lidar import echo
        from k1lidar.i18n import T
        echo(T("pa_fail", err=e))
        sys.exit(1)
    # riga letta dal componente Moonraker
    print("\n>>> PA OTTIMALE (LiDAR): %s <<<" % ("%.4f" % res["final_pa"]).rstrip("0").rstrip("."))


if __name__ == "__main__":
    main()
