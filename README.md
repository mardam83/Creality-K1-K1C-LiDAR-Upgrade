# K1 LiDAR — Pressure Advance and first layer check with the Creality K1 laser

Use the LiDAR (laser) module of the **Creality K1** from Klipper, through Fluidd or
Mainsail:

- **Find the Pressure Advance of a filament** with one button, without printing an
  object: the printer prints a small test pattern on the left edge of the plate, scans
  it with the laser and gives you the PA with an error range. The value is **saved for
  that filament profile of your slicer** and reused automatically.
- **Before a print** (optional): the filament profile and temperatures are read from
  the gcode; a profile already calibrated gets its saved PA instantly, a new one is
  calibrated first. Optionally the **first layer is scanned** and the print pauses (or
  is cancelled) if it is bad, with the position of the defect in mm.

📖 **Manual**: [English](docs/MANUAL.md) · [Italiano](docs/GUIDA_IT.md)

## Requirements
- Creality **K1** (CR-K1) **with the LiDAR module**. K1 Max / K1C: not supported.
- Root access and **Moonraker + Fluidd or Mainsail** (e.g. via the
  [Creality Helper-Script](https://github.com/Guilouz/Creality-Helper-Script)).
- Tested on firmware 1.3.5.22. Python 3.8 and numpy are already in the firmware.
- OrcaSlicer, Creality Print, PrusaSlicer, SuperSlicer or Bambu Studio.

## Install
On the printer, SSH as root:
```sh
git clone https://github.com/<user>/k1-lidar /usr/data/k1-lidar
sh /usr/data/k1-lidar/install.sh --with-hook          # add --lang it for Italian messages
```
`--with-hook` enables the "before a print" features; without it you only get the
buttons. Updates via Fluidd/Mainsail → *Update manager* → **k1-lidar**. Run
`install.sh` again after a Creality firmware update. Uninstall:
`sh /usr/data/k1-lidar/uninstall.sh`.

In Fluidd → **Macros** every `LIDAR_...` button shows what it does; **`LIDAR_HELP`**
prints a short guide in the console.

## Filament profiles
The PA is saved **per filament profile of the slicer + nozzle temperature**, e.g.
"Generic PLA" at 220 °C. The profile name travels inside the gcode, so at print start
the printer knows which filament is loaded.

- **Two different filaments need two different profiles** in the slicer (duplicate and
  rename, e.g. "PLA eSun orange", "PLA Sunlu black"), even at the same temperature.
- Turn **off "Enable pressure advance"** in the slicer's filament profile, or the
  slicer's PA overrides the measured one.
- `LIDAR_PA_LIST` lists them, `LIDAR_PA_SET` sets one by hand, `LIDAR_PA_FORGET`
  deletes one. Stored in `/usr/data/lidar_calib.json`.

**Which PA a print uses**: 1) the gcode's, if the slicer has *Enable pressure advance*
on; 2) otherwise the one measured for that profile and temperature; 3) if there is none
and `LIDAR_FLOW` is on, it is measured before the print; 4) otherwise `printer.cfg`.

## Quick start (Fluidd / Mainsail → Macros)
**Find the PA of a filament now** (prints only the test pattern at X 0–30 mm, ~15 min):
```
LIDAR_PA_CALIBRATE GCODE="any_file_sliced_with_that_profile.gcode"
```
or `LIDAR_PA_CALIBRATE FILAMENT="exact slicer profile name" MATERIAL=PLA NOZZLE=220 BED=60`.
Result in the console, e.g.
`LiDAR PA result: 0.0274 (80% range 0.0230-0.0303), confidence high`.

**Calibrate automatically before prints** — press once:

| Button | |
|---|---|
| `LIDAR_FLOW MODE=1` | profiles never measured are calibrated before the print; the others get their saved PA |
| `LIDAR_FIRSTLAYER ON=1 ON_BAD=pause` | also scan the first layer |
| `LIDAR_STATUS` · `LIDAR_CANCEL` | what is on · stop a job |

Then print as usual (Fluidd, Mainsail or the display).

## How the PA is measured
Every corner of the zig-zag test pattern is printed with a different PA (32 values).
With too little PA a sharp corner gets a blob, with too much it gets thin or hollow.
The laser measures how **full** each corner is compared with the straight line; the PA
is the value at which the corner is as full as the straight line, with an 80 % range
from a bootstrap over all 32 corners. Two independent calibrations of the same PLA
spool gave 0.0274 and 0.0270. Details in the [manual](docs/MANUAL.md#how-the-pa-is-measured).

Creality's own PA analysis is not used (on the same filament it returned 0.054 or
0.0375 depending on where the value fell in the pattern).

## Safety
Keep the left edge of the plate clear when a calibration may run. Test pattern travel
moves are at Z 5 mm. `LIDAR_CANCEL` stops a job and switches laser and heaters off. The
print hook is optional: `install.sh --no-hook` makes prints stock again.

## Disclaimer
Not affiliated with Creality. Use at your own risk: it moves the printer, heats it and
prints. All code in this repository is original work; it talks to the printer's stock
software and laser module only through their interfaces.

License: [GPL-3.0](LICENSE).
