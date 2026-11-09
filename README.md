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

📖 **Manual**: [English](docs/MANUAL.md) · [Italiano](docs/GUIDA_IT.md) · [Detailed explanation & diagrams (IT)](docs/SPIEGAZIONE_SEMPLICE.md)

## Requirements
- Creality **K1** (CR-K1) **with the LiDAR module**. K1 Max / K1C: not supported.
- Root access and **Moonraker + Fluidd or Mainsail** (e.g. via the
  [Creality Helper-Script](https://github.com/Guilouz/Creality-Helper-Script)).
- Tested on firmware 1.3.5.22. Python 3.8 and numpy are already in the firmware.
- OrcaSlicer, Creality Print, PrusaSlicer, SuperSlicer or Bambu Studio.

## Install
From a PC, in the downloaded folder (needs the printer IP and its root password):
```sh
install-from-pc.bat 192.168.1.xx --with-hook          # Windows (or double-click it)
sh install-from-pc.sh 192.168.1.xx --with-hook        # Linux / macOS
```
Add `--lang it` for Italian messages. Or on the printer, SSH as root:
```sh
git clone https://github.com/mardam83/Creality-K1-K1C-LiDAR-Upgrade /usr/data/k1-lidar
sh /usr/data/k1-lidar/install.sh --with-hook
```
`--with-hook` enables the "before a print" features; without it you only get the
buttons. Update: run `install-from-pc` again with no options, or
`sh /usr/data/k1-lidar/update.sh` on the printer (`--rollback` to go back). Run
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
- With `LIDAR_FLOW` on, the LiDAR PA **replaces** the PA the slicer writes in the gcode
  (*Enable pressure advance*). With `LIDAR_FLOW` off, the slicer's PA wins: turn that
  option off in the filament profile if you want the saved PA.
- `LIDAR_PA_LIST` lists them, `LIDAR_PA_SET` sets one by hand, `LIDAR_PA_FORGET`
  deletes one. Stored in `/usr/data/lidar_calib.json`.

**Which PA a print uses**: with `LIDAR_FLOW` on, always the LiDAR one (saved for that
profile and temperature, or measured before the print if there is none), even if the
gcode sets its own. With `LIDAR_FLOW` off: 1) the gcode's, if the slicer has *Enable
pressure advance* on; 2) otherwise the saved one; 3) otherwise `printer.cfg`.

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
| `LIDAR_ASK_ON` | or decide at every print: Fluidd/Mainsail asks (PA / first layer / both / none), saved settings after 60 s without an answer |
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

## Improvements over the stock Creality system

Real-world testing on the K1 (firmware 1.3.5.22) identified several structural flaws in Creality's factory implementation that this project redesigns from scratch:

### 1. Pressure Advance (PA) calibration
* **True physical repeatability**:
  * *Stock Creality*: Two clean runs with the same PLA spool 20 minutes apart yielded **0.054** and **0.0375**; the algorithm favored rows based on position in the pattern rather than corner geometry.
  * *K1 LiDAR*: Analyzes physical corner volume fill ($\pm 1.5$ mm) across all laser profiles. Repeatability tests on the same spool consistently gave **0.0274** and **0.0270** with an 80% bootstrap confidence interval.
* **Laser frame synchronization**:
  * *Stock Creality*: Inferred frame positions from frame count (~11 fps). Because Klipper delays motion execution responses, frames were registered late, capturing the head stationary at the end of travel with incorrect coordinates.
  * *K1 LiDAR*: Uses a single continuous pass at controlled speed (`F400`), matching each frame timestamp directly against Klipper's planned motion queue (`dump_trapq`). Max position error is under 0.3 mm.
* **Validation & safety**:
  * Incomplete scans or corrupted rows are rejected with clear diagnostic messages instead of saving erroneous values. Travel moves use 5 mm Z-hops to avoid collisions with printed test lines.
* **Per-filament profile storage**:
  * Stores and recalls PA values per slicer filament profile and nozzle temperature (OrcaSlicer, PrusaSlicer, Bambu Studio, Creality Print), with customized search ranges for PLA/ABS/ASA, PETG, and TPU.

### 2. First layer inspection (C2 Analyzer)
* **Precise defect localization**:
  * *Stock Creality*: Returns an opaque binary verdict (pass/fail) without indicating where the defect is, and can be fooled by surface reflections.
  * *K1 LiDAR (C2)*: Compares a 3D height map directly against the sliced G-code, reporting missing or lifted extrusion with exact $(X, Y)$ coordinates and defect area in mm².
* **Elimination of phantom defects**:
  * Compensates for bed thermal expansion (Z-offset shifts up to 0.1 mm between runs), measures and corrects ~3 mm laser lens distortion, and filters out optical reflections from glossy plates.
* **Fix for the infinite pause loop**:
  * Stock firmware frequently left the first-layer check flag set, pausing the printer at every subsequent layer. K1 LiDAR properly clears the layer state upon resumption.
* **Interactive print-start prompt (`LIDAR_ASK`)**:
  * An interactive Fluidd/Mainsail prompt (`action:prompt`) lets you choose whether to run PA calibration, inspect the first layer, do both, or skip before each print.

## Safety
Keep the left edge of the plate clear when a calibration may run. Test pattern travel
moves are at Z 5 mm. `LIDAR_CANCEL` stops a job and switches laser and heaters off. The
print hook is optional: `install.sh --no-hook` makes prints stock again.

## Disclaimer
Not affiliated with Creality. Use at your own risk: it moves the printer, heats it and
prints. All code in this repository is original work; it talks to the printer's stock
software and laser module only through their interfaces.

License: [GPL-3.0](LICENSE).
