# K1 LiDAR — user manual

**Pressure Advance** calibration and **first layer check** with the LiDAR (laser)
module of the **Creality K1**, from Klipper, with Fluidd or Mainsail.

> Italiano: [GUIDA_IT.md](GUIDA_IT.md)

## Contents
1. [What it does](#what-it-does)
2. [Requirements](#requirements)
3. [Installation](#installation)
4. [Filament profiles: how the PA is linked to your slicer](#filament-profiles-how-the-pa-is-linked-to-your-slicer)
5. [Which PA a print uses](#which-pa-a-print-uses)
6. [Finding the PA of a filament (no print)](#finding-the-pa-of-a-filament-no-print)
7. [Calibration before a print](#calibration-before-a-print)
8. [All the buttons](#all-the-buttons)
9. [How the PA is measured](#how-the-pa-is-measured)
10. [Reading the result](#reading-the-result)
11. [Safety](#safety) · [How it works](#how-it-works) · [Status and limits](#status-and-limits) · [Troubleshooting](#troubleshooting)

## What it does
- **Finds the PA of a filament** with one button, without printing an object: the
  printer prints a small test pattern on the left edge of the plate, scans it with the
  laser and gives you the PA with its error range. The value is **saved for that
  filament profile** and reused automatically.
- **Before a print** (optional): the filament profile and temperatures are read from
  the gcode. A profile that was already calibrated gets its saved PA at once; a new one
  is calibrated first. Optionally the **first layer is scanned** and the print pauses
  (or is cancelled) if it is bad, with the position of the defect in mm.

## Requirements
- **K1 with the LiDAR module** (model CR-K1): the installer checks it. K1 Max and K1C
  are not supported (not tested).
- **Root** access and **Moonraker + Fluidd or Mainsail** (for example with the
  [Creality Helper-Script](https://github.com/Guilouz/Creality-Helper-Script)).
- Tested on firmware **1.3.5.22**; with other versions the installer warns.
- Nothing to download on the printer: Python 3.8 and numpy are part of the firmware.
- A slicer that writes its settings at the end of the gcode: OrcaSlicer, Creality
  Print, PrusaSlicer, SuperSlicer, Bambu Studio.

## Installation
You need root access on the printer (screen → Settings → Root account: it shows the
password) and its IP address.

**From a PC (easiest)** — download the repository (*Code → Download ZIP*, then unzip),
then from that folder:

- **Windows 10/11**: double-click `install-from-pc.bat`, type the printer IP and, the
  first time, the options `--with-hook --lang it` (or just `--with-hook`). Or from a
  terminal: `install-from-pc.bat 192.168.1.xx --with-hook`
- **Linux / macOS**: `sh install-from-pc.sh 192.168.1.xx --with-hook`

The root password is asked once. The script copies the toolkit over SSH (only `ssh` and
`tar` are needed, both built into Windows) to `/usr/data/k1-lidar` and runs the installer.

**On the printer** (SSH as root), if it has `git`:
```sh
git clone https://github.com/mardam83/Creality-K1-K1C-LiDAR-Upgrade /usr/data/k1-lidar
sh /usr/data/k1-lidar/install.sh --with-hook          # add --lang it for Italian messages
```
The installer checks the prerequisites, backs up `printer.cfg`, links the macros and the
Moonraker component, adds an *Update manager* entry and restarts Klipper and Moonraker.
It is safe to run again.

| Option | |
|---|---|
| `--with-hook` | enables the "before a print" features (wraps `SDCARD_PRINT_FILE`; with every feature off it changes nothing). Without it you only get the buttons. |
| `--no-hook` | removes the hook (prints are stock again) |
| `--lang en\|it` | language of the messages |
| `--check` | only checks the prerequisites, changes nothing |

- **Updates**: download the new version and run `install-from-pc` again **without
  options** (hook, language, settings and calibrations are kept). Or on the printer:
  `sh /usr/data/k1-lidar/update.sh` (downloads the latest version; `git pull` for a git
  checkout, which also shows up in Fluidd/Mainsail → *Update manager*). The previous
  version is kept: `sh /usr/data/k1-lidar/update.sh --rollback` goes back to it, and if
  the installer fails the previous version is restored automatically. Updating is
  refused while a print or a LiDAR job is running.
- **After a Creality firmware update** run `install.sh` again (the update rewrites
  `printer.cfg` and may delete `moonraker.conf`).
- **Uninstall**: `sh /usr/data/k1-lidar/uninstall.sh` (`--purge` also deletes the saved
  calibrations).

After installing, open Fluidd → **Macros**: every `LIDAR_...` button shows a
description of what it does. **`LIDAR_HELP`** prints a short guide in the console.

## Filament profiles: how the PA is linked to your slicer
The PA is saved **per filament profile of the slicer and per nozzle temperature**:

```
"Creality HF Generic Speed PLA - Copia" at 220 °C  ->  PA 0.0274
"Creality Generic PETG+HS ESUN"         at 245 °C  ->  PA ...
```

- The **profile name** is the one you pick in the slicer's *Filament* box. The slicer
  writes it into the gcode (`filament_settings_id`), so at print start the printer
  knows which filament is loaded and looks up its PA.
- The **nozzle temperature** is the one of the layers after the first. The same
  profile used at a different temperature is a different entry (the PA changes with
  temperature).
- **Two different filaments need two different profiles**, even if they print at the
  same temperature: in the slicer duplicate the profile and give it a clear name, e.g.
  "PLA eSun orange" and "PLA Sunlu black". The name is what tells them apart: if you
  print both with the same profile, they share one PA.
- With `LIDAR_FLOW` on, the LiDAR PA **replaces** the one the slicer writes into the
  gcode (*Enable pressure advance*). With `LIDAR_FLOW` off the slicer's PA wins: turn
  that option off in the filament profile if you want the saved PA.

Where it is stored: `/usr/data/lidar_calib.json` on the printer (outside the toolkit
folder, so updates do not touch it). Manage it with the buttons:

| Button | |
|---|---|
| `LIDAR_PA_LIST` | lists the profiles with a saved PA: name, temperature, PA, range, date |
| `LIDAR_PA_SET PA=0.027 GCODE="file.gcode"` | sets the PA of the profile used in that gcode by hand (or `FILAMENT="exact profile name" NOZZLE=220`) |
| `LIDAR_PA_FORGET FILAMENT="profile name" [NOZZLE=220]` | deletes it (without `NOZZLE`: every temperature) |

## Which PA a print uses
**With `LIDAR_FLOW` on (1 or 2) the LiDAR PA always wins**: the one saved for the
profile and temperature, or the one measured before the print. If the gcode sets its
own PA (slicer *Enable pressure advance*, `M900 K`, the display's quiet mode), during
that print it is replaced by the LiDAR value; a console line tells you so. Outside a
print `SET_PRESSURE_ADVANCE` works as usual. Needs the hook (`--with-hook`).

With `LIDAR_FLOW` off, in this order — the first one that applies wins:

1. **The gcode's own PA**, if the slicer profile has *Enable pressure advance* on. You
   get a warning in the console.
2. **The PA measured with the LiDAR** for that profile and temperature, if there is one.
   It is applied just before the print starts (`LIDAR_PA_APPLY ON=1`, the default;
   needs the hook).
3. If `LIDAR_FLOW` is on and the profile has no PA yet: **it is measured now**, before
   the print, then saved and applied.
4. Otherwise **the value in `printer.cfg`** (`pressure_advance` in `[extruder]`).

Note: with `LIDAR_FLOW` off, the display's *quiet mode*, if you turn it on during a
print, sets PA 0.05 until you turn it off.

## Finding the PA of a filament (no print)
Load the filament, clear the left edge of the plate (X 0–30 mm), then in Fluidd →
Macros:

**Recommended — from a gcode** (upload any file sliced with that filament profile):
```
LIDAR_PA_CALIBRATE GCODE="my_part_PLA.gcode"
```
Profile name, material and temperatures are taken from the gcode: the result is saved
exactly under the name that prints with that profile will look for. The file is only
read, not printed.

**By hand**:
```
LIDAR_PA_CALIBRATE FILAMENT="Creality HF Generic Speed PLA - Copia" MATERIAL=PLA NOZZLE=220 BED=60
```
`FILAMENT` must be **exactly** the profile name of the slicer, otherwise prints will
not find it. Without `FILAMENT` and `GCODE` the PA is saved under the material name
only (e.g. `PLA_T220`) and you get a warning.

- Takes about 15 minutes: factory leveling, scan of the bare plate, test pattern,
  cool-down, scan of the pattern, analysis.
- The PA is applied at once and saved. The console shows for example
  `LiDAR PA result: 0.0274 (80% range 0.0230-0.0303), confidence high`.
- Optional: `FINE=1` adds a second pattern 30 mm to the right with steps of 0.001
  around the first result (keep that area clear too). `START=` / `STEP=` change the
  range of the 32 test values (see [How the PA is measured](#how-the-pa-is-measured)).

## Calibration before a print
Needs the hook (`install.sh --with-hook`). In Fluidd → Macros press **once**:

| Button | What happens at every print start |
|---|---|
| `LIDAR_FLOW MODE=1` | **recommended**: profiles never measured are calibrated before the print (~15 min, only the first time); for the others the saved PA is applied at once |
| `LIDAR_FLOW MODE=2` | calibrates before **every** print |
| `LIDAR_FLOW MODE=0` | off (the saved PA is still applied, see `LIDAR_PA_APPLY`) |
| `LIDAR_FIRSTLAYER ON=1 ON_BAD=pause` | also checks the first layer (`ON_BAD=cancel` cancels a bad print) |
| `LIDAR_PA_APPLY ON=0` | do not apply saved PAs (use `printer.cfg` / the gcode) |
| `LIDAR_ASK_ON` | **ask every time**: see below |

The choice is kept (also after a restart); `LIDAR_STATUS` shows what is on.

**Deciding print by print.** With `LIDAR_ASK_ON` (or `LIDAR_ASK ON=1 TIMEOUT=60`) every
print start opens a dialog in Fluidd/Mainsail with the file name and the buttons
**PA calibration + first layer**, **PA calibration only**, **First layer check only**,
**Print without LiDAR** and **Do not print**. With no answer within the time limit
(default 60 s, 10 to 600) the print starts with the saved settings above, e.g. when you
start from the display and Fluidd is not open. Closing the dialog with the X counts as
no answer. "PA calibration" uses the `LIDAR_FLOW` mode (1 if it is off). `LIDAR_CANCEL`
while the dialog is open = do not print. `LIDAR_ASK_OFF` removes the question. Needs
Fluidd 1.30+ or Mainsail 2.9+. Then start
your prints as usual — from Fluidd, Mainsail or the display. What happens:

1. Profile name, material and temperatures are read from the gcode (otherwise from
   `START_PRINT EXTRUDER_TEMP= BED_TEMP=`; `LIDAR_SET_MATERIAL` sets a fallback).
2. Saved PA for that profile and temperature → applied, no test pattern.
   Otherwise, with `LIDAR_FLOW` on, the plate is leveled, the test pattern is printed
   on the left edge, scanned, and the PA is saved and applied.
3. First layer check (if on): the layer is printed, the print pauses, the layer is
   scanned: **GOOD** → resumes; **BAD** → pause (or cancel); **UNCERTAIN** → always
   pause. The plate is leveled only once.
4. The print continues normally.

Keep the left edge of the plate clear whenever a calibration may run.

## All the buttons
| Button | |
|---|---|
| `LIDAR_HELP` | short guide in the console |
| `LIDAR_STATUS` | current settings (and whether the hook is installed) |
| `LIDAR_PA_CALIBRATE` | find the PA now: `GCODE=` or `FILAMENT=`, `MATERIAL=`, `NOZZLE=`, `BED=`, `FINE=1`, `START=`, `STEP=` |
| `LIDAR_PA_SET` | set a PA by hand: `PA=` and `GCODE=` or `FILAMENT=` + `NOZZLE=` |
| `LIDAR_PA_LIST` / `LIDAR_PA_FORGET` | list / delete saved profiles |
| `LIDAR_FLOW MODE=0/1/2` (`LIDAR_FLOW_ON` / `_OFF`) | calibration before a print |
| `LIDAR_FIRSTLAYER ON=0/1 ON_BAD=pause/cancel` (`_ON` / `_OFF`) | first layer check |
| `LIDAR_PA_APPLY ON=0/1` | apply the saved PA at print start |
| `LIDAR_ASK ON=0/1 TIMEOUT=60` (`LIDAR_ASK_ON` / `_OFF`) | ask before every print (dialog in Fluidd/Mainsail) |
| `LIDAR_SET_MATERIAL MATERIAL= HOTEND= BED=` | fallback when the gcode does not say |
| `LIDAR_CANCEL` | stop the running job; laser and heaters off |
| `LIDAR_LANGUAGE LANG=en/it` | language of the messages |

HTTP API for other tools: `GET /machine/k1lidar/status`, `GET /machine/k1lidar/filaments`,
`POST /machine/k1lidar/settings`, `POST /machine/k1lidar/pa`, `POST /machine/k1lidar/set`,
`POST /machine/k1lidar/print`, `POST /machine/k1lidar/cancel`.

## How the PA is measured
The test pattern is made of 4 columns of zig-zags; every **corner** of a zig-zag is
printed with a different PA (32 values). At a sharp corner the head brakes and
accelerates again: with too little PA the corner gets a blob, with too much the
extruder pulls back too hard and the corner becomes **thin or hollow**. For every
corner the laser measures the volume of the line at the corner compared with the
straight part of the same line (the *corner fill*). The fill decreases steadily as the
PA grows; the chosen PA is the one at which the corner is **as full as the straight
line**. The 32 corners are analysed together, a small known effect of the corner's
position in the row is removed, and a bootstrap gives the 80 % range.

Default range of the 32 values — chosen so that the expected value falls in the middle
of the pattern, because the first rows print slightly worse:

| Material | Range | Step |
|---|---|---|
| PLA, ABS, ASA | 0.006 – 0.0525 | 0.0015 |
| PETG | 0.016 – 0.078 | 0.002 |
| TPU (experimental) | 0.050 – 0.205 | 0.005 |

## Reading the result
- **confidence high**: use it.
- **confidence LOW**: the value is saved and applied, but the console lists why (noisy
  scan, weak trend, wide range). Repeat on a clean plate.
- **"PA outside the range of the pattern"**: the estimate falls outside the printed
  values, so it is not a measurement — nothing is saved. Repeat with the suggested
  `START=`.
- **Scan errors** (missing rows, empty scan) stop the calibration: nothing is saved.

## Safety
- Keep the left edge of the plate clear (X 0–30 mm, 60 mm with `FINE=1`) whenever a
  calibration may run.
- All travel moves of the test pattern are done at Z 5 mm.
- `LIDAR_CANCEL` stops a job and switches laser and heaters off; a job that fails
  switches the heaters off too.
- The hook is optional: if something goes wrong, `install.sh --no-hook` (or remove
  `[include k1lidar_hook.cfg]` from `printer.cfg`) and prints are stock again.
- Raw scans of every run are kept in `/usr/data/lidar_archive/` (useful when reporting
  a problem).

## How it works
- The laser module is driven directly over its serial port; Creality's middleware is
  only a fallback.
- Scans are one continuous linear pass, like the stock firmware. The position of every
  laser profile is taken from Klipper's own motion queue (`motion_report/dump_trapq`)
  on Klipper's clock, so each profile is placed within ~0.3 mm. The sensor delivers
  ~11 profiles per second; the pass speed adapts so there is one every ~0.6 mm.
- The PA analysis (`algo/pa_phys.py`) and the first layer analysis (`fl_analyze.py`:
  height map aligned to the gcode, missing/lifted areas) are part of this project.
  Creality's own PA analysis is **not** used: on the same filament it returned 0.054
  or 0.0375 depending on where the value fell in the pattern.

## Status and limits
- Validated on a K1 with PLA (220/60 °C): two independent calibrations of the same spool
  gave **0.0274** (80 % range 0.0230–0.0303) and **0.0270** (0.0233–0.0310). Creality's
  analysis on the same scans: 0.054, 0.0375, 0.036.
- The PETG/ABS/ASA ranges are reasoned starting values, TPU is experimental: please
  report your results.
- Only the K1 (CR-K1) is supported by the installer.

## Troubleshooting
| Symptom | What to do |
|---|---|
| a print does not use the measured PA | `LIDAR_PA_LIST`: is the profile name the same as in the slicer, and the temperature the same? Is `LIDAR_FLOW` on (otherwise the slicer's *Enable pressure advance* wins)? Is `LIDAR_STATUS` showing the hook installed? |
| `LiDAR: another operation is running` | wait, or `LIDAR_CANCEL` |
| buttons missing in Fluidd | run `install.sh` again; check `[include k1lidar.cfg]` in `printer.cfg` |
| Fluidd stops working after a firmware update | the update deleted `moonraker.conf`: run `install.sh` again |
| laser not found | `sh /usr/bin/laser_power.sh on`, then retry; check the module cable |
| result with LOW confidence | clean the plate, check the first rows printed well, repeat |

Not affiliated with Creality. Use at your own risk: it moves the printer, heats it and
prints. License: GPL-3.0.
