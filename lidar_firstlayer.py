#!/usr/bin/env python3
# lidar_firstlayer.py - controllo del primo strato col LiDAR (riga di comando).
# La logica sta in k1lidar/firstlayer.py; questo file resta per compatibilita'
# (macro, Moonraker, test) e riesporta le sue funzioni.
#
#   python3 lidar_firstlayer.py --gcode FILE.gcode --factory-level --resume [--on-bad cancel]
import os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from k1lidar.firstlayer import *  # noqa: F401,F403,E402
from k1lidar.firstlayer import main  # noqa: E402

if __name__ == "__main__":
    main()
