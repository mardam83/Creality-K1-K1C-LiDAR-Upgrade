#!/usr/bin/env python3
# lidar_job.py - punto d'ingresso dei job LiDAR (avvio stampa, taratura a pulsante).
# Lo lancia il componente Moonraker; si puo' usare anche a mano. Vedi k1lidar/job.py.
import os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from k1lidar.job import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
