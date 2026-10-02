"""
test_endzone.py
================
Run the evacuation-zone logic on its own -- put the robot just inside the
zone (or on the silver tape) and run:

    python3 line_follow/test_endzone.py        # one ball (stage 1 test)
    python3 line_follow/test_endzone.py 3      # all three

Tune thresholds in endzone.py / zone_vision.py. To check detection only
(no driving), run: python3 line_follow/zone_vision.py
"""

import sys
import os
import time
import logging

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control.motors import BLDCMotorDriver
from camera_watcher import CameraWatcher
from tof_detector import ToFDetector
from endzone import run_endzone


def main():
    logging.basicConfig(level=logging.INFO)
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    motors = BLDCMotorDriver()
    cam = CameraWatcher(); cam.start()
    tof = ToFDetector(); tof.start()
    time.sleep(1.5)   # let camera + ToF produce first readings
    try:
        run_endzone(motors, cam, tof, max_balls=n)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        motors.stop()
        cam.stop()
        tof.stop()


if __name__ == "__main__":
    main()
