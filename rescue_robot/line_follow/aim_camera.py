"""
aim_camera.py
==============
Find the best camera-servo angle. Moves the camera servo (GPIO19) to a value
you type, HOLDS it there (unlike test_camera_servo.py, which detaches the
servo at the end so the camera can sag back), waits for a fresh frame and
saves it to ~/cam_check.jpg so you can copy it to the laptop and look.

servo value: -1 .. +1  (0 = 135 deg). Try steps of 0.1 and see which
direction tilts the camera forward.

Usage (on the Pi):
    python3 line_follow/aim_camera.py
    > 0.2        <- type a value, Enter
    > 0.3
    > q          <- quit (servo released)

On the laptop, after each value:
    scp nigesh@rescue.local:~/cam_check.jpg . ; start cam_check.jpg
"""

import os
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_watcher import CameraWatcher
from camera_servo import CameraServo

OUT = os.path.expanduser("~/cam_check.jpg")


def main():
    cam = CameraWatcher()
    cam.start()
    servo = CameraServo()
    value = 0.0
    print(__doc__.split("Usage")[0])
    try:
        while True:
            txt = input(f"servo value (now {value:+.2f}), or q: ").strip().lower()
            if txt in ("q", "quit", "exit"):
                break
            try:
                value = max(-1.0, min(1.0, float(txt)))
            except ValueError:
                print("  type a number between -1 and 1")
                continue
            servo.set_value(value)          # moves and waits for it to settle
            time.sleep(0.5)                 # let the camera catch up
            frame, _ = cam.get_frame()
            if frame is None:
                print("  no camera frame yet -- try again")
                continue
            cv2.imwrite(OUT, frame)
            print(f"  saved {OUT} at servo value {value:+.2f}  (servo is holding this position)")
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\nLast value: {value:+.2f}  <- tell Claude this number")
        servo.release()
        cam.stop()


if __name__ == "__main__":
    main()
