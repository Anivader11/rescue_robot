"""
test_turn.py
============
Sanity-check script for in-place turning, using the SAME motor driver
main.py uses (control.motors.BLDCMotorDriver) — not a standalone I2C
script like test_forwardv2.py. This exercises the exact code path
main.py/state_machine.py will use for MARKER_TURN / LOST-search rotation,
so a pass here means turning will behave correctly under the real state
machine too.

Requires calibration_offsets.json to exist (see control/motors.py /
calibrate_once.py) — same requirement as running main.py without --stub.

Run:
    python3 test_turn.py

Watch the robot from above. It should:
    1. Rotate LEFT (counter-clockwise, viewed from above) for TURN_TIME
    2. Pause
    3. Rotate RIGHT (clockwise, viewed from above) for TURN_TIME
    4. Stop

If left/right come out swapped or the robot drives instead of spinning,
that means LEFT_INVERTED/RIGHT_INVERTED in control/motors.py need
flipping — do NOT touch the sign convention proven by test_forwardv2.py
(right side positive / left side negative = forward) to fix this; only
adjust the rotate_left/rotate_right split if it's actually backwards.
"""

import time
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(message)s",
                     datefmt="%H:%M:%S")

from control.motors import BLDCMotorDriver

TURN_SPEED = 25.0   # percent, keep low for a first test — matches
                     # SEARCH_ROTATE_SPEED / MARKER_TURN_SPEED order of
                     # magnitude in control/state_machine.py
TURN_TIME  = 2.0     # seconds per direction
PAUSE_TIME = 1.0     # seconds between the two turns

def main():
    motors = BLDCMotorDriver()
    try:
        print(f"Rotating LEFT at {TURN_SPEED}% for {TURN_TIME}s...")
        end = time.time() + TURN_TIME
        while time.time() < end:
            motors.rotate_left(TURN_SPEED)
            time.sleep(0.02)
        motors.stop()

        print(f"Pausing {PAUSE_TIME}s...")
        time.sleep(PAUSE_TIME)

        print(f"Rotating RIGHT at {TURN_SPEED}% for {TURN_TIME}s...")
        end = time.time() + TURN_TIME
        while time.time() < end:
            motors.rotate_right(TURN_SPEED)
            time.sleep(0.02)
        motors.stop()

        print("Done.")
    finally:
        motors.stop()
        motors.cleanup()

if __name__ == "__main__":
    main()
