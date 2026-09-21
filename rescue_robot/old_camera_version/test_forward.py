"""
test_forward.py
================
Minimal test: initialise all 4 motors via BLDCMotorDriver (same class
main.py uses — same PID constants, same calibration_offsets.json) and
drive straight forward for 15 seconds, then stop. No cameras, no state
machine.

BLDCMotorDriver no longer calibrates live — it only reads
calibration_offsets.json (written by calibrate_once.py). Run
calibrate_once.py first if you haven't already, or after any power loss
to the driver boards / after swapping a motor.

Run:
    python3 test_forward.py
"""

import time
import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

from control.motors import BLDCMotorDriver

FORWARD_SPEED = 50.0   # -100..100, tune as needed
RUN_SECONDS = 15.0
RAMP_STEPS = 20
RAMP_TIME_S = 1.0   # total time to ramp 0 -> FORWARD_SPEED


def main():
    log.info("Initialising motors (loading calibration_offsets.json, should be quick)...")
    motors = BLDCMotorDriver()

    try:
        # All 4 motors start from a dead stop simultaneously. Jumping
        # straight to full commanded speed on all 4 at once is a large
        # instantaneous current demand — if the power supply can't deliver
        # that all at once, some motors can brown out/stall while others
        # (lower resistance/better margin) keep going. Ramp up gradually
        # instead, same idea as identify_motors.py's ramp_speed(), to see if
        # that's what's actually happening here.
        log.info(f"Ramping to {FORWARD_SPEED} over {RAMP_TIME_S}s...")
        for i in range(1, RAMP_STEPS + 1):
            speed = FORWARD_SPEED * i / RAMP_STEPS
            motors.drive_straight(speed)
            time.sleep(RAMP_TIME_S / RAMP_STEPS)

        log.info(f"At full speed, holding for {RUN_SECONDS}s...")
        time.sleep(RUN_SECONDS)
    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        motors.stop()
        log.info("Motors stopped")
        if hasattr(motors, "cleanup"):
            motors.cleanup()
        log.info("Done")


if __name__ == "__main__":
    main()
