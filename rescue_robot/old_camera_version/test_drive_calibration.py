"""
test_drive_calibration.py
==========================
Measures real-world robot speed so the timing constants in
control/state_machine.py (OBSTACLE_ARC_DURATION_S / OBSTACLE_ARC_MIN_S,
MARKER_TURN_DURATION_S, U_TURN_DURATION_S, LOST_SEARCH_TIMEOUT_S) can be
computed from actual cm/s and deg/s instead of guessed — same idea as
calibrate_obstacle.py, but for motion instead of vision.

Two tests, both using the real BLDCMotorDriver (control/motors.py):
  1. STRAIGHT — drives forward for DRIVE_TIME seconds at DRIVE_PCT. You
     measure the actual distance travelled (tape measure) and enter it.
  2. ROTATE — spins in place for TURN_TIME seconds at TURN_PCT. You
     measure the actual rotation (mark the floor / use a protractor) and
     enter it.

Prints cm/s and deg/s at the end — send those back so the durations in
state_machine.py can be set from real geometry instead of a placeholder.

Run:
    python3 test_drive_calibration.py
"""

import time
from control.motors import BLDCMotorDriver

DRIVE_TIME = 2.0
TURN_TIME  = 2.0
DRIVE_PCT  = 100.0   # matches test_forwardv2.py's proven DRIVE_SPEED (100% = MAX_SPEED_UNITS)
TURN_PCT   = 25.0    # matches test_turn.py's TURN_SPEED


def timed_run(motors, action, seconds):
    end = time.time() + seconds
    while time.time() < end:
        action()
        time.sleep(0.02)
    motors.stop()


def main():
    motors = BLDCMotorDriver()
    try:
        input(f"\n[STRAIGHT] Clear space ahead, mark the robot's current "
              f"position, then press ENTER to drive forward at "
              f"{DRIVE_PCT:.0f}% for {DRIVE_TIME:.1f}s...")
        timed_run(motors, lambda: motors.drive_straight(DRIVE_PCT), DRIVE_TIME)
        dist_cm = float(input("Measured distance travelled, in cm: "))
        speed_cms = dist_cm / DRIVE_TIME
        print(f"  -> {speed_cms:.1f} cm/s at {DRIVE_PCT:.0f}% speed")

        time.sleep(1.0)

        input(f"\n[ROTATE] Mark the robot's current heading, then press "
              f"ENTER to rotate right at {TURN_PCT:.0f}% for "
              f"{TURN_TIME:.1f}s...")
        timed_run(motors, lambda: motors.rotate_right(TURN_PCT), TURN_TIME)
        deg = float(input("Measured rotation, in degrees: "))
        rate_dps = deg / TURN_TIME
        print(f"  -> {rate_dps:.1f} deg/s at {TURN_PCT:.0f}% turn speed")

        print("\n" + "=" * 50)
        print(f"STRAIGHT: {speed_cms:.1f} cm/s  @ {DRIVE_PCT:.0f}% (drive_straight)")
        print(f"ROTATE:   {rate_dps:.1f} deg/s  @ {TURN_PCT:.0f}% (rotate_right)")
        print("=" * 50)
        print("Send both numbers back — OBSTACLE_ARC_DURATION_S/MIN_S, "
              "MARKER_TURN_DURATION_S, U_TURN_DURATION_S and "
              "LOST_SEARCH_TIMEOUT_S in control/state_machine.py can be "
              "computed from these instead of left as guesses.")
    finally:
        motors.stop()
        motors.cleanup()


if __name__ == "__main__":
    main()
