"""
pid_line_follow_only.py
==========================
Bare-bones PID black line following -- ONLY the line following part, no
camera, no ToF obstacle evasion, no IMU ramp boost. Useful for testing/
tuning the core line-following behaviour on its own, or as a simpler
starting point to build back up from.

Sensor power: the line sensor board has an EN (enable) pin -- BCM22 --
that must be driven HIGH before the two OUT pins will report anything
real. Without this, the sensors are simply unpowered.

Sensor pins are claimed with an internal pull-up (lgpio.SET_PULL_UP) --
this is the exact setup confirmed working in sensor_tests/test_line.py.
WHITE -> LOW (0), BLACK -> HIGH (1).

Sensor spacing: the two sensors sit far enough apart that when the robot
is centered on the line, BOTH see white (0, 0) -- the line runs between
them, not under either one. Only one sensor reads black (1) when the
robot has drifted enough for that sensor to catch the line's edge. So
(0,0) and (1,1) both mean "centered, drive straight" (error 0) -- no
separate "line lost" failsafe needed, the plain PID math already does
the right thing for every sensor combination.

Usage:
    python3 line_follow/pid_line_follow_only.py
"""

import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import lgpio
from control.motors import BLDCMotorDriver

EN_PIN = 22      # line sensor array's enable line -- must be driven HIGH or the sensors don't power up at all
LEFT_PIN = 24
RIGHT_PIN = 23

BASE_SPEED = 30.0
Kp = 20.0
Kd = 10.0

LOOP_HZ = 50
LOOP_SLEEP = 1.0 / LOOP_HZ


def main():
    h = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_output(h, EN_PIN, 1)   # EN HIGH -- enable the sensor array before we try reading it
    # Internal pull-up on both sensor pins -- confirmed working in
    # sensor_tests/test_line.py. WHITE -> LOW (0), BLACK -> HIGH (1).
    lgpio.gpio_claim_input(h, LEFT_PIN, lgpio.SET_PULL_UP)
    lgpio.gpio_claim_input(h, RIGHT_PIN, lgpio.SET_PULL_UP)

    motors = BLDCMotorDriver()

    last_error = 0.0

    print("Starting plain PID line following. Ctrl-C to stop.")
    try:
        while True:
            left_val = lgpio.gpio_read(h, LEFT_PIN)
            right_val = lgpio.gpio_read(h, RIGHT_PIN)

            error = float(left_val - right_val)
            correction = Kp * error + Kd * (error - last_error)
            last_error = error

            left_speed = BASE_SPEED + correction
            right_speed = BASE_SPEED - correction
            motors.set_speeds(left_speed, right_speed)

            time.sleep(LOOP_SLEEP)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        motors.stop()
        lgpio.gpio_write(h, EN_PIN, 0)   # EN LOW -- power the sensor array back down on exit
        lgpio.gpiochip_close(h)


if __name__ == "__main__":
    main()
