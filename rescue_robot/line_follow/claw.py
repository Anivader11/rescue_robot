"""
claw.py
========
Rear claw mechanism wrapped into one class, using the exact servo values
already confirmed in test_claw_open_close.py / test_claw_servo.py /
test_grab_ball.py.

    GPIO13 -> 9g claw open/close servo (positional)
    GPIO12 -> Parallax continuous-rotation lift servo (SPEED based, so
              "down"/"up" = spin that way for LIFT_DOWN_TIME_S / LIFT_UP_TIME_S, then stop)

Usage:
    claw = Claw()
    claw.open(); claw.lower(); ...; claw.close(); claw.lift()
    claw.release()     # on shutdown
"""

import time
from gpiozero import Servo

CLAW_LIFT_PIN = 12
CLAW_OPEN_CLOSE_PIN = 13

OPEN_VALUE = -1.0       # from test_claw_open_close.py (calibrated)
CLOSE_VALUE = 1.0

# Lift values copied from test_claw_servo.py (calibrated). Note that script
# leaves the servo running "down" through its sleep(DURATION_S) AND the
# following sleep(1), so the real calibrated down travel is 1.5 s at 0.8,
# and the up travel is 0.5 s at -1.0. Reproduced exactly here.
LIFT_DOWN_VALUE = 0.8
LIFT_UP_VALUE = -1.0
LIFT_STOP_VALUE = 0.0      # 1.5 ms pulse = Parallax CR servo stopped
LIFT_DOWN_TIME_S = 0.5 + 1.0
LIFT_UP_TIME_S = 0.5

OPEN_CLOSE_SETTLE_S = 0.5


class Claw:
    def __init__(self):
        self._grip = Servo(CLAW_OPEN_CLOSE_PIN, min_pulse_width=0.0005, max_pulse_width=0.0025)
        self._lift = Servo(CLAW_LIFT_PIN, min_pulse_width=0.0013, max_pulse_width=0.0017)
        self._lift.value = LIFT_STOP_VALUE

    def open(self):
        self._grip.value = OPEN_VALUE
        time.sleep(OPEN_CLOSE_SETTLE_S)

    def close(self):
        # Keep sending the CLOSE pulse (don't detach) so it keeps gripping.
        self._grip.value = CLOSE_VALUE
        time.sleep(OPEN_CLOSE_SETTLE_S)

    def lower(self, fraction=1.0):
        """fraction=1.0 is all the way down, 0.5 about halfway (timed)."""
        self._lift.value = LIFT_DOWN_VALUE
        time.sleep(LIFT_DOWN_TIME_S * fraction)
        self._lift.value = LIFT_STOP_VALUE

    def lift(self, fraction=1.0):
        self._lift.value = LIFT_UP_VALUE
        time.sleep(LIFT_UP_TIME_S * fraction)
        self._lift.value = LIFT_STOP_VALUE

    def release(self):
        self._lift.value = LIFT_STOP_VALUE
        time.sleep(0.05)
        self._lift.detach()
        self._grip.detach()
