"""
test_grab_ball.py
====================
Trial run of the full "grab the ball" mechanism: drive motors + claw
open/close servo + claw lift servo, all in one sequence. Uses the exact
values you already confirmed working in test_claw_servo.py and
test_claw_open_close.py -- this just chains them together with the
drive motors in between.

Sequence:
    1. Open the claw (same open/close logic as test_claw_open_close.py)
    2. Reverse toward the ball (tune REVERSE_DURATION_S by eye)
    3. Close the claw (grab)
    4. Lift the claw up (same down-then-up logic as test_claw_servo.py)
    5. Stop everything

Usage:
    python3 line_follow/test_grab_ball.py
"""

import sys
import os
import time
from time import sleep

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gpiozero import Servo
from control.motors import BLDCMotorDriver

# --- Pins ---
CLAW_LIFT_PIN = 12       # Parallax continuous-rotation claw-lift servo
CLAW_OPEN_CLOSE_PIN = 13  # 9g claw open/close servo

# --- Claw open/close servo values (confirmed in test_claw_open_close.py) ---
OPEN_VALUE = -1.0
CLOSE_VALUE = 1.0


# --- Claw lift servo values (confirmed in test_claw_servo.py) ---
DOWN_VALUE = 1      # full speed "down"
UP_VALUE = -1.0        # full speed "up"
      # how long to run the lift servo in each direction

# --- Drive motors ---
REVERSE_SPEED = 30.0        # % speed while backing up toward the ball
REVERSE_DURATION_S = 1.0    # how long to reverse -- tune this to match the ball's distance


def main():
    motors = BLDCMotorDriver()
    claw_open_close = Servo(CLAW_OPEN_CLOSE_PIN, min_pulse_width=0.0005, max_pulse_width=0.0025)
    claw_lift = Servo(CLAW_LIFT_PIN, min_pulse_width=0.0013, max_pulse_width=0.0017)

    try:
        print("Opening claw")
        claw_open_close.value = OPEN_VALUE
        sleep(0.5)

        print("Moving down")
        claw_lift.value = DOWN_VALUE

        sleep(1)

        print("Closing claw")
        claw_open_close.value = CLOSE_VALUE

        sleep(0.5)
        
        print("Moving up")
        claw_lift.value = UP_VALUE
        
        sleep(1)   # let the servo stop before reversing direction
            
        print("Done -- check whether the ball is held and lifted.")
    finally:
        motors.stop()
        claw_open_close.detach()
        claw_lift.detach()


if __name__ == "__main__":
    main()
