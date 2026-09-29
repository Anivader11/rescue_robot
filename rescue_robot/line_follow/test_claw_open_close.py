"""
test_claw_open_close.py
==========================
Bare-minimum test for the 9g claw open/close servo (NOT the Parallax
claw-lift servo -- that one is test_claw_servo.py). Uses gpiozero's
Servo class, same confirmed-working approach as test_servo.py.

servo.value runs from -1 to +1, mapped linearly across this servo's full
0-270 degree range:
    -1  -> 0 degrees   (treated as fully OPEN below)
    +1  -> 270 degrees (treated as fully CLOSED below)

Edit OPEN_VALUE / CLOSE_VALUE below, rerun, and see whether the claw
actually reaches fully open and fully closed without straining.

Usage:
    python3 line_follow/test_claw_open_close.py
"""

from gpiozero import Servo
from time import sleep

CLAW_SERVO_PIN = 13

servo = Servo(CLAW_SERVO_PIN, min_pulse_width=0.0005, max_pulse_width=0.0025)

OPEN_VALUE = -1.0    # 0 degrees -- tune this until the claw is fully open
CLOSE_VALUE = 1.0    # 270 degrees -- tune this until the claw is fully closed
HOLD_S = 2.0         # how long to hold each position before moving to the next

while True:
    print("Closing claw")
    servo.value = CLOSE_VALUE
    sleep(HOLD_S)

    
    print("Opening claw")
    servo.value = OPEN_VALUE
    sleep(HOLD_S)

