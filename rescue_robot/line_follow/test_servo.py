"""
test_servo.py
===============
Bare-minimum servo test using gpiozero (this is the approach confirmed
working on the robot -- gpiozero handles the pulse-width timing more
reliably than driving lgpio.tx_servo() directly).

servo.value runs from -1 to +1, and maps linearly across this servo's
full 0-270 degree range:
    -1  -> 0 degrees
     0  -> 135 degrees
    +1  -> 270 degrees

Usage:
    python3 line_follow/test_servo.py
"""

from gpiozero import Servo
from time import sleep

SERVO_PIN = 19

servo = Servo(SERVO_PIN, min_pulse_width=0.0005, max_pulse_width=0.0025)

while True:
    print("Moving to 0")
    servo.value = -1
    sleep(2)

    print("Moving to 135")
    servo.value = 0
    sleep(2)

    print("Moving to 270")
    servo.value = 1
    sleep(2)
