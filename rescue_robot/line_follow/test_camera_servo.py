"""
test_camera_servo.py
=======================
Bare-minimum test for the camera-pan servo: goes to "home" (looking
forward), then rotates down, then back to home. Uses gpiozero's Servo
class, same confirmed-working approach as the claw servos.

servo.value runs from -1 to +1, mapped linearly across this servo's full
0-270 degree range:
    -1  -> 0 degrees
     0  -> 135 degrees
    +1  -> 270 degrees

HOME_VALUE and DOWN_VALUE below are starting guesses -- adjust them
until the servo actually lands on your real "forward" and "90 degrees
down" positions, the same way you dialed in the claw servo values.

Usage:
    python3 line_follow/test_camera_servo.py
"""

from gpiozero import Servo
from time import sleep

SERVO_PIN = 19

servo = Servo(SERVO_PIN, min_pulse_width=0.0005, max_pulse_width=0.0025)
# <-- tune until this is "forward"/home
UP_VALUE = 0.75   # <-- tune until this is exactly 90 degrees down from home
HOLD_S = 2.0



print("Moving up 90")
servo.value = UP_VALUE
sleep(HOLD_S)


servo.detach()   # stop sending pulses altogether