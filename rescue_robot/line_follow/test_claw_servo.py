"""
test_claw_servo.py
=====================
Bare-minimum test for the Parallax Continuous Rotation Servo driving the
claw up/down. Uses gpiozero's Servo class -- same confirmed-working
approach as test_servo.py -- since raw lgpio.tx_servo() pulses weren't
being read reliably by this servo.

Unlike the 9g positional servos, this one does NOT hold an angle -- the
value controls SPEED and DIRECTION, continuously, for as long as it's
set:
    servo.value = -1   -> full speed one way   ("up")
    servo.value =  0   -> stop
    servo.value = +1   -> full speed the other way ("down")
    anything in between -> proportional speed

CALIBRATION: Parallax's own docs say unit-to-unit variation in the
servo's internal electronics means value=0 doesn't always land on a
perfect stop out of the box -- there's a small trim potentiometer on the
side of the servo for this. To calibrate: hold servo.value = 0 for a
while and slowly turn the trim pot with a small screwdriver until the
servo stops drifting. Do this once before relying on value=0 as "stop"
elsewhere in the code.

Usage:
    python3 line_follow/test_claw_servo.py
"""

from gpiozero import Servo
from time import sleep, time

SERVO_PIN = 12   # Parallax claw-LIFT servo (camera=19, claw open/close=13)

servo = Servo(SERVO_PIN, min_pulse_width=0.0013, max_pulse_width=0.0017)

DOWN_VALUE = 0.8      # full speed "down" -- flipped from -1, servo was spinning backwards
UP_VALUE = -1.0   # full speed "up" -- flipped from +1
DURATION_S = 0.5    # <-- change this to see more/less travel, then rerun

print("Moving down")
servo.value = DOWN_VALUE
sleep(DURATION_S)

sleep(1)   # let the servo stop before reversing direction

print("Moving up")
servo.value = UP_VALUE
sleep(DURATION_S)



print("Stopping")
servo.value = 0
sleep(0.1)

servo.detach()   # stop sending pulses altogether
print("Done.")
