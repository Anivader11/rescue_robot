from gpiozero import Servo
from time import sleep

servo = Servo(19, min_pulse_width=0.0005, max_pulse_width=0.0025)

while True:
    print ("Moving to 0")
    servo.value = -1
    sleep(2)

    print ("Moving to 135")
    servo.value = 0
    sleep(2)

    print ("Moving to 270")
    servo.value = 1
    sleep(2)