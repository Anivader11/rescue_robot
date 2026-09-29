"""
camera_servo.py
==================
Controls the 9g camera-pan servo using gpiozero's Servo class -- this is
the approach confirmed working on the robot (see test_servo.py), so the
real code now matches it instead of driving lgpio.tx_servo() directly.

servo.value runs from -1 to +1, mapped linearly across this servo's full
0-270 degree range:
    -1  -> 0 degrees
     0  -> 135 degrees
    +1  -> 270 degrees
"""

from gpiozero import Servo
import time

SERVO_PIN = 19

MOVE_SETTLE_S = 0.3   # how long to wait after a move before considering it "done"

# Tune these two if you want look_forward()/look_up() to land on different
# angles -- pick any value from -1 (0 deg) to +1 (270 deg).
FORWARD_VALUE = 0.0    # 135 degrees -- straight ahead
LOOK_UP_VALUE = 0.6    # ~216 degrees -- tilted up toward the evacuation zone


class CameraServo:
    def __init__(self, pin=SERVO_PIN):
        self._servo = Servo(pin, min_pulse_width=0.0005, max_pulse_width=0.0025)

    def set_value(self, value):
        """value: -1 (0 deg) to +1 (270 deg), 0 = 135 deg (straight ahead)."""
        value = max(-1.0, min(1.0, value))
        self._servo.value = value
        time.sleep(MOVE_SETTLE_S)

    def look_forward(self):
        self.set_value(FORWARD_VALUE)

    def look_up(self):
        self.set_value(LOOK_UP_VALUE)

    def release(self):
        self._servo.detach()

    def close(self):
        self.release()


if __name__ == "__main__":
    cam = CameraServo()
    try:
        while True:
            print("Looking forward...")
            cam.look_forward()
            time.sleep(1)

            print("Looking up...")
            cam.look_up()
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        cam.close()
