"""
test_tof_distance.py
=======================
Bare-minimum distance readout for the SteelBar ToF sensor (tof_sensor.py) --
same idea as the Adafruit VL53L4CD example you found, just using the
sensor we actually have on this robot (SteelBar, I2C address 0x51, not
a VL53L4CD -- different chip, but same job: point it at something and
read a distance back).

Use this tomorrow to find the exact distance-from-ball reading where
you want the "stop and spin around" behavior to trigger -- put the ball
at different distances in front of the sensor and watch the printed
number.

Usage:
    python3 line_follow/test_tof_distance.py
"""

import time
from tof_sensor import SteelBarToF

sensor = SteelBarToF()

print("Reading ToF distance. Ctrl-C to stop.")
try:
    while True:
        distance_mm = sensor.current_measurement()
        print(f"Distance: {distance_mm} mm")
        time.sleep(0.2)
except KeyboardInterrupt:
    print("\nStopped.")
