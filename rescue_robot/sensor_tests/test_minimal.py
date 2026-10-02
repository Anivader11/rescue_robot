import os
os.environ.setdefault("GPIOZERO_PIN_FACTORY", "lgpio")
import time
import lgpio
from gpiozero import Device, DigitalInputDevice, DigitalOutputDevice

SENSOR_PINS = [23, 24]
EN_PIN = 22 

POLL_HZ = 20
POLL_SLEEP = 1.0 / POLL_HZ

h = lgpio.gpiochip_open(0)



while True:
    for pin in SENSOR_PINS:
        print(lgpio.gpio_read(h, pin), end=" ")
    print()
    time.sleep(0.1)
