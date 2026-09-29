import lgpio
import time

EN_PIN = 22     # BCM22 / physical pin 15
LEFT_PIN = 24   # BCM24 / physical pin 18
RIGHT_PIN = 23  # BCM23 / physical pin 16

h = lgpio.gpiochip_open(1)
lgpio.gpio_claim_output(h, EN_PIN, 1)
time.sleep(0.2)  # let the sensor's output settle after EN goes HIGH, before reading it

# Pull-down added: if the sensor's OUT pin only actively drives HIGH (black)
# and leaves the line floating/high-impedance for LOW (white) instead of
# properly driving both states, an unconnected Pi input with no pull
# resistor will read random noise instead of a clean 0. A pull-down gives
# the pin a defined rest state so "white" reads as a clean 0 even if the
# sensor itself isn't driving it.
lgpio.gpio_claim_input(h, LEFT_PIN, lgpio.SET_PULL_DOWN)
lgpio.gpio_claim_input(h, RIGHT_PIN, lgpio.SET_PULL_DOWN)
lgpio.gpio_set_debounce_micros(h, LEFT_PIN, 0)
lgpio.gpio_set_debounce_micros(h, RIGHT_PIN, 0)

while True:
    print(lgpio.gpio_read(h, LEFT_PIN), lgpio.gpio_read(h, RIGHT_PIN))
    time.sleep(0.05)
