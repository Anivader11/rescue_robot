import time
import lgpio

# Raspberry Pi BCM GPIO numbers
SENSOR_PINS = [24, 23]

# Raspberry Pi GPIO chip
GPIO_CHIP = 0

# Polling rate
POLL_HZ = 20
POLL_SLEEP = 1.0 / POLL_HZ

# Open GPIO chip
h = lgpio.gpiochip_open(GPIO_CHIP)
EN_PIN = 22

lgpio.gpio_claim_output(h, EN_PIN, 1)  # EN HIGH = sensor enabled
# Configure sensor outputs as inputs with an internal pull-up.
# The Parallax sensor outputs:
#   WHITE -> LOW (0)
#   BLACK -> HIGH (1)
for pin in SENSOR_PINS:
    lgpio.gpio_claim_input(h, pin, lgpio.SET_PULL_UP)

try:
    while True:
        readings = []

        for pin in SENSOR_PINS:
            readings.append(lgpio.gpio_read(h, pin))

        print(*readings)

        time.sleep(POLL_SLEEP)
except KeyboardInterrupt:
    print("\nStopping...")

finally:
    lgpio.gpiochip_close(h)