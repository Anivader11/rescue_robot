"""
test_line8.py
==============
Live check of all 8 outputs on the Parallax #28034 line sensor array.
Same confirmed setup as test_line.py: EN (BCM22) driven HIGH, every OUT
pin claimed with lgpio.SET_PULL_UP. WHITE -> 0, BLACK -> 1.

Prints one line per reading, e.g.
    OUT0..7  . . # # . . . .   pos=-1.0
'#' = black, '.' = white. pos runs -3.5 (OUT0 side) .. +3.5 (OUT7 side).

Checks to do:
  1. Slide a strip of black tape slowly across under the array: the '#'
     should move smoothly from one end to the other, with no gaps and
     no sensor that never changes (= loose/wrong wire).
  2. Note which side OUT0 is on (robot's LEFT or RIGHT) -- the line-follow
     code needs to know.

Usage:
    python3 sensor_tests/test_line8.py
"""

import time
import lgpio

EN_PIN = 22
#            OUT0 OUT1 OUT2 OUT3 OUT4 OUT5 OUT6 OUT7
SENSOR_PINS = [5,   6,   24,  16,  17,  23,  25,  26]
WEIGHTS = [-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5]

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, EN_PIN, 1)
for pin in SENSOR_PINS:
    lgpio.gpio_claim_input(h, pin, lgpio.SET_PULL_UP)

print("OUT0..OUT7 on BCM", SENSOR_PINS, "-- Ctrl-C to stop\n")
try:
    while True:
        vals = [lgpio.gpio_read(h, p) for p in SENSOR_PINS]
        black = [w for w, v in zip(WEIGHTS, vals) if v]
        pos = f"{sum(black) / len(black):+.1f}" if black else "lost"
        bar = " ".join("#" if v else "." for v in vals)
        print(f"OUT0..7  {bar}   pos={pos}")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    lgpio.gpio_write(h, EN_PIN, 0)
    lgpio.gpiochip_close(h)
