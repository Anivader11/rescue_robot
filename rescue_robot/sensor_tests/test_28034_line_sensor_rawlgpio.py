"""
test_28034_line_sensor_rawlgpio.py
====================================
Same test as test_28034_line_sensor.py, but reads the GPIO pins directly
through the lgpio library instead of going through gpiozero.

Why: on the last run, the sensor's own onboard LED was confirmed to be
correctly toggling on/off with black vs white, but the Python readout
still got stuck reporting "white" after a while. That means the receiver
electronics are fine and the bug is specifically in how the previous
script was reading the pin. gpiozero sits on top of lgpio as an extra
layer of abstraction (event threads, cached state, debounce handling) --
this script removes that layer and just asks the RP1 chip directly, on
every single loop iteration, what voltage each pin currently reads, with
nothing cached in between. If this version stays accurate where the
gpiozero version got stuck, that confirms the bug was in gpiozero's
abstraction layer on this Pi 5, not in your wiring or the sensor.

Pin table (same wiring as before):
    EN   -> BCM22 (physical pin 15) -- driven HIGH to enable the array
    OUT0 -> BCM23 (physical pin 16)
    OUT1 -> BCM24 (physical pin 18)
"""

import time
import lgpio

EN_PIN = 22
SENSOR_PINS = [23, 24]
POLL_HZ = 20
POLL_SLEEP = 1.0 / POLL_HZ


def main():
    h = lgpio.gpiochip_open(0)   # open the Pi 5's main GPIO chip fresh

    lgpio.gpio_claim_output(h, EN_PIN)
    lgpio.gpio_write(h, EN_PIN, 1)   # EN HIGH = enabled

    for pin in SENSOR_PINS:
        lgpio.gpio_claim_input(h, pin)   # no pull-up/pull-down -- sensor drives its own line
        # RP1 (the Pi 5's GPIO chip) applies input debounce/glitch filtering
        # by default, and there are known reports of a pin latching a stale
        # value after certain rapid transitions until the debounce state is
        # explicitly reset. Force debounce to 0 so nothing is filtered or
        # cached at the driver level -- every read reflects the pin's
        # current raw electrical state, full stop.
        lgpio.gpio_set_debounce_micros(h, pin, 0)

    print(f"lgpio chip opened, handle={h}")
    print(f"EN pin (BCM {EN_PIN}) driven HIGH -- array enabled")
    print(f"Reading {len(SENSOR_PINS)} sensor(s) on BCM pins {SENSOR_PINS} via raw lgpio")
    print("HIGH(1) = black line detected, LOW(0) = white/no line. Ctrl-C to stop.\n")

    loop_count = 0
    try:
        while True:
            parts = []
            for i, pin in enumerate(SENSOR_PINS):
                val = lgpio.gpio_read(h, pin)   # fresh read, every loop, no caching
                state = "BLACK" if val else "white"
                parts.append(f"S{i}(pin{pin})={state}(raw={val})")
            print("  ".join(parts))
            loop_count += 1
            if loop_count % (POLL_HZ * 3) == 0:
                # Every ~3 seconds, fully release and re-claim each input
                # line. If the RP1 driver is latching a stale debounce/edge
                # state internally, this forces a clean reset of that state
                # without restarting the whole script.
                for pin in SENSOR_PINS:
                    lgpio.gpio_free(h, pin)
                for pin in SENSOR_PINS:
                    lgpio.gpio_claim_input(h, pin)
                    lgpio.gpio_set_debounce_micros(h, pin, 0)
            time.sleep(POLL_SLEEP)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        lgpio.gpio_write(h, EN_PIN, 0)
        lgpio.gpiochip_close(h)


if __name__ == "__main__":
    main()
