"""
test_28034_line_sensor.py
==========================
Quick test script for the Parallax 28034 IR Line Follower array on a
Raspberry Pi 5.

Pin table, straight from the Parallax 28034 datasheet (11-pin header,
pins numbered 1-11 in order):
    Pin 1  = GND
    Pin 2  = Vdd (supply voltage)
    Pin 3  = EN  (enable -- pull LOW for low-power/disabled mode; HIGH
                  or left floating high = normal operation)
    Pin 4  = OUT0
    Pin 5  = OUT1
    Pin 6  = OUT2
    Pin 7  = OUT3
    Pin 8  = OUT4
    Pin 9  = OUT5
    Pin 10 = OUT6
    Pin 11 = OUT7

Logic level, quoted directly from the datasheet:
    "When the IR LED is over a white surface, light is reflected to
    the IR receiver, and its output is low. When the IR LED is over a
    black surface, no light is reflected to the IR receiver, and its
    output is high."
    So: HIGH = black, LOW = white. This script already matches that.

Confirmed on real hardware: the sensor's own onboard LED lights up
correctly over white and goes off over black on its own, with no code
involved -- so the board is a normal push-pull output, actively
driving the line HIGH or LOW itself. It does NOT need a pull-up
resistor.

This matters because gpiozero's DigitalInputDevice has a specific
gotcha: pull_up=True does not just enable a pull-up resistor, it also
INVERTS the .value property (it's built around the assumption of a
button wired to ground, where pulling the pin LOW should read as
"active"/True). With pull_up=True, a pin that is genuinely HIGH reads
back as 0, and a genuinely LOW pin reads back as 1 -- backwards. That
inversion was the actual bug in an earlier version of this script.
Since the sensor drives its own output and needs no pull-up, this
version uses pull_up=False, which reads the pin's real, non-inverted
state.

Wiring:
    - Board Pin 1 (GND) -> Pi physical pin 9  (GND)
    - Board Pin 2 (Vdd) -> Pi physical pin 17 (3.3V)
    - Board Pin 3 (EN)  -> Pi GPIO5 (physical pin 29) -- this script
      drives that pin HIGH at startup to enable the array.
    - Board OUT0..OUTn  -> one Pi GPIO pin each (BCM numbering), listed
      in SENSOR_PINS below, in the SAME left-to-right order as
      OUT0, OUT1, OUT2... on the board -- double check this against
      your actual wiring, a swapped order will just mislabel which
      sensor is which, not break the readings themselves.

Only wire up as many OUT pins as you actually have sensors connected
right now -- edit SENSOR_PINS to match.
"""

import os
# Force gpiozero onto the lgpio backend, which is the backend that
# actually talks to the Raspberry Pi 5's RP1 GPIO chip correctly. This
# project already hit RP1-specific flakiness on the I2C side (the BLDC
# motor driver errors earlier in the build) -- gpiozero silently
# defaulting to the wrong pin factory on a Pi 5 is a known way to get
# GPIO reads that never change no matter what's on the pin. Set this
# BEFORE importing gpiozero.
os.environ.setdefault("GPIOZERO_PIN_FACTORY", "lgpio")

import time
from gpiozero import Device, DigitalInputDevice, DigitalOutputDevice

# ---------------------------------------------------------------------
# EDIT THIS: BCM pin numbers for each OUT pin you wired up, in the same
# order as OUT0, OUT1, OUT2... on the board.
# ---------------------------------------------------------------------
SENSOR_PINS = [23, 27]   # BCM pin numbers, one per OUT pin used
EN_PIN = 22               # BCM pin for the board's EN line -- BCM22 = physical pin 15

POLL_HZ = 20   # how many times per second to print a reading
POLL_SLEEP = 1.0 / POLL_HZ


def main():
    print(f"gpiozero pin factory in use: {Device.pin_factory!r}")
    en = DigitalOutputDevice(EN_PIN, initial_value=True)  # HIGH = enabled,
    # per the datasheet: "pull low for low-power mode" -- initial_value
    # MUST be True here, False disables the whole array.
    # pull_up=False -- the sensor drives its own output (confirmed by its
    # onboard LED reacting correctly with no code involved), so no pull-up
    # is needed, and pull_up=True would invert the reading (see docstring).
    sensors = [DigitalInputDevice(pin, pull_up=False) for pin in SENSOR_PINS]

    print(f"EN pin (BCM {EN_PIN}) driven HIGH -- array enabled")
    print(f"Reading {len(sensors)} sensor(s) on BCM pins {SENSOR_PINS}, pull_up=False")
    print("HIGH(1) = black line detected, LOW(0) = white/no line. Ctrl-C to stop.\n")

    try:
        while True:
            readings = [s.value for s in sensors]   # 1 = HIGH (black), 0 = LOW (white)
            parts = []
            for i, val in enumerate(readings):
                state = "BLACK" if val else "white"
                parts.append(f"S{i}(pin{SENSOR_PINS[i]})={state}(raw={val})")
            print("  ".join(parts))
            time.sleep(POLL_SLEEP)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        for s in sensors:
            s.close()
        en.close()


if __name__ == "__main__":
    main()
