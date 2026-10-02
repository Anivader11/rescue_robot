"""
calibrate_servo.py
====================
Interactive calibration tool for the generic 9g (SG90-clone) servo that
swivels the camera up once bottle_detector.py confirms an obstacle.

WHY THIS IS NEEDED (from a quick search before writing this):
    Cheap 9g servos are not built to a tight spec. Datasheets often quote
    a nominal pulse width range of ~500-2400us for the full 0-180 degree
    sweep, but individual clone units vary -- push too close to either
    end and the servo will strain, buzz, or grind against its internal
    stop instead of stopping cleanly. There isn't one "correct" number
    that works for every 9g servo; you have to sweep this specific unit
    and find where it actually stops moving freely.

    Sources: SunFounder's SG90 lesson and the electronicshub SG90 guide
    both quote 500-2400us as the datasheet range but note real-world
    per-unit safe range is narrower; the "full 180 degrees" script on
    GitHub (kuzned/servo_correction) exists specifically because the
    naive 1000-2000us assumption clips real SG90 clones short of 180.

    Separately: this robot is a Pi 5, and the classic `pigpio` daemon
    (the usual answer for jitter-free servo PWM on older Pis) does not
    talk to the Pi 5's GPIO controller (RP1) the way it did on
    Pi 4 and earlier -- lots of reports of it simply not working there.
    Since this project already uses `lgpio` for the line sensors
    (pid_line_follow.py), and lgpio has built-in hardware-timed servo
    support (`tx_servo`), that's what this uses too -- no extra
    dependency, and it avoids the timing jitter that plain software PWM
    (bit-banged in a Python loop) is prone to, which shows up as visible
    twitching on a servo.

Usage:
    python3 line_follow/calibrate_servo.py
    Then use the on-screen keys to sweep the pulse width and find:
      - MIN_PULSE_US -- as low as it goes before straining/buzzing
      - MAX_PULSE_US -- as high as it goes before straining/buzzing
      - CENTER_PULSE_US -- wherever "camera looking forward" is
    Write those three numbers into camera_servo.py's constants once found.

Wiring:
    Servo signal wire -> a free GPIO pin (SERVO_PIN below, default 18 --
    change it if that pin's in use elsewhere on this robot)
    Servo V+ -> 5V (NOT a GPIO pin -- servos draw more current than a
    GPIO pin can safely source, especially under load/stall)
    Servo GND -> Pi GND (must share ground with the Pi even though power
    comes from the 5V rail directly)
"""

import sys
import termios
import tty

import lgpio

SERVO_PIN = 18  # BCM pin -- change if already used elsewhere

START_PULSE_US = 1500  # neutral/center starting point for most servos
STEP_US = 25            # how much each keypress adjusts the pulse width
MIN_SAFE_US = 400       # hard floor -- don't let the tool itself command
MAX_SAFE_US = 2600      # something wildly out of any plausible servo's range


def _get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main():
    h = lgpio.gpiochip_open(0)

    pulse_us = START_PULSE_US
    lgpio.tx_servo(h, SERVO_PIN, pulse_us)

    print("Servo calibration -- pin", SERVO_PIN)
    print("  [ / ]  -- decrease / increase by", STEP_US, "us")
    print("  { / }  -- decrease / increase by 100us (coarse)")
    print("  c      -- mark this pulse width as CENTER")
    print("  n      -- mark this pulse width as MIN (low end)")
    print("  x      -- mark this pulse width as MAX (high end)")
    print("  q      -- quit and print a summary")
    print()

    marks = {"center": None, "min": None, "max": None}

    try:
        while True:
            print(f"\rpulse={pulse_us}us     ", end="", flush=True)
            key = _get_key()

            if key == "q":
                break
            elif key == "[":
                pulse_us = max(MIN_SAFE_US, pulse_us - STEP_US)
            elif key == "]":
                pulse_us = min(MAX_SAFE_US, pulse_us + STEP_US)
            elif key == "{":
                pulse_us = max(MIN_SAFE_US, pulse_us - 100)
            elif key == "}":
                pulse_us = min(MAX_SAFE_US, pulse_us + 100)
            elif key == "c":
                marks["center"] = pulse_us
                print(f"\n  marked CENTER = {pulse_us}us")
            elif key == "n":
                marks["min"] = pulse_us
                print(f"\n  marked MIN = {pulse_us}us")
            elif key == "x":
                marks["max"] = pulse_us
                print(f"\n  marked MAX = {pulse_us}us")
            else:
                continue

            lgpio.tx_servo(h, SERVO_PIN, pulse_us)

    except KeyboardInterrupt:
        pass
    finally:
        lgpio.tx_servo(h, SERVO_PIN, 0)  # stop sending pulses (releases servo)
        lgpio.gpiochip_close(h)

    print("\n\n--- Calibration summary ---")
    print(f"MIN_PULSE_US    = {marks['min']}")
    print(f"CENTER_PULSE_US = {marks['center']}")
    print(f"MAX_PULSE_US    = {marks['max']}")
    print("\nPaste these three values into camera_servo.py's constants.")


if __name__ == "__main__":
    main()
