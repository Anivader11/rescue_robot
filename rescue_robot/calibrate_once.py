"""
calibrate_once.py
==================
Calibrates all 4 motors ONCE and saves the resulting ELECANGLEOFFSET /
SINCOSCENTRE values to calibration_offsets.json.

Closely follows SteelBar's own reference example
(Aw3someAndrew/SteelBar_CircuitPython_powerful_bldc_driver) rather than our
own test scripts — in particular it does NOT impose an artificial timeout on
calibration. The reference script simply blocks on
`while not is_calibration_finished(): ...` until the driver itself reports
done, however long that actually takes. Our earlier 30s timeout guess was
wrong (calibration was still visibly running past it), so this script trusts
the driver instead of a guessed time limit.

Run this once after every power-on of the motor driver boards (offsets are
volatile — they reset when the boards lose power). Then main.py / motors.py
can load calibration_offsets.json and apply the saved offsets directly with
set_ELECANGLEOFFSET()/set_SINCOSCENTRE() instead of re-running the full
calibration routine on every run.

Usage:
    python3 calibrate_once.py
"""

import sys
import json
import time
import board
import busio
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver

# Same addresses/names as control/motors.py — keep these in sync.
MOTORS = {
    "LEFT_FRONT":  0x20,
    "LEFT_REAR":   0x1B,
    "RIGHT_FRONT": 0x19,
    "RIGHT_REAR":  0x1A,
}

OUTPUT_PATH = "calibration_offsets.json"

# Calibration parameters — straight from SteelBar's reference script:
#   voltage=300   -> 300/3399 * vcc volts
#   speed=2097152 -> 2097152/65536 elecangle/s
#   scycles=50000 -> settle time
#   cycles=500000 -> calibration time
CAL_VOLTAGE = 300
CAL_SPEED = 2097152
CAL_SCYCLES = 50000
CAL_CYCLES = 500000


def main():
    i2c = busio.I2C(board.SCL, board.SDA)

    results = {}

    for name, address in MOTORS.items():
        print(f"\n=== {name} (0x{address:02X}) ===")
        m = PowerfulBLDCDriver(i2c, address)

        version = m.get_firmware_version()
        print(f"Firmware version: {version}")
        if version != 3:
            print(f"ERROR: unexpected firmware version for {name}, expected 3. Skipping.")
            continue

        # Same PID/current/speed setup as the reference script, before
        # entering calibration mode.
        m.set_current_limit_foc(65536)  # 1 amp — only matters in FOC run mode
        m.set_id_pid_constants(1500, 200)
        m.set_iq_pid_constants(1500, 200)
        m.set_speed_pid_constants(4e-2, 4e-4, 3e-2)
        m.set_position_pid_constants(275, 0, 0)
        m.set_position_region_boundary(250000)
        m.set_speed_limit(10000000)

        m.configure_operating_mode_and_sensor(15, 1)  # calibration mode, sin/cos encoder
        m.configure_command_mode(15)                    # calibration mode

        m.set_calibration_options(CAL_VOLTAGE, CAL_SPEED, CAL_SCYCLES, CAL_CYCLES)
        m.start_calibration()

        print(f"Calibrating {name}...", end="", flush=True)
        start = time.monotonic()
        while not m.is_calibration_finished():
            print(".", end="", flush=True)
            m.update_quick_data_readout()
            time.sleep(0.5)
        elapsed = time.monotonic() - start
        print(f"\n{name} calibration finished in {elapsed:.1f}s")

        offset = m.get_calibration_ELECANGLEOFFSET()
        centre = m.get_calibration_SINCOSCENTRE()
        print(f"  ELECANGLEOFFSET: {offset}")
        print(f"  SINCOSCENTRE:    {centre}")

        results[name] = {
            "address": address,
            "elec_angle_offset": offset,
            "sincos_centre": centre,
            "calibration_time_s": round(elapsed, 1),
        }

        # Back to FOC + speed mode
        m.configure_operating_mode_and_sensor(3, 1)   # FOC, sin/cos
        m.configure_command_mode(12)                    # speed
        m.clear_faults()
        m.set_speed(0)

    with open(OUTPUT_PATH, "w") as f:
        json.dump(results, f, indent=2)

    print(f"\nSaved offsets for {len(results)}/{len(MOTORS)} motors to {OUTPUT_PATH}")
    if len(results) < len(MOTORS):
        missing = set(MOTORS) - set(results)
        print(f"WARNING: missing calibration for: {sorted(missing)}")


if __name__ == "__main__":
    main()
