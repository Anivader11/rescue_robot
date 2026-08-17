"""
test_individual_livecal.py
===========================
Diagnostic-only script — NOT part of the normal run flow.

Same test as test_individual.py (spin each of the 4 motors alone, others
held at 0, print position/speed telemetry) but initialises motors with a
FULL LIVE CALIBRATION every time, instead of loading calibration_offsets.json.

Purpose: isolate whether "only RIGHT_FRONT moves" is caused by
loading saved ELECANGLEOFFSET/SINCOSCENTRE values from JSON (suspected —
this exact symptom was seen before when trying that shortcut), or whether
it's a deeper hardware/config issue that happens either way.

If all 4 motors move here (live calibration) but only RIGHT_FRONT moves in
test_individual.py (loaded from JSON) — that CONFIRMS the bug is in
"apply saved offsets without re-running calibration", not your wiring or
motors. If only RIGHT_FRONT still moves here too, the bug is deeper than
the load-from-JSON path and calibration_offsets.json isn't the culprit.

This script deliberately duplicates motors.py's PID constants and
calibration sequence rather than importing BLDCMotorDriver, since
BLDCMotorDriver no longer calibrates live at all (by design, per the
current control/motors.py) — this script exists purely to test the
hypothesis, not to become a new permanent code path.

Run:
    python3 test_individual_livecal.py
"""

import time
import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

import board
import busio
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver

from control.motors import (
    LEFT_FRONT_ADDR, LEFT_REAR_ADDR, RIGHT_FRONT_ADDR, RIGHT_REAR_ADDR,
    IQ_KP, IQ_KI, ID_KP, ID_KI,
    SPEED_KP, SPEED_KI, SPEED_KD,
    POSITION_KP, POSITION_KI, POSITION_KD, POS_REGION_BOUNDARY,
    CURRENT_LIMIT_FOC, SPEED_LIMIT_UNITS,
)

# Same as calibrate_once.py / SteelBar's reference script.
CAL_VOLTAGE = 300
CAL_SPEED = 2097152
CAL_SCYCLES = 50000
CAL_CYCLES = 500000

TEST_SPEED_UNITS = 3_000_000   # matches test_individual.py
RAMP_STEPS = 10
RAMP_DELAY = 0.05
HOLD_S = 3.0

MOTORS = {
    "LEFT_FRONT":  LEFT_FRONT_ADDR,
    "LEFT_REAR":   LEFT_REAR_ADDR,
    "RIGHT_FRONT": RIGHT_FRONT_ADDR,
    "RIGHT_REAR":  RIGHT_REAR_ADDR,
}


def apply_pid_constants(m):
    m.set_current_limit_foc(CURRENT_LIMIT_FOC)
    m.set_id_pid_constants(ID_KP, ID_KI)
    m.set_iq_pid_constants(IQ_KP, IQ_KI)
    m.set_speed_pid_constants(SPEED_KP, SPEED_KI, SPEED_KD)
    m.set_position_pid_constants(POSITION_KP, POSITION_KI, POSITION_KD)
    m.set_position_region_boundary(POS_REGION_BOUNDARY)
    m.set_speed_limit(SPEED_LIMIT_UNITS)


def init_and_calibrate_live(i2c, name, address):
    log.info(f"[{name}] Connecting at 0x{address:02X}...")
    m = PowerfulBLDCDriver(i2c, address)

    version = m.get_firmware_version()
    if version != 3:
        raise RuntimeError(f"{name} at 0x{address:02X}: unexpected firmware v{version}")
    log.info(f"[{name}] firmware v{version} OK")

    apply_pid_constants(m)

    log.info(f"[{name}] Calibrating live...")
    m.configure_operating_mode_and_sensor(15, 1)
    m.configure_command_mode(15)
    m.set_calibration_options(CAL_VOLTAGE, CAL_SPEED, CAL_SCYCLES, CAL_CYCLES)
    m.start_calibration()

    start = time.monotonic()
    while not m.is_calibration_finished():
        m.update_quick_data_readout()
        time.sleep(0.5)
    elapsed = time.monotonic() - start

    offset = m.get_calibration_ELECANGLEOFFSET()
    centre = m.get_calibration_SINCOSCENTRE()
    log.info(f"[{name}] calibrated in {elapsed:.1f}s: "
             f"ELECANGLEOFFSET={offset}, SINCOSCENTRE={centre}")

    m.configure_operating_mode_and_sensor(3, 1)   # FOC, sin/cos
    m.configure_command_mode(12)                    # speed

    # Re-apply PID/current/speed-limit constants AFTER calibration — same
    # fix as control/motors.py._init_motor(), since switching into
    # calibration mode may reset/ignore these registers.
    apply_pid_constants(m)

    m.clear_faults()
    return m


def ramp(motor, target, steps=RAMP_STEPS, delay=RAMP_DELAY):
    for i in range(1, steps + 1):
        motor.set_speed(int(target * i / steps))
        time.sleep(delay)


def read_telemetry(motor, name, label):
    motor.update_quick_data_readout()
    pos = motor.get_position_QDR()
    spd = motor.get_speed_QDR()
    e1 = motor.get_ERROR1_QDR()
    e2 = motor.get_ERROR2_QDR()
    flag = " <-- FAULT" if (e1 or e2) else ""
    print(f"  [{label}] {name}: pos={pos}  speed={spd}  "
          f"ERROR1=0x{e1:02X} ERROR2=0x{e2:02X}{flag}")


def main():
    i2c = busio.I2C(board.SCL, board.SDA)

    log.info("Initialising + LIVE calibrating all 4 motors "
              "(this will take a couple minutes)...")
    motors = {}
    for name, addr in MOTORS.items():
        motors[name] = init_and_calibrate_live(i2c, name, addr)
    log.info("All 4 motors live-calibrated.")

    try:
        for name, motor in motors.items():
            input(f"\nENTER to spin {name} alone (others held at 0)...")
            read_telemetry(motor, name, "BEFORE")
            ramp(motor, TEST_SPEED_UNITS)
            read_telemetry(motor, name, "AT SPEED")
            time.sleep(HOLD_S)
            ramp(motor, 0)
            print(f"  -> did {name} move?")
    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        for motor in motors.values():
            motor.set_speed(0)
        i2c.deinit()
        log.info("Motors stopped")


if __name__ == "__main__":
    main()
