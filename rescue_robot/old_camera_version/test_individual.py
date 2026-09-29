"""
test_individual.py
===================
Initialises all 4 motors via the real BLDCMotorDriver (same
calibration_offsets.json loading path as main.py / test_forward.py — it
does NOT calibrate live, only reads the saved file), then spins each one
INDIVIDUALLY (one at a time, ramped), with the other three held at zero
speed the whole time.

Purpose: isolate whether "only RIGHT_FRONT moves" is a simultaneous-driving
problem (all 4 commanded at once) or whether 3 of the 4 motors just don't
respond post-calibration regardless of how many others are also being
driven.

Prereq: calibration_offsets.json must exist and cover all 4 motors — run
calibrate_once.py first if you haven't already, or BLDCMotorDriver() will
raise immediately.

Run:
    python3 test_individual.py
"""

import time
import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

from control.motors import BLDCMotorDriver

TEST_SPEED_UNITS = 3_000_000   # raw driver units, matches identify_motors.py's TARGET_SPEED scale
RAMP_STEPS = 10
RAMP_DELAY = 0.05
HOLD_S = 3.0


def ramp(motor, target, steps=RAMP_STEPS, delay=RAMP_DELAY):
    for i in range(1, steps + 1):
        motor.set_speed(int(target * i / steps))
        time.sleep(delay)


def read_telemetry(motor, name, label):
    """Read the driver chip's own view of position/speed/faults. No error
    flags showing up (confirmed on all 4 motors) rules out overcurrent/
    overtemp/fault-under-load — this checks the next layer down: does the
    chip's closed-loop FOC controller even register a nonzero commanded/
    measured speed, or does it just sit at zero regardless of what
    set_speed() was told? That tells us whether the command is reaching
    the control loop at all vs. reaching it but producing no real motion."""
    motor.update_quick_data_readout()
    pos = motor.get_position_QDR()
    spd = motor.get_speed_QDR()
    e1 = motor.get_ERROR1_QDR()
    e2 = motor.get_ERROR2_QDR()
    flag = " <-- FAULT" if (e1 or e2) else ""
    print(f"  [{label}] {name}: pos={pos}  speed={spd}  "
          f"ERROR1=0x{e1:02X} ERROR2=0x{e2:02X}{flag}")


def main():
    log.info("Initialising motors (loading calibration_offsets.json, should be quick)...")
    motors = BLDCMotorDriver()

    named = [
        ("LEFT_FRONT",  motors._lf),
        ("LEFT_REAR",   motors._lr),
        ("RIGHT_FRONT", motors._rf),
        ("RIGHT_REAR",  motors._rr),
    ]

    try:
        for name, motor in named:
            input(f"\nENTER to spin {name} alone (others held at 0)...")
            read_telemetry(motor, name, "BEFORE")   # baseline before commanding speed
            ramp(motor, TEST_SPEED_UNITS)
            read_telemetry(motor, name, "AT SPEED")  # while ramped up and held
            time.sleep(HOLD_S)
            ramp(motor, 0)
            print(f"  -> did {name} move?")
    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        motors.stop()
        log.info("Motors stopped")
        if hasattr(motors, "cleanup"):
            motors.cleanup()


if __name__ == "__main__":
    main()
