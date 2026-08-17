"""
motors.py
=========
Motor driver for 4x RoboMaster M2006 P36 brushless motors via I2C.
Uses the Powerful BLDC Driver board with CircuitPython library.

Layout:
    Left side:  motors at I2C addresses LEFT_FRONT_ADDR, LEFT_REAR_ADDR
    Right side: motors at I2C addresses RIGHT_FRONT_ADDR, RIGHT_REAR_ADDR

All four motors on same I2C bus. Pi 5 I2C1 on pins 3 (SDA) and 5 (SCL).
I2C speed must be 1MHz — set in /boot/firmware/config.txt:
    dtoverlay=i2c1,pins_2_3,baudrate=1000000

Speed units: 1 LSB = 2^-16 electrical revolutions/second.
BASE_SPEED_UNITS is the cruising speed in these units.
Tune this value on the floor — start low and increase.

Sign convention:
    Positive speed = forward for BOTH sides (left motors may need negating
    depending on physical mounting orientation — set LEFT_INVERTED below).
"""

import os
import json
import time
import logging
from abc import ABC, abstractmethod

log = logging.getLogger(__name__)

# calibration_offsets.json lives next to this file's project root (saved by
# calibrate_once.py). Loaded once at import time so BLDCMotorDriver() doesn't
# need to hit disk per-motor.
CALIBRATION_OFFSETS_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "calibration_offsets.json"
)

# ---------------------------------------------------------------------------
# I2C addresses — read the label on each motor driver board
# Change these to match your actual boards
# ---------------------------------------------------------------------------
LEFT_FRONT_ADDR  = 0x20
LEFT_REAR_ADDR   = 0x1B
RIGHT_FRONT_ADDR = 0x19
RIGHT_REAR_ADDR  = 0x1A

# ---------------------------------------------------------------------------
# Motor orientation — if left motors are mounted mirrored, negate their speed
# ---------------------------------------------------------------------------
LEFT_INVERTED  = True    # flip if left wheels spin backwards
RIGHT_INVERTED = False

# ---------------------------------------------------------------------------
# Speed constants for M2006 P36
# 1 LSB = 2^-16 electrical rev/s
# M2006 has 36:1 gearbox, 7 pole pairs
# ---------------------------------------------------------------------------
BASE_SPEED_UNITS  = 30000000    # cruising speed — tune on floor
MAX_SPEED_UNITS   = 55050240    # hard cap sent to driver
SPEED_LIMIT_UNITS = 55050240   # driver-side speed limit register

# M2006 P36 FOC tuning constants (from Tuning_Constants.docx, firmware v3)
IQ_KP = 1500
IQ_KI = 200
ID_KP = 1500
ID_KI = 200
SPEED_KP = 4e-2
SPEED_KI = 4e-4
SPEED_KD = 3e-2
POSITION_KP = 275
POSITION_KI = 0
POSITION_KD = 0
POS_REGION_BOUNDARY = 250000
CURRENT_LIMIT_FOC = 65536   # 1 amp = 65536 units

# NOTE: motors.py does NOT run calibration itself anymore — it only ever
# reads calibration_offsets.json, written by calibrate_once.py. All the
# calibration-run parameters (voltage/speed/scycles/cycles) live there now,
# not here. Rerun calibrate_once.py after any power-cycle of the driver
# boards, or after swapping/rewiring a motor — its output feeds this file.


# ---------------------------------------------------------------------------
# Abstract base
# ---------------------------------------------------------------------------

class MotorDriver(ABC):

    @abstractmethod
    def set_speeds(self, left: float, right: float) -> None:
        """
        Set left and right drive speeds.
        left, right: -100.0 (full reverse) .. 0 .. +100.0 (full forward)
        Internally scaled to BLDC speed units.
        """

    def drive(self, base_speed: float, turn: float) -> None:
        """
        Differential drive.
        turn > 0 → turn right (left wheel speeds up, right wheel slows)
        turn < 0 → turn left  (right wheel speeds up, left wheel slows)
        Values in -100..100 percentage range.
        """
        left  = max(-100.0, min(100.0, base_speed + turn))
        right = max(-100.0, min(100.0, base_speed - turn))
        self.set_speeds(left, right)

    def stop(self) -> None:
        self.set_speeds(0.0, 0.0)

    def rotate_left(self, speed: float = 30.0) -> None:
        self.set_speeds(-speed, speed)

    def rotate_right(self, speed: float = 30.0) -> None:
        self.set_speeds(speed, -speed)

    def drive_straight(self, speed: float = 50.0) -> None:
        self.set_speeds(speed, speed)


# ---------------------------------------------------------------------------
# Real I2C driver using CircuitPython powerfulbldcdriver library
# ---------------------------------------------------------------------------

class BLDCMotorDriver(MotorDriver):
    """
    Controls 4x M2006 P36 motors via the Powerful BLDC Driver boards over I2C.
    Requires CircuitPython libraries installed on the Pi.

    Install:
        pip install git+https://github.com/Aw3someAndrew/SteelBar_CircuitPython_powerful_bldc_driver.git

    Also requires in /boot/firmware/config.txt:
        dtoverlay=i2c1,pins_2_3,baudrate=1000000
    """

    def __init__(self):
        try:
            import board
            import busio
            from steelbar_powerful_bldc_driver import PowerfulBLDCDriver
            self._driver_cls = PowerfulBLDCDriver
        except ImportError as e:
            raise RuntimeError(
                f"CircuitPython libraries not found: {e}\n"
                "Run: pip install git+https://github.com/Aw3someAndrew/"
                "SteelBar_CircuitPython_powerful_bldc_driver.git"
            )

        import board, busio
        self._i2c = busio.I2C(board.SCL, board.SDA)
        self._saved_offsets = self._load_saved_offsets()

        log.info("[BLDC] Initialising motors on I2C1...")
        self._lf = self._init_motor(LEFT_FRONT_ADDR,  "LEFT_FRONT")
        self._lr = self._init_motor(LEFT_REAR_ADDR,   "LEFT_REAR")
        self._rf = self._init_motor(RIGHT_FRONT_ADDR, "RIGHT_FRONT")
        self._rr = self._init_motor(RIGHT_REAR_ADDR,  "RIGHT_REAR")
        log.info("[BLDC] All motors initialised")

    def _load_saved_offsets(self) -> dict:
        """Load calibration_offsets.json saved by calibrate_once.py, keyed
        by motor name (LEFT_FRONT, LEFT_REAR, RIGHT_FRONT, RIGHT_REAR).

        motors.py no longer calibrates anything itself — it ONLY ever reads
        this file. If it's missing, that's a hard stop: run
        calibrate_once.py first. This is deliberate — no silent fallback to
        a live calibration that could mask a stale/wrong file.
        """
        if not os.path.exists(CALIBRATION_OFFSETS_PATH):
            raise RuntimeError(
                f"[BLDC] {CALIBRATION_OFFSETS_PATH} not found. "
                "motors.py only reads saved calibration values — it does "
                "not calibrate live. Run calibrate_once.py first (after "
                "every power-cycle of the driver boards, or after "
                "swapping/rewiring a motor), then try again."
            )
        with open(CALIBRATION_OFFSETS_PATH, "r") as f:
            data = json.load(f)
        log.info(f"[BLDC] Loaded saved calibration offsets for: {sorted(data.keys())}")
        return data

    def _init_motor(self, address: int, name: str):
        """Initialise one motor driver: set PID constants, load saved
        calibration offsets, switch to FOC speed mode.

        Never calibrates live. If `name` has no entry in
        calibration_offsets.json, this raises rather than silently
        calibrating or silently running uncalibrated — re-run
        calibrate_once.py to (re)generate a complete file.
        """
        m = self._driver_cls(self._i2c, address)

        version = m.get_firmware_version()
        if version != 3:
            raise RuntimeError(
                f"[BLDC] {name} at 0x{address:02X}: unexpected firmware v{version}, expected v3"
            )
        log.info(f"[BLDC] {name} 0x{address:02X} firmware v{version} OK")

        # Same order as calibrate_once.py / SteelBar's reference script.
        m.set_current_limit_foc(CURRENT_LIMIT_FOC)
        m.set_id_pid_constants(ID_KP, ID_KI)
        m.set_iq_pid_constants(IQ_KP, IQ_KI)
        m.set_speed_pid_constants(SPEED_KP, SPEED_KI, SPEED_KD)
        m.set_position_pid_constants(POSITION_KP, POSITION_KI, POSITION_KD)
        m.set_position_region_boundary(POS_REGION_BOUNDARY)
        m.set_speed_limit(SPEED_LIMIT_UNITS)

        saved = self._saved_offsets.get(name)
        if not saved:
            raise RuntimeError(
                f"[BLDC] No saved calibration for {name} in "
                f"{CALIBRATION_OFFSETS_PATH}. Run calibrate_once.py to "
                "(re)generate a complete file covering all 4 motors."
            )
        self._apply_saved_offsets(m, name, saved)

        # FOC + sin/cos encoder, speed command mode
        m.configure_operating_mode_and_sensor(3, 1)   # 3=FOC, 1=sin/cos
        m.configure_command_mode(12)                   # 12=speed

        # Re-apply PID/current/speed-limit constants AFTER calibration.
        # Theory: switching into calibration mode (15) may reset or ignore
        # these registers (calibration is an open-loop voltage-driven
        # rotation, doesn't need them) — calibrate_once.py never actually
        # commanded a real speed afterward so this was never caught. Only
        # RIGHT_FRONT moved under a real speed command post-calibration
        # while all 4 clearly moved during calibration itself, which points
        # at exactly this: PID gains not surviving the mode round-trip.
        m.set_current_limit_foc(CURRENT_LIMIT_FOC)
        m.set_id_pid_constants(ID_KP, ID_KI)
        m.set_iq_pid_constants(IQ_KP, IQ_KI)
        m.set_speed_pid_constants(SPEED_KP, SPEED_KI, SPEED_KD)
        m.set_position_pid_constants(POSITION_KP, POSITION_KI, POSITION_KD)
        m.set_position_region_boundary(POS_REGION_BOUNDARY)
        m.set_speed_limit(SPEED_LIMIT_UNITS)

        m.clear_faults()
        return m

    def _apply_saved_offsets(self, m, name: str, saved: dict) -> None:
        """Write a previously-calibrated ELECANGLEOFFSET/SINCOSCENTRE pair
        instead of running live calibration.

        Still has to enter calibration mode (15) to write these registers —
        same requirement as start_calibration() — then switches back out.
        No start_calibration()/is_calibration_finished() wait, since we're
        not measuring anything, just restoring known-good values.
        """
        offset = saved["elec_angle_offset"]
        centre = saved["sincos_centre"]
        log.info(f"[BLDC] Applying saved calibration for {name}: "
                 f"ELECANGLEOFFSET={offset}, SINCOSCENTRE={centre}")

        m.configure_operating_mode_and_sensor(15, 1)   # 15=calibration, 1=sin/cos
        m.configure_command_mode(15)                     # 15=calibration

        m.set_ELECANGLEOFFSET(offset)
        m.set_SINCOSCENTRE(centre)

    def _pct_to_units(self, pct: float) -> int:
        """Convert -100..100 percentage to BLDC speed units."""
        units = int((pct / 100.0) * MAX_SPEED_UNITS)
        return max(-MAX_SPEED_UNITS, min(MAX_SPEED_UNITS, units))

    def set_speeds(self, left: float, right: float) -> None:
        left_units  = self._pct_to_units(left)
        right_units = self._pct_to_units(right)

        if LEFT_INVERTED:
            left_units = -left_units
        if RIGHT_INVERTED:
            right_units = -right_units

        # Both left motors get the same speed
        self._lf.set_speed(left_units)
        self._lr.set_speed(left_units)

        # Both right motors get the same speed
        self._rf.set_speed(right_units)
        self._rr.set_speed(right_units)

        log.debug(f"[BLDC] L={left_units}  R={right_units}")

    def check_errors(self) -> dict:
        """
        Read ERROR1 byte from all four motors.
        Returns dict of motor name → error byte. 0x00 = no error.
        Call this periodically to catch overcurrent/overtemp.
        """
        errors = {}
        for name, motor in [
            ("LEFT_FRONT",  self._lf),
            ("LEFT_REAR",   self._lr),
            ("RIGHT_FRONT", self._rf),
            ("RIGHT_REAR",  self._rr),
        ]:
            motor.update_quick_data_readout()
            e1 = motor.get_ERROR1_QDR()
            e2 = motor.get_ERROR2_QDR()
            if e1 or e2:
                log.warning(f"[BLDC] {name} ERROR1=0x{e1:02X} ERROR2=0x{e2:02X}")
            errors[name] = (e1, e2)
        return errors

    def cleanup(self) -> None:
        self.stop()
        self._i2c.deinit()


# ---------------------------------------------------------------------------
# PID controller (unchanged from original)
# ---------------------------------------------------------------------------

class PIDController:
    """
    Line-following PID. error is normalised -1..1 from LineDetector.
    Output is a turn value passed to MotorDriver.drive().
    """

    def __init__(self, kp: float = 0.40, ki: float = 0.001, kd: float = 0.15):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral   = 0.0
        self._prev_error = 0.0
        self._last_time  = time.monotonic()
        self.integral_max = 100.0

    def compute(self, error: float) -> float:
        now = time.monotonic()
        dt  = max(now - self._last_time, 0.001)

        self._integral += error * dt
        self._integral  = max(-self.integral_max,
                              min(self.integral_max, self._integral))

        derivative = (error - self._prev_error) / dt
        output = (self.kp * error
                + self.ki * self._integral
                + self.kd * derivative)

        self._prev_error = error
        self._last_time  = now
        return output

    def reset(self) -> None:
        self._integral   = 0.0
        self._prev_error = 0.0
        self._last_time  = time.monotonic()


# ---------------------------------------------------------------------------
# Stub driver for testing without hardware
# ---------------------------------------------------------------------------

class StubMotorDriver(MotorDriver):
    """Prints commands instead of driving hardware. Safe on any machine."""

    def __init__(self, verbose: bool = True):
        self.verbose     = verbose
        self.last_left   = 0.0
        self.last_right  = 0.0

    def set_speeds(self, left: float, right: float) -> None:
        self.last_left  = left
        self.last_right = right
        if self.verbose:
            print(f"  [motors] L={left:+6.1f}  R={right:+6.1f}")

    def cleanup(self) -> None:
        pass
