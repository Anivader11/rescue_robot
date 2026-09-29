"""
motors.py
=========
Motor driver for the robot's two front RoboMaster M2006 P36 brushless
motors via I2C, using the Powerful BLDC Driver boards.

TWO WHEEL DRIVE — the rear right motor failed, so the robot now runs on
just the two FRONT motors (LEFT_FRONT, RIGHT_FRONT). Both rear motors are
disconnected/unused; MOTOR_CONFIG below only lists the two front ones.
Steering still works the same way as before (differential drive between
left and right), just with one wheel per side instead of two.

Kept deliberately simple: BLDCMotorDriver below is a straight port of
test_forwardv2.py's setup_motor() — the only script confirmed to actually
drive the robot correctly — wrapped so state_machine.py can use it through
the same MotorDriver interface as StubMotorDriver. No calibration file,
no firmware-version check, no position PID, no double-apply of constants.

Sign convention (proven by test_forwardv2.py):
    Forward = right side gets +speed, left side gets -speed.

IMPORTANT — calibration values are volatile:
    elec_angle_offset / sincos_centre in MOTOR_CONFIG below reset if the
    driver boards lose power. If the boards get power-cycled, re-run
    calibrate_once.py (or re-verify test_forwardv2.py still drives
    straight) and paste the new values into MOTOR_CONFIG here.

I2C1 on Pi GPIO pins 3 (SDA) / 5 (SCL), 1MHz — set in
/boot/firmware/config.txt:
    dtoverlay=i2c1,pins_2_3,baudrate=1000000
"""

import time
import logging
from abc import ABC, abstractmethod

log = logging.getLogger(__name__)

# Only the two front motors — rear right failed, rear left is now unused
# too so the robot drives symmetrically on one wheel per side. Same
# proven values as test_forwardv2.py's MOTOR_CONFIG for these two.
MOTOR_CONFIG = {
    "LEFT_FRONT":  {"address": 0x20, "elec_angle_offset": 1327246080, "sincos_centre": 1230},
    "RIGHT_FRONT": {"address": 0x19, "elec_angle_offset": 1185441536, "sincos_centre": 1248},
}

SPEED_LIMIT     = 546133333   # driver-side hard cap (test_forwardv2.py proven)
MAX_SPEED_UNITS = 25000000   # what 100% maps to == test_forwardv2.py DRIVE_SPEED
                   

# ---------------------------------------------------------------------------
# Abstract base
# ---------------------------------------------------------------------------

class MotorDriver(ABC):

    @abstractmethod
    def set_speeds(self, left: float, right: float) -> None:
        """left, right: -100.0 (full reverse) .. 0 .. +100.0 (full forward)."""

    def drive(self, base_speed: float, turn: float) -> None:
        """
        Differential drive. turn > 0 → turn right, turn < 0 → turn left.
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
# Real I2C driver — direct port of test_forwardv2.py
# ---------------------------------------------------------------------------

class BLDCMotorDriver(MotorDriver):
    """
    Controls the 2 front M2006 P36 motors via the Powerful BLDC Driver
    boards over I2C. Same init sequence, PID constants, and calibration
    values as test_forwardv2.py.

    Install:
        pip install git+https://github.com/Aw3someAndrew/SteelBar_CircuitPython_powerful_bldc_driver.git
    """

    def __init__(self):
        import board
        import busio
        from steelbar_powerful_bldc_driver import PowerfulBLDCDriver

        self._i2c = busio.I2C(board.SCL, board.SDA)

        log.info("[BLDC] Initialising motors on I2C1...")
        self._lf = self._setup_motor(PowerfulBLDCDriver, MOTOR_CONFIG["LEFT_FRONT"])
        self._rf = self._setup_motor(PowerfulBLDCDriver, MOTOR_CONFIG["RIGHT_FRONT"])
        log.info("[BLDC] Both motors initialised (2 wheel drive)")

    def _setup_motor(self, driver_cls, cfg):
        m = driver_cls(self._i2c, cfg["address"])
        m.set_current_limit_foc(65536)              # 1 amp
        m.set_id_pid_constants(1100, 150)
        m.set_iq_pid_constants(1100, 150)
        m.set_speed_pid_constants(2.5e-2, 2.5e-4, 2e-2)
        m.set_position_region_boundary(250000)
        m.set_ELECANGLEOFFSET(cfg["elec_angle_offset"])
        m.set_SINCOSCENTRE(cfg["sincos_centre"])
        m.set_speed_limit(SPEED_LIMIT)
        m.configure_operating_mode_and_sensor(3, 1)  # FOC, sin/cos encoder
        m.configure_command_mode(12)                  # speed command mode
        m.clear_faults()
        return m

    def _pct_to_units(self, pct: float) -> int:
        units = int((pct / 100.0) * MAX_SPEED_UNITS)
        return max(-MAX_SPEED_UNITS, min(MAX_SPEED_UNITS, units))

    def set_speeds(self, left: float, right: float) -> None:
        # Proven convention: right side positive, left side negative = forward.
        left_units  = -self._pct_to_units(left)
        right_units =  self._pct_to_units(right)

        self._lf.set_speed(left_units)
        self._rf.set_speed(right_units)

        log.debug(f"[BLDC] L={left_units}  R={right_units}")

    def stop(self) -> None:
        for m in (self._lf, self._rf):
            m.set_speed(0)

    def cleanup(self) -> None:
        self.stop()
        for m in (self._lf, self._rf):
            m.clear_faults()
        self._i2c.deinit()


# ---------------------------------------------------------------------------
# PID controller (unchanged)
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
