"""
accel_speed_boost.py
======================
Background thread that polls the BNO085's raw accelerometer
(imu_sensor.py) and flags a speed boost when Y acceleration drops
below ACCEL_Y_BOOST_THRESHOLD -- e.g. the reading you saw on the bench,
"Accel (m/s^2): X=+0.12 Y=-9.78 Z=+0.34", going more negative than
-15 m/s^2 on Y.

Same pattern as tof_detector.py / green_detector.py: a daemon thread
that just reads and records state, doesn't touch the motors itself.
pid_line_follow.py reads get_boost() each loop tick and scales
BASE_SPEED up while it's True.

Usage:
    boost = AccelSpeedBoost()
    boost.start()
    ...
    is_boosted, accel_y = boost.get_boost()
    ...
    boost.stop()
"""

import time
import threading
import logging

from imu_sensor import BNO085IMU

log = logging.getLogger(__name__)

POLL_HZ = 20
POLL_SLEEP = 1.0 / POLL_HZ

ACCEL_Y_BOOST_THRESHOLD = -15.0  # m/s^2 -- Y below this triggers the boost


class AccelSpeedBoost:
    def __init__(self):
        self._lock = threading.Lock()
        self._boosted = False
        self._accel_y = 0.0
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def get_boost(self):
        """Returns (is_boosted, accel_y)."""
        with self._lock:
            return self._boosted, self._accel_y

    def _loop(self):
        try:
            imu = BNO085IMU()
        except Exception:
            log.exception("[accel-speed-boost] failed to open BNO085 over I2C")
            return

        log.info("[accel-speed-boost] IMU started")

        while self._running:
            try:
                _ax, ay, _az = imu.read_acceleration()
            except Exception:
                log.exception("[accel-speed-boost] read failed")
                time.sleep(POLL_SLEEP)
                continue

            boosted = ay < ACCEL_Y_BOOST_THRESHOLD

            with self._lock:
                if boosted != self._boosted:
                    log.info(f"[accel-speed-boost] boost {'ON' if boosted else 'OFF'} (Y={ay:+.2f})")
                self._boosted = boosted
                self._accel_y = ay

            time.sleep(POLL_SLEEP)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    b = AccelSpeedBoost()
    b.start()
    try:
        while True:
            boosted, accel_y = b.get_boost()
            print(f"boosted={boosted}  Y={accel_y:+6.2f}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        b.stop()
