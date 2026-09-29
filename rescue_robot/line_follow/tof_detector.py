"""
tof_detector.py
=================
Background thread that continuously polls the SteelBar ToF distance
sensor (see tof_sensor.py) and records the latest distance reading, same
pattern as green_detector.py / spill_detector.py. Does NOT (yet) tell
the robot to do anything -- just detects and records, for now, per your
earlier answer. Wire up actual obstacle-avoidance/stop logic once this
is confirmed working on hardware.

Usage:
    detector = ToFDetector()
    detector.start()
    ...
    distance_mm, seen_at = detector.get_latest()
    ...
    detector.stop()
"""

import time
import threading
import logging

from tof_sensor import SteelBarToF

log = logging.getLogger(__name__)

POLL_HZ = 20   # how often to check for a new reading
POLL_SLEEP = 1.0 / POLL_HZ


class ToFDetector:
    def __init__(self):
        self._lock = threading.Lock()
        self._distance_mm = None
        self._seen_at = 0.0
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

    def get_latest(self):
        """Returns (distance_mm, seen_at) for the most recent reading,
        or (None, 0.0) if nothing has been read yet."""
        with self._lock:
            return self._distance_mm, self._seen_at

    def _loop(self):
        try:
            sensor = SteelBarToF()
        except Exception:
            log.exception("[tof-detector] failed to open ToF sensor over I2C")
            return

        log.info("[tof-detector] sensor started")

        while self._running:
            try:
                distance_mm = sensor.current_measurement()
            except Exception:
                log.exception("[tof-detector] read failed")
                time.sleep(POLL_SLEEP)
                continue

            with self._lock:
                self._distance_mm = distance_mm
                self._seen_at = time.monotonic()

            time.sleep(POLL_SLEEP)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    d = ToFDetector()
    d.start()
    try:
        while True:
            distance_mm, seen_at = d.get_latest()
            print(f"distance_mm={distance_mm} seen_at={seen_at}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        d.stop()
