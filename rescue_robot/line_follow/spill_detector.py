"""
spill_detector.py
==================
Simple background camera thread that watches for the silver reflective
spill-zone tape. Same pattern as green_detector.py -- runs on its own
thread, just records whether spill tape was seen and when, and prints it.
Does not (yet) tell the robot to do anything -- wire that up once
detection is confirmed working on hardware.

Detection method: uses the trained classifier in spill_classifier.py
(logistic regression over color + texture features, trained from real
samples with collect_spill_samples.py / train_spill_classifier.py --
plain brightness/color thresholds did not reliably separate tape from
tile, see project notes). Checks a fixed box in the middle of the frame
each tick, same as the calibration/collection tools.

Before running this on the Pi, make sure spill_model_weights.py (trained
on your laptop) has been copied next to this file.
"""

import time
import threading
import logging

from spill_classifier import extract_features, predict_silver

log = logging.getLogger(__name__)

CAM_INDEX = 0
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
FPS = 15

BOX_SIZE = 150            # same box size used during sample collection/training
MIN_CONFIDENCE = 0.7       # only count it as "seen" above this confidence


class SpillDetector:
    """
    Call start() to begin watching in the background. Call get_latest()
    any time to get (seen, seen_at) for the most recent detection --
    seen is just True/False, seen_at is the time.monotonic() it happened.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._seen = False
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
        with self._lock:
            return self._seen, self._seen_at

    def _loop(self):
        try:
            from picamera2 import Picamera2
        except ImportError:
            log.error("picamera2 not installed. Run: sudo apt install python3-picamera2 -y")
            return

        cam = Picamera2(CAM_INDEX)
        config = cam.create_preview_configuration(
            main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "BGR888"},
            controls={"FrameRate": FPS}
        )
        cam.configure(config)
        cam.start()
        log.info("[spill-detector] camera started on CAM%d", CAM_INDEX)

        cx, cy = FRAME_WIDTH // 2, FRAME_HEIGHT // 2
        half = BOX_SIZE // 2
        x1, y1 = max(0, cx - half), max(0, cy - half)
        x2, y2 = min(FRAME_WIDTH, cx + half), min(FRAME_HEIGHT, cy + half)

        while self._running:
            frame = cam.capture_array()
            roi = frame[y1:y2, x1:x2]
            feats = extract_features(roi)
            is_silver, confidence = predict_silver(feats)

            seen = is_silver and confidence >= MIN_CONFIDENCE
            with self._lock:
                self._seen = seen
                if seen:
                    self._seen_at = time.monotonic()

            time.sleep(1.0 / FPS)

        cam.stop()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    d = SpillDetector()
    d.start()
    try:
        while True:
            seen, seen_at = d.get_latest()
            print(f"seen={seen} seen_at={seen_at}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        d.stop()
