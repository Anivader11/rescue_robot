"""
camera_watcher.py
===================
Single shared background camera thread that both green-marker detection
and spill/silver-tape detection run on top of. Only ONE physical camera
exists on the robot, so green_detector.py and spill_detector.py can't
each open their own Picamera2 instance at the same time -- this replaces
both with one thread that grabs a frame once per tick and runs both
checks on that same frame.

Usage (same shape as the old GreenDetector/SpillDetector):
    watcher = CameraWatcher()
    watcher.start()
    ...
    side, seen_at = watcher.get_green()      # "left"/"right"/None, timestamp
    seen, seen_at = watcher.get_spill()       # True/False, timestamp
    frame, seen_at = watcher.get_frame()      # latest raw BGR frame, for
                                               # on-demand checks like
                                               # bottle_detector.py
    ...
    watcher.stop()
"""

import time
import threading
import logging

import cv2
import numpy as np

from spill_classifier import extract_features, predict_silver

log = logging.getLogger(__name__)

CAM_INDEX = 0
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
FPS = 24

# --- Green marker settings (from green_detector.py) ---
MIN_GREEN_AREA = 300
GREEN_H_LOW,  GREEN_H_HIGH  = 38,  85
GREEN_S_LOW,  GREEN_S_HIGH  = 60, 255
GREEN_V_LOW,  GREEN_V_HIGH  = 40, 255

# --- Spill/silver tape settings (from spill_detector.py) ---
SPILL_BOX_SIZE = 150            # same box size used during sample collection/training
SPILL_MIN_CONFIDENCE = 0.7       # only count it as "seen" above this confidence


class CameraWatcher:
    def __init__(self):
        self._lock = threading.Lock()
        self._green_side = None
        self._green_seen_at = 0.0
        self._spill_seen = False
        self._spill_seen_at = 0.0
        self._frame = None
        self._frame_seen_at = 0.0
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

    def get_green(self):
        with self._lock:
            return self._green_side, self._green_seen_at

    def get_spill(self):
        with self._lock:
            return self._spill_seen, self._spill_seen_at

    def get_frame(self):
        """Returns (frame, seen_at) for the most recent captured frame --
        a copy, safe to hand to on-demand checks like bottle_detector.py
        without racing the capture thread. (None, 0.0) if nothing captured
        yet."""
        with self._lock:
            if self._frame is None:
                return None, 0.0
            return self._frame.copy(), self._frame_seen_at

    def _check_green(self, frame, half_w):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            (GREEN_H_LOW, GREEN_S_LOW, GREEN_V_LOW),
            (GREEN_H_HIGH, GREEN_S_HIGH, GREEN_V_HIGH)
        )
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_GREEN_AREA or area <= best_area:
                continue
            best_area = area
            best = cnt

        if best is not None:
            M = cv2.moments(best)
            cx = M["m10"] / M["m00"] if M["m00"] > 0 else half_w
            side = "right" if cx > half_w else "left"
            with self._lock:
                self._green_side = side
                self._green_seen_at = time.monotonic()
        else:
            # No green contour this frame -- revert to None instead of
            # leaving the last detected side stuck forever.
            with self._lock:
                self._green_side = None

    def _check_spill(self, frame, box):
        x1, y1, x2, y2 = box
        roi = frame[y1:y2, x1:x2]
        feats = extract_features(roi)
        is_silver, confidence = predict_silver(feats)

        seen = is_silver and confidence >= SPILL_MIN_CONFIDENCE
        with self._lock:
            self._spill_seen = seen
            if seen:
                self._spill_seen_at = time.monotonic()

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
        log.info("[camera-watcher] camera started on CAM%d", CAM_INDEX)

        half_w = FRAME_WIDTH / 2

        cx, cy = FRAME_WIDTH // 2, FRAME_HEIGHT // 2
        half = SPILL_BOX_SIZE // 2
        spill_box = (
            max(0, cx - half), max(0, cy - half),
            min(FRAME_WIDTH, cx + half), min(FRAME_HEIGHT, cy + half),
        )

        while self._running:
            frame = cam.capture_array()
            self._check_green(frame, half_w)
            self._check_spill(frame, spill_box)
            with self._lock:
                self._frame = frame
                self._frame_seen_at = time.monotonic()
            time.sleep(1.0 / FPS)

        cam.stop()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    w = CameraWatcher()
    w.start()
    try:
        while True:
            side, g_at = w.get_green()
            seen, s_at = w.get_spill()
            print(f"green_side={side} spill_seen={seen}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        w.stop()
