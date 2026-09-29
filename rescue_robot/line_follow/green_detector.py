"""
green_detector.py
==================
Simple background camera thread that watches for a green marker square
and records which side of the frame it was on (left or right).

Runs entirely separately from the PID line-following loop. It just
keeps a single shared "last detection" record, with a timestamp, that
pid_line_follow.py checks each tick. It does NOT decide when to turn
or how long -- that's handled in pid_line_follow.py, using
GREEN_ACT_DELAY_S and GREEN_TURN_DURATION_S over there, so the camera
might see the marker "too early" and that's fine, the delay is tuned
separately from detection itself.

Detection method: same HSV green range already proven in
vision/line_detector.py. Convert frame to HSV, threshold for green,
find the largest green contour, and check whether its centre is left
or right of the frame's centre line.
"""

import time
import threading
import logging

import cv2
import numpy as np

log = logging.getLogger(__name__)

CAM_INDEX = 0
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
FPS = 24

MIN_GREEN_AREA = 300   # ignore small green specks/noise

GREEN_H_LOW,  GREEN_H_HIGH  = 38,  85
GREEN_S_LOW,  GREEN_S_HIGH  = 60, 255
GREEN_V_LOW,  GREEN_V_HIGH  = 40, 255


class GreenDetector:
    """
    Call start() to begin watching in the background. Call
    get_latest() any time to get (side, seen_at) for the most recent
    detection, or (None, None) if nothing has been seen yet.
    side is "left" or "right".
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._side = None
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
            return self._side, self._seen_at

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
        log.info("[green-detector] camera started on CAM%d", CAM_INDEX)

        half_w = FRAME_WIDTH / 2

        while self._running:
            frame = cam.capture_array()
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
                    self._side = side
                    self._seen_at = time.monotonic()
            else:
                # No green contour this frame -- revert to None instead of
                # leaving the last detected side stuck forever.
                with self._lock:
                    self._side = None

            time.sleep(1.0 / FPS)

        cam.stop()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    d = GreenDetector()
    d.start()
    try:
        while True:
            side, seen_at = d.get_latest()
            print(f"side={side} seen_at={seen_at}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        d.stop()
