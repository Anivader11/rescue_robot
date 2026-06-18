"""
vision_thread.py
================
Camera threads for two Pi Camera Module V2 units connected via CSI.

Camera assignment (based on physical robot):
    CAM0 — downward facing  → line following
    CAM1 — forward facing   → obstacle detection

Uses picamera2 (not cv2.VideoCapture) because Pi Camera Modules are
CSI cameras, not USB cameras — they don't appear as /dev/videoX devices
and won't open with VideoCapture.

Install picamera2 if not present:
    sudo apt install python3-picamera2 -y

If cameras are on opposite ports to what's listed, swap LINE_CAM and FWD_CAM.
"""

import threading
import logging
import time
import numpy as np
from vision.line_detector import LineDetector
from control.shared_vision import SharedVision

log = logging.getLogger(__name__)

# Which CSI port each camera is on
LINE_CAM = 0   # downward facing — line following
FWD_CAM  = 1   # forward facing  — obstacle detection

FRAME_WIDTH  = 640
FRAME_HEIGHT = 480
LINE_CAM_FPS = 30
FWD_CAM_FPS  = 15   # obstacles don't need high frame rate

# Obstacle detection: contour area at a known distance (calibrate on your robot)
AREA_AT_15CM = 8000.0


class VisionThread:

    def __init__(
        self,
        shared:            SharedVision,
        line_cam_index:    int  = LINE_CAM,
        fwd_cam_index:     int  = FWD_CAM,
        debug:             bool = False,
    ):
        self.shared         = shared
        self.debug          = debug
        self._line_cam_idx  = line_cam_index
        self._fwd_cam_idx   = fwd_cam_index
        self._running       = False
        self._cal_frame     = None

        self.detector = LineDetector(FRAME_WIDTH, FRAME_HEIGHT, debug=debug)

        self._line_thread = threading.Thread(
            target=self._line_loop, name="line-cam", daemon=True
        )
        self._fwd_thread = threading.Thread(
            target=self._fwd_loop, name="fwd-cam", daemon=True
        )

    def start(self, calibration_frame=None) -> None:
        self._running   = True
        self._cal_frame = calibration_frame
        self._line_thread.start()
        self._fwd_thread.start()
        log.info("[Vision] Started — line=CAM%d  fwd=CAM%d",
                 self._line_cam_idx, self._fwd_cam_idx)

    def stop(self) -> None:
        self._running = False
        self._line_thread.join(timeout=3.0)
        self._fwd_thread.join(timeout=3.0)
        log.info("[Vision] Stopped")

    # ------------------------------------------------------------------
    # Line camera loop — Pi Camera Module V2 via picamera2
    # ------------------------------------------------------------------

    def _line_loop(self) -> None:
        try:
            from picamera2 import Picamera2
        except ImportError:
            log.error("[line-cam] picamera2 not installed. Run: sudo apt install python3-picamera2 -y")
            return

        cam = Picamera2(self._line_cam_idx)
        config = cam.create_preview_configuration(
            main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "BGR888"},
            controls={"FrameRate": LINE_CAM_FPS}
        )
        cam.configure(config)
        cam.start()
        log.info("[line-cam] Camera started on CAM%d", self._line_cam_idx)

        calibrated = False

        try:
            while self._running:
                # capture_array returns BGR numpy array directly
                frame = cam.capture_array()

                if not calibrated:
                    ref = self._cal_frame if self._cal_frame is not None else frame
                    self.detector.calibrate(ref)
                    calibrated = True
                    log.info("[line-cam] Calibrated")

                result = self.detector.detect(frame)
                self.shared.update_line(result)

                if self.debug and result.debug_frame is not None:
                    # Save debug frame to disk periodically for headless inspection
                    # (imshow won't work headless; use calibrate.py for live view)
                    pass

        finally:
            cam.stop()
            log.info("[line-cam] Camera stopped")

    # ------------------------------------------------------------------
    # Forward camera loop — Pi Camera Module V2 via picamera2
    # ------------------------------------------------------------------

    def _fwd_loop(self) -> None:
        try:
            from picamera2 import Picamera2
        except ImportError:
            log.error("[fwd-cam] picamera2 not installed. Run: sudo apt install python3-picamera2 -y")
            return

        cam = Picamera2(self._fwd_cam_idx)
        config = cam.create_preview_configuration(
            main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "BGR888"},
            controls={"FrameRate": FWD_CAM_FPS}
        )
        cam.configure(config)
        cam.start()
        log.info("[fwd-cam] Camera started on CAM%d", self._fwd_cam_idx)

        try:
            import cv2
            while self._running:
                frame = cam.capture_array()
                obstacle, distance_cm = self._detect_obstacle(frame, cv2)
                self.shared.update_forward(obstacle, distance_cm)

        finally:
            cam.stop()
            log.info("[fwd-cam] Camera stopped")

    def _detect_obstacle(self, frame: np.ndarray, cv2) -> tuple:
        """
        Detect obstacles in the forward camera frame.

        Obstacles (§2.4.5): min 150mm high, base max 150mm diameter,
        placed min 250mm from field edge. They are large, blocky, and dark
        relative to the white tile background.

        Returns (obstacle_detected: bool, distance_cm: float)
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        # Only look in lower half — obstacles sit on the floor
        roi = gray[h // 2:, :]

        # Adaptive threshold — more robust than fixed value under venue lighting
        binary = cv2.adaptiveThreshold(
            roi, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            51, 8
        )

        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        obstacle    = False
        distance_cm = 999.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 1500:
                continue

            x, y, bw, bh = cv2.boundingRect(cnt)

            # Must be tall enough — obstacles are min 150mm, debris is max 3mm
            # At typical camera height, 150mm obstacle ≈ 30+ px tall
            if bh < 30:
                continue

            # Must be blocky — not the line itself
            if bw < 20:
                continue

            # Rough distance: larger contour area = closer
            est_cm = (AREA_AT_15CM / area) * 15.0
            if est_cm < distance_cm:
                distance_cm = est_cm
                obstacle    = True

        return obstacle, distance_cm
