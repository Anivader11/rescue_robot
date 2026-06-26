"""
line_detector.py
================
Robust black-line detector for RoboCup Junior Rescue Line (Open division).
Designed for Raspberry Pi 5 + downward-facing Pi Camera Module V2.
"""

import cv2
import numpy as np
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

# ---------------------------------------------------------------------------
# Tunables
# ---------------------------------------------------------------------------
ROI_NEAR  = (0.75, 1.00)
ROI_MID   = (0.50, 0.75)
ROI_FAR   = (0.25, 0.50)

ADAPTIVE_BLOCK  = 51
ADAPTIVE_C      = 8

MIN_LINE_AREA      = 300
LINE_WIDTH_MIN_PX  = 6
LINE_WIDTH_MAX_PX  = 200

LOST_PATIENCE_FRAMES = 8

INTERSECTION_SPAN_FRACTION = 0.45

GREEN_H_LOW,  GREEN_H_HIGH  = 38,  85
GREEN_S_LOW,  GREEN_S_HIGH  = 60, 255
GREEN_V_LOW,  GREEN_V_HIGH  = 40, 255
GREEN_MIN_AREA_PX           = 400

TAPE_BRIGHTNESS_DELTA = 15
TAPE_MIN_ROW_FRACTION = 0.50
TAPE_SEARCH_FRAC      = (0.55, 1.00)


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

class LineState(Enum):
    TRACKING       = auto()
    RECOVERING     = auto()
    LOST           = auto()
    INTERSECTION   = auto()
    ENTERING_SPILL = auto()


@dataclass
class LineResult:
    state:          LineState
    error:          float = 0.0
    error_px:       float = 0.0
    confidence:     float = 0.0
    line_width_px:  float = 0.0
    intersection:   bool  = False
    green_marker:   bool  = False
    green_marker_x: float = 0.0
    spill_tape:     bool  = False
    last_error:     float = 0.0
    debug_frame:    Optional[np.ndarray] = field(default=None, repr=False)
    green_marker:   bool  = False
    green_marker_x: float = 0.0    # normalised centre x of marker
    debug_frame:    Optional[np.ndarray] = field(default=None, repr=False)



# ---------------------------------------------------------------------------
# Main detector
# ---------------------------------------------------------------------------

class LineDetector:

    def __init__(self, frame_width: int = 640, frame_height: int = 480, debug: bool = False):
        self.fw = frame_width
        self.fh = frame_height
        self.debug = debug
        self._darkness_bias   = 0
        self._tile_brightness = 200.0
        self._lost_counter    = 0
        self._last_error      = 0.0
        self._last_state      = LineState.LOST

    def calibrate(self, frame: np.ndarray) -> dict:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        sz = 30
        corners = [
            gray[:sz, :sz], gray[:sz, w-sz:],
            gray[h-sz:, :sz], gray[h-sz:, w-sz:],
        ]
        bg_brightness = float(np.mean([c.mean() for c in corners]))
        self._tile_brightness = bg_brightness
        strip = gray[int(h*0.7):int(h*0.9), int(w*0.3):int(w*0.7)]
        line_brightness = float(strip.min())
        contrast = bg_brightness - line_brightness
        if contrast < 40:
            self._darkness_bias = 4
        elif contrast < 70:
            self._darkness_bias = 0
        else:
            self._darkness_bias = -3
        result = {
            "bg_brightness": round(bg_brightness, 1),
            "line_brightness": round(line_brightness, 1),
            "contrast": round(contrast, 1),
            "darkness_bias": self._darkness_bias,
        }
        print(f"[LineDetector] Calibration: {result}")
        return result

    def detect(self, frame: np.ndarray) -> LineResult:
        if frame is None or frame.size == 0:
            return LineResult(state=LineState.LOST, last_error=self._last_error)
        h, w = frame.shape[:2]
        debug_frame = frame.copy() if self.debug else None
        binary      = self._threshold(frame)
        green_marker, green_marker_x = self._detect_green_marker(frame, debug_frame)
        spill_tape   = self._detect_spill_tape(frame, debug_frame)
        near = self._analyse_roi(binary, ROI_NEAR, w, h, debug_frame, (0, 255, 0))
        mid  = self._analyse_roi(binary, ROI_MID,  w, h, debug_frame, (0, 200, 255))
        far  = self._analyse_roi(binary, ROI_FAR,  w, h, debug_frame, (255, 150, 0))
        intersection = self._detect_intersection(binary, ROI_MID, w, h, debug_frame)
        return self._build_result(near, mid, far, intersection, green_marker, green_marker_x, spill_tape, w, debug_frame)

    def _threshold(self, frame):
        lab  = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        L    = lab[:, :, 0]
        c    = ADAPTIVE_C + self._darkness_bias
        binary = cv2.adaptiveThreshold(L, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                        cv2.THRESH_BINARY_INV, ADAPTIVE_BLOCK, c)
        k_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binary  = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, k_close, iterations=1)
        k_open  = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary  = cv2.morphologyEx(binary, cv2.MORPH_OPEN,  k_open,  iterations=1)
        binary  = self._mask_green(frame, binary)
        return binary

    def _mask_green(self, frame, binary):
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           (GREEN_H_LOW, GREEN_S_LOW, GREEN_V_LOW),
                           (GREEN_H_HIGH, GREEN_S_HIGH, GREEN_V_HIGH))
        binary[mask > 0] = 0
        return binary

    def _analyse_roi(self, binary, roi_frac, frame_w, frame_h, debug_frame, colour):
        y_start = int(frame_h * roi_frac[0])
        y_end   = int(frame_h * roi_frac[1])
        roi     = binary[y_start:y_end, :]
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_LINE_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if not (LINE_WIDTH_MIN_PX <= w <= LINE_WIDTH_MAX_PX):
                continue
            if area > best_area:
                best_area = area
                M = cv2.moments(cnt)
                cx = int(M["m10"] / M["m00"]) if M["m00"] > 0 else x + w // 2
                best = {
                    "cx": cx, "cy": y + h // 2 + y_start,
                    "area": area, "width_px": w,
                    "confidence": min(1.0, area / 6000),
                    "bbox": (x, y + y_start, w, h),
                }
        if best and self.debug and debug_frame is not None:
            x, y, w, h = best["bbox"]
            cv2.rectangle(debug_frame, (x, y), (x+w, y+h), colour, 2)
            cv2.circle(debug_frame, (best["cx"], best["cy"]), 5, colour, -1)
        return best

    def _detect_intersection(self, binary, roi_frac, frame_w, frame_h, debug_frame):
        y_start = int(frame_h * roi_frac[0])
        y_end   = int(frame_h * roi_frac[1])
        roi     = binary[y_start:y_end, :]
        row_coverage = (roi > 0).sum(axis=1) / frame_w
        detected = float(row_coverage.max()) >= INTERSECTION_SPAN_FRACTION
        if detected and self.debug and debug_frame is not None:
            cv2.line(debug_frame, (0, y_start), (frame_w, y_start), (0, 0, 255), 2)
            cv2.putText(debug_frame, "INTERSECTION", (10, y_start+15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        return detected

    def _detect_green_marker(self, frame, debug_frame):
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                       (GREEN_H_LOW, GREEN_S_LOW, GREEN_V_LOW),
                       (GREEN_H_HIGH, GREEN_S_HIGH, GREEN_V_HIGH))
        h, w = mask.shape
        mask[h // 2:, :] = 0
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < GREEN_MIN_AREA_PX:
                continue
            x, y, cw, ch = cv2.boundingRect(cnt)
            if 0.4 < (cw / max(ch, 1)) < 2.5:
            # Normalise centre x: -1 = far left, +1 = far right
                marker_cx = x + cw / 2
                marker_x_norm = (marker_cx - w / 2) / (w / 2)
                if self.debug and debug_frame is not None:
                    cv2.rectangle(debug_frame, (x, y), (x+cw, y+ch), (0, 200, 0), 2)
                    cv2.putText(debug_frame, f"MARKER x={marker_x_norm:+.2f}",
                            (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 2)
            return True, marker_x_norm
        return False, 0.0

    def _detect_spill_tape(self, frame, debug_frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        y_start = int(h * TAPE_SEARCH_FRAC[0])
        roi = gray[y_start:, :]
        threshold_brightness = self._tile_brightness + TAPE_BRIGHTNESS_DELTA
        bright_fraction = (roi > threshold_brightness).sum(axis=1) / w
        detected = float(bright_fraction.max()) >= TAPE_MIN_ROW_FRACTION
        if detected and self.debug and debug_frame is not None:
            best_row = int(bright_fraction.argmax()) + y_start
            cv2.line(debug_frame, (0, best_row), (w, best_row), (255, 255, 255), 3)
            cv2.putText(debug_frame, "SPILL TAPE", (10, best_row-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return detected

    def _build_result(self, near, mid, far, intersection, green_marker, green_marker_x, spill_tape, frame_w, debug_frame):
        half_w = frame_w / 2

        def norm_error(r):
            raw = r["cx"] - half_w
            return float(np.clip(raw / half_w, -1.0, 1.0)), float(raw)

        if spill_tape:
            self._lost_counter = 0
            return LineResult(state=LineState.ENTERING_SPILL, error=self._last_error,
                              error_px=self._last_error*half_w, confidence=1.0,
                              intersection=False, green_marker=green_marker,
                              green_marker_x=green_marker_x,
                              spill_tape=True, last_error=self._last_error,
                              debug_frame=debug_frame)

        if near is not None:
            error_norm, error_px = norm_error(near)
            self._lost_counter = 0
            self._last_error   = error_norm
            state = LineState.INTERSECTION if intersection else LineState.TRACKING
            if self.debug and debug_frame is not None:
                col = (0, 255, 0) if state == LineState.TRACKING else (0, 100, 255)
                cv2.putText(debug_frame, f"err={error_norm:+.3f}  {state.name}",
                            (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
            return LineResult(state=state, error=error_norm, error_px=error_px,
                              confidence=near["confidence"], line_width_px=near["width_px"],
                              intersection=intersection, green_marker=green_marker,
                              green_marker_x=green_marker_x,
                              spill_tape=False, last_error=self._last_error,
                              debug_frame=debug_frame)

        recovery = mid if mid is not None else far
        if recovery is not None:
            error_norm, error_px = norm_error(recovery)
            self._lost_counter = 0
            self._last_error   = error_norm
            if self.debug and debug_frame is not None:
                cv2.putText(debug_frame, f"RECOVERING err={error_norm:+.3f}",
                            (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
            return LineResult(state=LineState.RECOVERING, error=error_norm, error_px=error_px,
                              confidence=recovery["confidence"]*0.6,
                              line_width_px=recovery["width_px"],
                              intersection=intersection, green_marker=green_marker,
                              green_marker_x=green_marker_x,
                              spill_tape=False, last_error=self._last_error,
                              debug_frame=debug_frame)

        self._lost_counter += 1
        if self._lost_counter >= LOST_PATIENCE_FRAMES:
            state = LineState.LOST
        else:
            state = self._last_state if self._last_state != LineState.LOST else LineState.RECOVERING
        if self.debug and debug_frame is not None:
            cv2.putText(debug_frame, f"LOST ({self._lost_counter}/{LOST_PATIENCE_FRAMES})",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        self._last_state = state
        return LineResult(state=state, error=self._last_error,
                          error_px=self._last_error*half_w, confidence=0.0,
                          intersection=False, green_marker=green_marker,
                          green_marker_x=green_marker_x,
                          spill_tape=False, last_error=self._last_error,
                          debug_frame=debug_frame)
