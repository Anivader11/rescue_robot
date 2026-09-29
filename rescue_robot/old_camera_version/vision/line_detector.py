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

# The forward camera got physically moved into the line camera's mounting
# spot after the original line camera broke. If it landed rotated 180 in
# that new mount, left/right AND near/far all come in mirrored, so every
# steering correction fights itself. Flip this to True and retest before
# touching any PID or ROI values if the robot seems to correct the wrong
# way / always drift the same direction.
FLIP_180 = True

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

# How far up the frame a green marker can be and still trigger a turn.
# 0.5 = marker must be in the bottom HALF of the frame (near field only).
# Lowering this makes the robot recognise the marker, and so start
# turning, earlier/further away -- raising it makes it wait until the
# marker is closer before turning. Tune this in small steps and re-test:
# too low and the robot pivots before it has actually reached the
# intersection, which risks a Lack of Progress call under rule 6.4.1.7 --
# this is exactly the failure the near-field gate was added to prevent
# (see the comment in _detect_green_marker below).
GREEN_NEAR_GATE_FRACTION = 0.42   # was 0.5 -- lowered to start turns a little earlier


# ---------------------------------------------------------------------------
# Calibration persistence — the constants above are what calibrate.py tunes
# and saves to calibration.json. load_calibration() applies a saved file back
# onto this module so a real run picks up on-site tuning, not just defaults.
# ---------------------------------------------------------------------------

def load_calibration(path: str = "calibration.json") -> dict:
    """
    Load a calibration.json saved by calibrate.py and apply it to this
    module's constants. Call once at startup before creating LineDetector.
    Returns the loaded dict, or {} if no file was found.
    """
    import json, os
    if not os.path.exists(path):
        print(f"[LineDetector] No {path} found — using built-in defaults")
        return {}

    with open(path, "r") as f:
        values = json.load(f)

    applied = {}
    for key, value in values.items():
        if key in globals():
            globals()[key] = value
            applied[key] = value
        # darkness_bias isn't a module constant — it's a per-instance value
        # set by LineDetector.calibrate() itself, so it's intentionally skipped.

    print(f"[LineDetector] Loaded calibration from {path}: {applied}")
    return applied


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

class LineState(Enum):
    TRACKING       = auto()
    RECOVERING     = auto()
    LOST           = auto()
    INTERSECTION   = auto()


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
    double_marker:  bool  = False   # both sides → U-turn
    last_error:     float = 0.0
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
        if FLIP_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        h, w = frame.shape[:2]
        debug_frame  = frame.copy() if self.debug else None
        binary       = self._threshold(frame)
        near = self._analyse_roi(binary, ROI_NEAR, w, h, debug_frame, (0, 255, 0))
        mid  = self._analyse_roi(binary, ROI_MID,  w, h, debug_frame, (0, 200, 255))
        far  = self._analyse_roi(binary, ROI_FAR,  w, h, debug_frame, (255, 150, 0))
        # Marker side is judged relative to the LINE, not the frame centre —
        # if the robot approaches an intersection offset from the line, a
        # frame-centre reference can put the marker on the wrong side.
        line_cx = near["cx"] if near is not None else (mid["cx"] if mid is not None else None)
        green_marker, green_marker_x, double_marker = self._detect_green_marker(frame, debug_frame, line_cx)
        intersection = self._detect_intersection(binary, ROI_MID, w, h, debug_frame)
        return self._build_result(
            near, mid, far, intersection,
            green_marker, green_marker_x, double_marker,
            w, debug_frame
        )

    def _threshold(self, frame):
        lab    = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        L      = lab[:, :, 0]
        c      = ADAPTIVE_C + self._darkness_bias
        binary = cv2.adaptiveThreshold(
            L, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, ADAPTIVE_BLOCK, c
        )
        k_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binary  = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, k_close, iterations=1)
        k_open  = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary  = cv2.morphologyEx(binary, cv2.MORPH_OPEN,  k_open,  iterations=1)
        binary  = self._mask_green(frame, binary)
        # Zero out top 25% too close, only sees robot chassis
        binary[:int(binary.shape[0] * 0.25), :] = 0
        return binary

    def _mask_green(self, frame, binary):
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            (GREEN_H_LOW,  GREEN_S_LOW,  GREEN_V_LOW),
            (GREEN_H_HIGH, GREEN_S_HIGH, GREEN_V_HIGH)
        )
        binary[mask > 0] = 0
        return binary

    def _analyse_roi(self, binary, roi_frac, frame_w, frame_h, debug_frame, colour):
        y_start = int(frame_h * roi_frac[0])
        y_end   = int(frame_h * roi_frac[1])
        roi     = binary[y_start:y_end, :]
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best      = None
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
                M  = cv2.moments(cnt)
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
        return False

    def _detect_green_marker(self, frame, debug_frame, line_cx=None):
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            (GREEN_H_LOW,  GREEN_S_LOW,  GREEN_V_LOW),
            (GREEN_H_HIGH, GREEN_S_HIGH, GREEN_V_HIGH)
        )
        h, w = mask.shape
        # Only trigger on markers in the NEAR (bottom) half of the frame —
        # i.e. when the marker is under/just ahead of the robot. Rule 2.3.1.4
        # places markers immediately BEFORE the intersection, so triggering on
        # far-field sightings made the robot pivot mid-tile, well before the
        # intersection (→ Lack of Progress under 6.4.1.7). With the near-field
        # gate, by the time the marker fills the bottom half the robot is at
        # the intersection and the turn happens in the right place.
        mask[:int(h * GREEN_NEAR_GATE_FRACTION), :] = 0

        # Reference x for "left/right of the line": the detected line centre
        # if we have one, else the frame centre as a fallback.
        ref_x = float(line_cx) if line_cx is not None else w / 2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid_markers = []   # (area, x_norm relative to line)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < GREEN_MIN_AREA_PX:
                continue
            x, y, cw, ch = cv2.boundingRect(cnt)
            if 0.4 < (cw / max(ch, 1)) < 2.5:
                marker_cx     = x + cw / 2
                marker_x_norm = (marker_cx - ref_x) / (w / 2)
                valid_markers.append((area, marker_x_norm))
                if self.debug and debug_frame is not None:
                    cv2.rectangle(debug_frame, (x, y), (x+cw, y+ch), (0, 200, 0), 2)
                    cv2.putText(debug_frame, f"M x={marker_x_norm:+.2f}",
                                (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 0), 2)

        if len(valid_markers) == 0:
            return False, 0.0, False

        # Check for double marker — one on each side of the line
        has_left  = any(x < -0.15 for _, x in valid_markers)
        has_right = any(x >  0.15 for _, x in valid_markers)
        double_marker = has_left and has_right

        if double_marker:
            if self.debug and debug_frame is not None:
                cv2.putText(debug_frame, "DOUBLE MARKER → U-TURN",
                            (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            return True, 0.0, True   # x=0.0 irrelevant for u-turn

        # Single marker — largest area wins (contour enumeration order is
        # arbitrary; a small green false-positive must not outvote the real
        # 40x40mm marker).
        valid_markers.sort(key=lambda t: -t[0])
        best_x = valid_markers[0][1]
        return True, best_x, False

    def _build_result(self, near, mid, far, intersection,
                      green_marker, green_marker_x, double_marker,
                      frame_w, debug_frame):
        half_w = frame_w / 2

        def norm_error(r):
            raw = r["cx"] - half_w
            return float(np.clip(raw / half_w, -1.0, 1.0)), float(raw)

        if near is not None:
            # Steer off the farthest band we can actually see, not just the
            # near band under the wheels -- this makes the robot lean into a
            # curve as soon as it appears ahead instead of only reacting once
            # it reaches the front of the robot. TRACKING state (and whether
            # we are on the line at all) is still decided by the near band,
            # since that is what confirms we haven't actually lost the line.
            look_ahead = far if far is not None else (mid if mid is not None else near)
            error_norm, error_px = norm_error(look_ahead)
            self._lost_counter = 0
            self._last_error   = error_norm
            state = LineState.INTERSECTION if intersection else LineState.TRACKING
            self._last_state   = state
            if self.debug and debug_frame is not None:
                col = (0, 255, 0) if state == LineState.TRACKING else (0, 100, 255)
                cv2.putText(debug_frame, f"err={error_norm:+.3f}  {state.name}",
                            (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
            return LineResult(
                state=state,
                error=error_norm,
                error_px=error_px,
                confidence=near["confidence"],
                line_width_px=near["width_px"],
                intersection=intersection,
                green_marker=green_marker,
                green_marker_x=green_marker_x,
                double_marker=double_marker,
                last_error=self._last_error,
                debug_frame=debug_frame
            )

        recovery = mid if mid is not None else far
        if recovery is not None:
            error_norm, error_px = norm_error(recovery)
            self._lost_counter = 0
            self._last_error   = error_norm
            self._last_state   = LineState.RECOVERING
            if self.debug and debug_frame is not None:
                cv2.putText(debug_frame, f"RECOVERING err={error_norm:+.3f}",
                            (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
            return LineResult(
                state=LineState.RECOVERING,
                error=error_norm,
                error_px=error_px,
                confidence=recovery["confidence"] * 0.6,
                line_width_px=recovery["width_px"],
                intersection=intersection,
                green_marker=green_marker,
                green_marker_x=green_marker_x,
                double_marker=double_marker,
                last_error=self._last_error,
                debug_frame=debug_frame
            )

        self._lost_counter += 1
        if self._lost_counter >= LOST_PATIENCE_FRAMES:
            state = LineState.LOST
        else:
            state = self._last_state if self._last_state != LineState.LOST else LineState.RECOVERING
        if self.debug and debug_frame is not None:
            cv2.putText(debug_frame, f"LOST ({self._lost_counter}/{LOST_PATIENCE_FRAMES})",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        self._last_state = state
        return LineResult(
            state=state,
            error=self._last_error,
            error_px=self._last_error * half_w,
            confidence=0.0,
            intersection=False,
            green_marker=green_marker,
            green_marker_x=green_marker_x,
            double_marker=double_marker,
            last_error=self._last_error,
            debug_frame=debug_frame
        )