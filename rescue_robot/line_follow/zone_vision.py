"""
zone_vision.py
===============
Camera checks used inside the evacuation zone (RCJA 2026 Open):

    find_balls(frame)          -> list of Ball(kind, cx, cy, r)
                                  kind = "alive" (silver) or "dead" (black)
    find_evac_point(frame, c)  -> (cx, cy, area) of the biggest "green" or
                                  "red" evacuation-point blob, or None

Ball detection: HoughCircles on the grey image, then classify by the
colour INSIDE the circle:
    dark  (mean V < BLACK_V_MAX)            -> dead victim (black ball)
    grey/bright, low saturation             -> live victim (silver ball)
    anything saturated (red/green etc.)     -> ignored
Plus a dark-blob fallback for the black ball in case Hough misses it
(black on a white floor is a very easy contour).

Run it on its own to tune thresholds -- it prints detections and saves an
annotated frame to ~/ez_debug.jpg every second (scp it back to look):
    python3 line_follow/zone_vision.py
"""

import os
import time
from collections import namedtuple

import cv2
import numpy as np

Ball = namedtuple("Ball", "kind cx cy r")

# --- Ball tunables ---
HOUGH_DP = 1.2
HOUGH_MIN_DIST = 30
HOUGH_PARAM1 = 100      # Canny high threshold
HOUGH_PARAM2 = 28       # accumulator threshold -- LOWER = more (and falser) circles
MIN_RADIUS = 8          # px, ball far away
MAX_RADIUS = 110        # px, ball right in front of the camera
BLACK_V_MAX = 70        # mean brightness below this = black ball
SILVER_S_MAX = 70       # mean saturation below this (and not black) = silver ball
IGNORE_ABOVE_Y = 0      # ignore circles whose centre is above this row (e.g. outside the walls)

# dark-blob fallback for the black ball
BLACK_BLOB_MIN_AREA = 150
BLACK_BLOB_MIN_CIRCULARITY = 0.65

# --- Evacuation point tunables ---
GREEN_LO, GREEN_HI = (38, 60, 40), (85, 255, 255)     # same range as camera_watcher.py
RED_LO1, RED_HI1 = (0, 100, 60), (10, 255, 255)       # red wraps round the hue circle
RED_LO2, RED_HI2 = (170, 100, 60), (180, 255, 255)
EVAC_MIN_AREA = 400

# The Pi camera is set to "BGR888", which on picamera2 actually delivers pixels
# in R,G,B order -- so to OpenCV red looks BLUE (confirmed: red triangle came
# out blue in cam_check.jpg). Only HUE is affected (brightness/saturation are
# the same either way), so only the red/green triangle check needs this.
FRAME_IS_RGB = True

# Balls already dropped sit INSIDE a hollow triangle -- ignore any ball whose
# centre is inside (or within this many px of) a red/green triangle's outline.
IGNORE_BALLS_IN_TRIANGLES = True
TRIANGLE_MARGIN_PX = 5


def _classify(hsv, cx, cy, r):
    mask = np.zeros(hsv.shape[:2], np.uint8)
    cv2.circle(mask, (int(cx), int(cy)), max(2, int(r * 0.7)), 255, -1)
    _h, s, v, _ = cv2.mean(hsv, mask=mask)
    if v < BLACK_V_MAX:
        return "dead"
    if s < SILVER_S_MAX:
        return "alive"
    return None


def find_balls(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.medianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 5)
    balls = []

    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=HOUGH_DP, minDist=HOUGH_MIN_DIST,
                               param1=HOUGH_PARAM1, param2=HOUGH_PARAM2,
                               minRadius=MIN_RADIUS, maxRadius=MAX_RADIUS)
    if circles is not None:
        for cx, cy, r in circles[0]:
            if cy < IGNORE_ABOVE_Y:
                continue
            kind = _classify(hsv, cx, cy, r)
            if kind:
                balls.append(Ball(kind, float(cx), float(cy), float(r)))

    # Fallback: round dark blob = black ball (skip if Hough already found one there)
    dark = cv2.inRange(hsv, (0, 0, 0), (180, 255, BLACK_V_MAX))
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        if area < BLACK_BLOB_MIN_AREA:
            continue
        perim = cv2.arcLength(c, True)
        circ = 4 * np.pi * area / (perim * perim) if perim > 0 else 0
        if circ < BLACK_BLOB_MIN_CIRCULARITY:
            continue
        (cx, cy), r = cv2.minEnclosingCircle(c)
        if cy < IGNORE_ABOVE_Y or r > MAX_RADIUS:
            continue
        if any(abs(b.cx - cx) < b.r and abs(b.cy - cy) < b.r for b in balls):
            continue
        balls.append(Ball("dead", float(cx), float(cy), float(r)))

    if IGNORE_BALLS_IN_TRIANGLES and balls:
        hulls = _triangle_hulls(frame)
        if hulls:
            balls = [b for b in balls
                     if all(cv2.pointPolygonTest(hl, (b.cx, b.cy), True) < -TRIANGLE_MARGIN_PX for hl in hulls)]
    return balls


def pick_target(balls, kind=None):
    """Nearest ball (= biggest radius), optionally only of one kind."""
    cands = [b for b in balls if kind is None or b.kind == kind]
    return max(cands, key=lambda b: b.r) if cands else None


def _true_hsv(frame):
    return cv2.cvtColor(frame, cv2.COLOR_RGB2HSV if FRAME_IS_RGB else cv2.COLOR_BGR2HSV)


def _colour_mask(hsv, colour):
    if colour == "green":
        return cv2.inRange(hsv, GREEN_LO, GREEN_HI)
    return cv2.inRange(hsv, RED_LO1, RED_HI1) | cv2.inRange(hsv, RED_LO2, RED_HI2)


def _triangle_hulls(frame):
    hsv = _true_hsv(frame)
    hulls = []
    for colour in ("red", "green"):
        contours, _ = cv2.findContours(_colour_mask(hsv, colour), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        hulls += [cv2.convexHull(c) for c in contours if cv2.contourArea(c) >= EVAC_MIN_AREA]
    return hulls


def find_evac_point(frame, colour):
    hsv = _true_hsv(frame)
    mask = _colour_mask(hsv, colour)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = max(contours, key=cv2.contourArea, default=None)
    if best is None or cv2.contourArea(best) < EVAC_MIN_AREA:
        return None
    M = cv2.moments(best)
    return M["m10"] / M["m00"], M["m01"] / M["m00"], cv2.contourArea(best)


def annotate(frame, balls, evac=None):
    out = frame.copy()
    for b in balls:
        col = (0, 0, 0) if b.kind == "dead" else (255, 255, 0)
        cv2.circle(out, (int(b.cx), int(b.cy)), int(b.r), col, 2)
        cv2.putText(out, b.kind, (int(b.cx - b.r), int(b.cy - b.r) - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
    for name, e in (evac or {}).items():
        if e:
            cv2.drawMarker(out, (int(e[0]), int(e[1])), (255, 0, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(out, name, (int(e[0]) + 6, int(e[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
    return out


if __name__ == "__main__":
    from camera_watcher import CameraWatcher
    from camera_servo import CameraServo, LOOK_UP_VALUE
    servo = CameraServo()
    servo.set_value(LOOK_UP_VALUE)          # same camera angle the endzone code uses (held while running)
    print(f"camera servo at LOOK_UP_VALUE = {LOOK_UP_VALUE}")
    w = CameraWatcher()
    w.start()
    out_path = os.path.expanduser("~/ez_debug.jpg")
    print(f"Watching for balls / evac points. Annotated frame saved to {out_path} every 1 s. Ctrl-C to stop.")
    try:
        while True:
            frame, _ = w.get_frame()
            if frame is not None:
                balls = find_balls(frame)
                evac = {"green": find_evac_point(frame, "green"), "red": find_evac_point(frame, "red")}
                print("balls:", [(b.kind, int(b.cx), int(b.cy), int(b.r)) for b in balls],
                      "| green:", evac["green"] and int(evac["green"][2]),
                      "| red:", evac["red"] and int(evac["red"][2]))
                cv2.imwrite(out_path, annotate(frame, balls, evac))
            time.sleep(1.0)
    except KeyboardInterrupt:
        w.stop()
        servo.release()
