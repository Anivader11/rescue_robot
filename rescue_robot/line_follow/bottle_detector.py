"""
bottle_detector.py
====================
Camera-based confirmation check for the ToF obstacle trigger: the ToF
sensor alone can't tell a ramp (a big flat surface rising up right in
front of it) apart from an actual obstacle (a 1.25L clear water
bottle) -- both can read <=50mm. This module looks at what the camera
sees at that moment and answers "does this actually look like a
bottle" before pid_line_follow.py commits to the evade sequence.

Why edges, not color: the bottle is clear plastic, so color/HSV
thresholding (like green_detector.py) won't work -- there's no
consistent color to key on. Instead this looks at SHAPE, using the
fact that a bottle at ~50mm fills most of the frame with two roughly
vertical side edges (its outline + the specular highlight line down a
cylindrical clear bottle), whereas a ramp filling the frame at the
same distance presents mostly diagonal/horizontal edges (its sloped
leading edge and the horizontal lines where the ramp surface meets the
tile).

Method:
    1. Canny edge detection on the grayscale frame.
    2. Probabilistic Hough transform to find line segments.
    3. Keep only near-vertical segments (within VERTICAL_ANGLE_TOLERANCE_DEG
       of straight up/down) that are at least MIN_LINE_LENGTH_FRAC of the
       frame height long.
    4. If there are at least MIN_VERTICAL_LINES such segments, AND the
       ratio of (near-vertical line length) to (all detected line
       length) is above VERTICAL_LINE_RATIO_THRESHOLD (i.e. the scene
       is dominated by vertical lines rather than diagonal/horizontal
       ones), call it a bottle.

This is a first pass, not a trained classifier -- tune the constants
below once you can test it against the actual bottle and the actual
ramp on the robot (their exact look will depend on lighting, camera
angle, and how close 50mm actually ends up framing things).

Usage:
    from bottle_detector import is_bottle
    confirmed = is_bottle(frame)   # frame: BGR image from Picamera2/OpenCV
"""

import numpy as np
import cv2

# --- Tunables (start here, adjust after testing on the real hardware) ---
CANNY_LOW = 50
CANNY_HIGH = 150

HOUGH_THRESHOLD = 30          # min votes for a line segment
HOUGH_MIN_LINE_LENGTH_PX = 40  # min segment length in pixels
HOUGH_MAX_LINE_GAP_PX = 10     # max gap to join segments into one line

VERTICAL_ANGLE_TOLERANCE_DEG = 15.0   # how far from straight up/down still counts as "vertical"
MIN_LINE_LENGTH_FRAC = 0.25           # a vertical segment must be at least this fraction of frame height
MIN_VERTICAL_LINES = 2                # need at least this many qualifying vertical segments
VERTICAL_LINE_RATIO_THRESHOLD = 0.5   # vertical line length must be >= this fraction of all line length


def _line_angle_deg(x1, y1, x2, y2):
    """Angle of the line from vertical, in degrees (0 = perfectly vertical)."""
    dx = x2 - x1
    dy = y2 - y1
    if dy == 0:
        return 90.0
    angle_from_vertical = np.degrees(np.arctan2(abs(dx), abs(dy)))
    return angle_from_vertical


def _line_length(x1, y1, x2, y2):
    return float(np.hypot(x2 - x1, y2 - y1))


def analyze_frame(frame):
    """Returns a dict of the raw measurements used to decide is_bottle(),
    useful for tuning/debugging (e.g. printing these live while testing)."""
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, CANNY_LOW, CANNY_HIGH)

    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=HOUGH_THRESHOLD,
        minLineLength=HOUGH_MIN_LINE_LENGTH_PX,
        maxLineGap=HOUGH_MAX_LINE_GAP_PX,
    )

    total_length = 0.0
    vertical_length = 0.0
    vertical_count = 0
    min_vertical_len_px = MIN_LINE_LENGTH_FRAC * h

    if lines is not None:
        # Some OpenCV versions return shape (N, 1, 4), others (N, 4) -- flatten to (N, 4).
        for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
            length = _line_length(x1, y1, x2, y2)
            total_length += length

            angle = _line_angle_deg(x1, y1, x2, y2)
            if angle <= VERTICAL_ANGLE_TOLERANCE_DEG and length >= min_vertical_len_px:
                vertical_length += length
                vertical_count += 1

    vertical_ratio = (vertical_length / total_length) if total_length > 0 else 0.0

    return {
        "vertical_count": vertical_count,
        "vertical_ratio": vertical_ratio,
        "total_line_length": total_length,
        "edges": edges,
    }


def is_bottle(frame):
    """Returns True if the frame looks like a clear vertical bottle rather
    than a ramp/flat surface. See module docstring for the method."""
    stats = analyze_frame(frame)
    return (
        stats["vertical_count"] >= MIN_VERTICAL_LINES
        and stats["vertical_ratio"] >= VERTICAL_LINE_RATIO_THRESHOLD
    )


if __name__ == "__main__":
    # Quick manual test using a laptop webcam -- point it at a water
    # bottle vs. a flat/angled surface (a book held at an angle works
    # as a rough ramp stand-in) and watch the numbers change.
    cap = cv2.VideoCapture(0)
    print("Testing bottle_detector.py on webcam. Ctrl-C to stop.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            stats = analyze_frame(frame)
            confirmed = (
                stats["vertical_count"] >= MIN_VERTICAL_LINES
                and stats["vertical_ratio"] >= VERTICAL_LINE_RATIO_THRESHOLD
            )
            print(
                f"bottle={confirmed}  vertical_count={stats['vertical_count']}  "
                f"vertical_ratio={stats['vertical_ratio']:.2f}"
            )
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
