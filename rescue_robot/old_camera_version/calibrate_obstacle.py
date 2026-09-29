"""
calibrate_obstacle.py
======================
Calibration helper for AREA_AT_18CM in vision/vision_thread.py.

Place a real obstacle (or whatever your competition uses) exactly 18cm
from the FORWARD camera lens, run this, and it prints the measured
contour area — paste that straight in as AREA_AT_18CM.

Uses the exact same detection pipeline as _detect_obstacle() in
vision_thread.py (grayscale → bottom-half ROI → adaptive threshold →
contour filtering) so the measured area is guaranteed to match what the
real robot would compute for the same obstacle at the same distance —
not a separate approximation.

Samples continuously for a few seconds so a
single lucky/unlucky frame doesn't set a skewed number — reports the
median qualifying contour area across the sample window.

Run:
    python3 calibrate_obstacle.py
"""

import time
import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

import cv2
import numpy as np

FWD_CAM_INDEX = 1     # matches FWD_CAM in vision_thread.py
FRAME_WIDTH   = 640
FRAME_HEIGHT  = 480

TARGET_DISTANCE_CM = 18.0
SAMPLE_SECONDS = 5.0
SAMPLE_HZ = 10

# Same filtering rules as _detect_obstacle() in vision_thread.py — kept
# identical on purpose, so a contour that would qualify on the real robot
# also qualifies here, and vice versa.
MIN_AREA   = 1500
MIN_HEIGHT = 30
MIN_WIDTH  = 20


def detect_obstacle_contours(frame, cv2):
    """Same logic as vision_thread.py's _detect_obstacle(), but returns
    ALL qualifying contour areas instead of just the closest one — useful
    here to see the full spread across the sample window."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    roi = gray[h // 2:, :]

    binary = cv2.adaptiveThreshold(
        roi, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        51, 8
    )

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    areas = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        if bh < MIN_HEIGHT:
            continue
        if bw < MIN_WIDTH:
            continue
        areas.append(area)
    return areas


def main():
    try:
        from picamera2 import Picamera2
    except ImportError:
        log.error("picamera2 not installed. Run: sudo apt install python3-picamera2 -y")
        return

    cam = Picamera2(FWD_CAM_INDEX)
    config = cam.create_preview_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "BGR888"}
    )
    cam.configure(config)
    cam.start()
    log.info(f"Camera started on CAM{FWD_CAM_INDEX}")

    try:
        input(f"\nPlace the obstacle exactly {TARGET_DISTANCE_CM:.0f}cm from the "
              "forward camera lens, centred in view. ENTER to start "
              f"{SAMPLE_SECONDS:.0f}s of sampling...")

        deadline = time.monotonic() + SAMPLE_SECONDS
        next_print = time.monotonic()
        largest_per_frame = []   # one value per frame: the largest qualifying contour
        frames_with_no_detection = 0
        frame_count = 0

        while time.monotonic() < deadline:
            frame = cam.capture_array()
            areas = detect_obstacle_contours(frame, cv2)
            frame_count += 1
            if areas:
                largest_per_frame.append(max(areas))
            else:
                frames_with_no_detection += 1

            if time.monotonic() >= next_print:
                remaining = deadline - time.monotonic()
                print(f"  {remaining:4.1f}s remaining... "
                      f"({len(largest_per_frame)}/{frame_count} frames detected something)")
                next_print = time.monotonic() + 1.0
            time.sleep(1.0 / SAMPLE_HZ)

        print("\n" + "=" * 60)
        if not largest_per_frame:
            print("NO CONTOUR DETECTED in any frame during the sample window.")
            print("Before trying again, check:")
            print("  - Is the obstacle actually in the forward camera's view "
                  "(not the downward line camera)?")
            print("  - Is it dark enough against the tile (detection looks for "
                  "dark blobs on a lighter background)?")
            print("  - Is it tall/wide enough on screen (needs >=30px tall, "
                  ">=20px wide, >=1500px^2 area at whatever distance it's at)?")
            return

        areas_arr = np.array(largest_per_frame)
        median_area = float(np.median(areas_arr))
        mean_area   = float(areas_arr.mean())
        std_area    = float(areas_arr.std())

        print(f"Detected in {len(largest_per_frame)}/{frame_count} frames "
              f"({frames_with_no_detection} missed)")
        print(f"Contour area — median: {median_area:.0f}  mean: {mean_area:.0f}  "
              f"std dev: {std_area:.0f}  min: {areas_arr.min():.0f}  "
              f"max: {areas_arr.max():.0f}")
        print("=" * 60)
        print(f"\nPaste into vision/vision_thread.py:")
        print(f"  AREA_AT_18CM = {median_area:.0f}")
        print("=" * 60)

        if std_area / median_area > 0.25:
            print("\n  NOTE: high variation between frames (relative to the "
                  "median) — check the obstacle and camera stayed still "
                  "during sampling, and that lighting isn't flickering/"
                  "changing. A noisy reading here means less reliable "
                  "distance estimates later.")
        if frames_with_no_detection / frame_count > 0.2:
            print(f"\n  NOTE: missed detection on {frames_with_no_detection} of "
                  f"{frame_count} frames — if this happens at the actual "
                  "18cm test distance, the same obstacle may be missed "
                  "outright at further distances during a real run (contour "
                  "only gets smaller as it moves away).")

    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        cam.stop()
        log.info("Camera stopped")


if __name__ == "__main__":
    main()
