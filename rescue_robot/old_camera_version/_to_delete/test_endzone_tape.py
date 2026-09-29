"""
test_endzone_tape.py
=====================
Calibration helper for the silver reflective end-zone tape ("spill tape"
in line_detector.py). Goal: find a brightness threshold that the tape is
GUARANTEED to exceed and the plain tile is GUARANTEED to stay below — not
just a rough average difference — so detection never misses the tape and
never false-triggers on a bright spot on the tile.

How it does that: instead of one snapshot per phase, it samples
continuously for several seconds while YOU move the camera around, so it
captures the actual RANGE each surface produces (glare spots, shadows,
different distances/angles) rather than one lucky/unlucky frame.

Step 1: point camera at plain tile, move it around over different tile
        spots (including any shiny/bright patches, edges, corners under
        different lighting) for the sample window — this finds the
        BRIGHTEST the tile ever gets.
Step 2: point camera at the silver tape, move/tilt it through the range
        of angles the robot might actually see it at on approach — this
        finds the DIMMEST the tape ever gets.

It then prints the worst-case numbers and a threshold that sits strictly
between them, with a margin on both sides. If the two ranges overlap (tile
sometimes brighter than tape), it tells you clearly instead of pretending
a safe threshold exists.

Run:
    python3 test_endzone_tape.py
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

from vision.line_detector import (
    LineDetector, load_calibration,
    TAPE_MIN_ROW_FRACTION, TAPE_SEARCH_FRAC, TAPE_MIN_CONSECUTIVE,
)

LINE_CAM_INDEX = 0
FRAME_WIDTH  = 640
FRAME_HEIGHT = 480
CALIBRATION_PATH = "calibration.json"

SAMPLE_SECONDS   = 5.0   # move the camera around for this long each phase
SAMPLE_HZ        = 10
PRINT_HZ         = 5

# Percentiles instead of true min/max — true max/min are just single-pixel
# sensor noise, which would make the threshold overly paranoid. p99/p1
# still capture real worst-case brightness spots while ignoring one-pixel
# outliers.
TILE_WORST_PERCENTILE = 99   # brightest realistic tile reading
TAPE_WORST_PERCENTILE = 1    # dimmest realistic tape reading

# Extra margin subtracted from the gap, split evenly toward each side, so
# the chosen threshold isn't sitting right on the measured edge.
SAFETY_MARGIN = 6.0


def sample_region(cam, search_frac, seconds, label):
    """Capture continuously for `seconds` while the user moves the camera,
    returning every pixel from the search-band ROI across all frames
    concatenated into one array — so percentiles reflect the full range
    seen during the whole window, not just one frame."""
    print(f"  Sampling for {seconds:.0f}s — move the camera around now "
          f"({label})...")
    deadline = time.monotonic() + seconds
    next_print = time.monotonic()
    all_pixels = []
    frame_count = 0
    while time.monotonic() < deadline:
        frame = cam.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        y_start = int(h * search_frac[0])
        roi = gray[y_start:, :]
        all_pixels.append(roi.flatten())
        frame_count += 1
        if time.monotonic() >= next_print:
            remaining = deadline - time.monotonic()
            print(f"    {remaining:4.1f}s remaining... ({frame_count} frames so far)")
            next_print = time.monotonic() + 1.0
        time.sleep(1.0 / SAMPLE_HZ)
    return np.concatenate(all_pixels)


def diagnostic_values(gray, tile_brightness, delta, absolute_min):
    h, w = gray.shape
    y_start = int(h * TAPE_SEARCH_FRAC[0])
    roi = gray[y_start:, :]
    threshold_brightness = min(tile_brightness + delta, absolute_min)
    bright_fraction = (roi > threshold_brightness).sum(axis=1) / w
    max_row_frac = float(bright_fraction.max()) if bright_fraction.size else 0.0
    consecutive = best_consecutive = 0
    for frac in bright_fraction:
        if frac >= TAPE_MIN_ROW_FRACTION:
            consecutive += 1
            best_consecutive = max(best_consecutive, consecutive)
        else:
            consecutive = 0
    return threshold_brightness, max_row_frac, best_consecutive


def main():
    load_calibration(CALIBRATION_PATH)

    try:
        from picamera2 import Picamera2
    except ImportError:
        log.error("picamera2 not installed. Run: sudo apt install python3-picamera2 -y")
        return

    cam = Picamera2(LINE_CAM_INDEX)
    config = cam.create_preview_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "BGR888"}
    )
    cam.configure(config)
    cam.start()
    log.info(f"Camera started on CAM{LINE_CAM_INDEX}")

    try:
        # --- Step 1: worst-case tile brightness ---
        input(f"\nStep 1/2: get ready to move the camera over PLAIN TILE "
              f"(different spots/lighting, no tape) for {SAMPLE_SECONDS:.0f}s. "
              "ENTER to start...")
        tile_pixels = sample_region(cam, TAPE_SEARCH_FRAC, SAMPLE_SECONDS, "plain tile")
        tile_worst = float(np.percentile(tile_pixels, TILE_WORST_PERCENTILE))
        tile_mean  = float(tile_pixels.mean())
        print(f"  Tile: mean={tile_mean:.1f}  "
              f"p{TILE_WORST_PERCENTILE} (worst-case brightest)={tile_worst:.1f}  "
              f"true max={tile_pixels.max()}")

        # --- Step 2: worst-case tape dimness ---
        input(f"\nStep 2/2: get ready to move/tilt the SILVER TAPE through "
              f"different angles in view for {SAMPLE_SECONDS:.0f}s. "
              "ENTER to start...")
        tape_pixels = sample_region(cam, TAPE_SEARCH_FRAC, SAMPLE_SECONDS, "silver tape")
        tape_worst = float(np.percentile(tape_pixels, TAPE_WORST_PERCENTILE))
        tape_mean  = float(tape_pixels.mean())
        print(f"  Tape: mean={tape_mean:.1f}  "
              f"p{TAPE_WORST_PERCENTILE} (worst-case dimmest)={tape_worst:.1f}  "
              f"true min={tape_pixels.min()}")

        # --- Find a safe separating threshold ---
        gap = tape_worst - tile_worst
        print("\n" + "=" * 60)
        print(f"Worst-case tile brightness (brightest it ever got): {tile_worst:.1f}")
        print(f"Worst-case tape brightness (dimmest it ever got):   {tape_worst:.1f}")
        print(f"Separation gap: {gap:.1f}")
        print("=" * 60)

        if gap <= SAFETY_MARGIN * 2:
            print("\n  NO SAFE THRESHOLD FOUND. The tile's brightest reading and "
                  "the tape's dimmest reading are too close together (or "
                  "overlapping) — any single brightness cutoff will either "
                  "miss the tape sometimes or false-trigger on the tile "
                  "sometimes. This is a lighting/angle problem, not a "
                  "constants problem:")
            print("    - Check the tape is catching a real specular "
                  "reflection at the angle the robot actually approaches it "
                  "(foil tape brightness is very angle-dependent)")
            print("    - Check for glare/hot spots on the plain tile during "
                  "step 1 (direct light reflecting off tile) — those are "
                  "what's eating into the gap")
            print("    - Try again after fixing venue lighting/tape angle")
        else:
            threshold = round(tile_worst + gap / 2, 1)
            threshold = min(threshold, 255.0)
            print(f"\nSAFE THRESHOLD (sits in the middle of the gap): {threshold}")
            print("\nPaste into vision/line_detector.py:")
            print(f"  TAPE_ABSOLUTE_MIN     = {threshold}")
            print(f"  TAPE_BRIGHTNESS_DELTA = 255.0   "
                  "# large on purpose — see note below")
            print("\nWhy TAPE_BRIGHTNESS_DELTA is set large: the actual pixel "
                  "threshold used at runtime is min(tile_brightness + DELTA, "
                  "ABSOLUTE_MIN). Setting DELTA large makes ABSOLUTE_MIN "
                  "always win regardless of how bright the current tile "
                  "happens to be — which is what you want here, since "
                  f"{threshold} was calculated to separate tile from tape "
                  "directly, not as an offset from the current tile reading.")
            print(f"\nMargin either side of the actual edge: {SAFETY_MARGIN}")

        print("\nOther two knobs (not measurable this way — depend on tape "
              "geometry/camera framing, not brightness):")
        print(f"  TAPE_MIN_ROW_FRACTION = {TAPE_MIN_ROW_FRACTION}  "
              "(lower if tape doesn't span most of the frame width)")
        print(f"  TAPE_MIN_CONSECUTIVE  = {TAPE_MIN_CONSECUTIVE}  "
              "(lower if the tape band is narrow / seen at a steep angle)")

        if gap <= SAFETY_MARGIN * 2:
            return   # no point live-monitoring with an unsafe threshold

        # --- Live monitor with the computed threshold ---
        print("\nLive monitor using the threshold above (Ctrl-C to stop, "
              "nothing written to any file)...\n")
        detector = LineDetector(FRAME_WIDTH, FRAME_HEIGHT, debug=False)
        last_print = 0.0
        print_interval = 1.0 / PRINT_HZ
        while True:
            frame = cam.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            now = time.monotonic()
            if now - last_print >= print_interval:
                last_print = now
                thr, max_row_frac, best_consecutive = diagnostic_values(
                    gray, 0.0, 255.0, threshold   # DELTA maxed out → ABSOLUTE_MIN always wins
                )
                fires = best_consecutive >= TAPE_MIN_CONSECUTIVE
                flag = " <-- WOULD DETECT" if fires else ""
                print(f"threshold={thr:.1f}  max_row_bright_fraction="
                      f"{max_row_frac:.3f}  consecutive_rows={best_consecutive}"
                      f"{flag}")

    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        cam.stop()
        log.info("Camera stopped")


if __name__ == "__main__":
    main()
