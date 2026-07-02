"""
calibrate.py
============
Interactive calibration tool — run ON SITE before competition.

Usage:
    python3 calibrate.py --camera 0

Controls:
    SPACE  = calibrate (point camera at tile first)
    d      = toggle debug overlay
    s      = save to calibration.json
    q      = quit
"""

import cv2
import numpy as np
import json
import argparse
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from vision.line_detector import LineDetector


def nothing(_): pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width",  type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {args.camera}")
        return

    detector   = LineDetector(args.width, args.height, debug=True)
    calibrated = False
    debug_on   = True

    cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Calibration", 700, 400)

    cv2.createTrackbar("Green H low",  "Calibration", 38,  90, nothing)
    cv2.createTrackbar("Green H high", "Calibration", 85, 120, nothing)
    cv2.createTrackbar("Green S low",  "Calibration", 60, 255, nothing)
    cv2.createTrackbar("Green V low",  "Calibration", 40, 255, nothing)
    cv2.createTrackbar("Tape V low",   "Calibration", 210, 255, nothing)
    cv2.createTrackbar("Tape S high",  "Calibration", 40, 255, nothing)
    cv2.createTrackbar("Adaptive C",   "Calibration", 8,  30, nothing)

    print("\n[Calibration] SPACE=calibrate  d=debug  s=save  q=quit\n")
    saved_values = {}

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Frame grab failed")
            break

        gh_low  = cv2.getTrackbarPos("Green H low",  "Calibration")
        gh_high = cv2.getTrackbarPos("Green H high", "Calibration")
        gs_low  = cv2.getTrackbarPos("Green S low",  "Calibration")
        gv_low  = cv2.getTrackbarPos("Green V low",  "Calibration")
        tv_low  = cv2.getTrackbarPos("Tape V low",   "Calibration")
        ts_high = cv2.getTrackbarPos("Tape S high",  "Calibration")
        adapt_c = cv2.getTrackbarPos("Adaptive C",   "Calibration")

        import vision.line_detector as ld
        ld.GREEN_H_LOW  = gh_low
        ld.GREEN_H_HIGH = gh_high
        ld.GREEN_S_LOW  = gs_low
        ld.GREEN_V_LOW  = gv_low
        ld.TAPE_V_LOW   = tv_low
        ld.TAPE_S_HIGH  = ts_high
        ld.ADAPTIVE_C   = adapt_c

        result = detector.detect(frame)
        display = result.debug_frame if (debug_on and result.debug_frame is not None) else frame

        status_colour = {
            "TRACKING":       (0, 255, 0),
            "RECOVERING":     (0, 200, 255),
            "LOST":           (0, 0, 255),
            "INTERSECTION":   (255, 100, 0),
            "ENTERING_SPILL": (255, 255, 255),
        }.get(result.state.name, (200, 200, 200))

        cv2.rectangle(display, (0, 0), (300, 75), (0, 0, 0), -1)
        cv2.putText(display, f"STATE: {result.state.name}", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_colour, 2)
        cv2.putText(display, f"error={result.error:+.3f}  conf={result.confidence:.2f}",
                    (8, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)
        cv2.putText(display, f"green={result.green_marker}  tape={result.spill_tape}",
                    (8, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        if not calibrated:
            cv2.putText(display, "PRESS SPACE TO CALIBRATE",
                        (80, display.shape[0]-15), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 140, 255), 2)

        cv2.imshow("Line Camera", display)

        # Coloured detection overlay
        binary  = detector._threshold(frame)
        overlay = (frame * 0.35).astype("uint8")
        overlay[binary > 0] = (0, 0, 220)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv,
                                  (ld.GREEN_H_LOW, ld.GREEN_S_LOW, ld.GREEN_V_LOW),
                                  (ld.GREEN_H_HIGH, ld.GREEN_S_HIGH, ld.GREEN_V_HIGH))
        overlay[green_mask > 0] = (200, 80, 0)
        h, w = frame.shape[:2]
        cv2.line(overlay, (0, int(h*0.75)), (w, int(h*0.75)), (80, 80, 80), 1)
        cv2.line(overlay, (0, int(h*0.50)), (w, int(h*0.50)), (60, 60, 60), 1)
        cv2.line(overlay, (0, int(h*0.25)), (w, int(h*0.25)), (40, 40, 40), 1)
        cv2.rectangle(overlay, (0, h-48), (210, h), (20, 20, 20), -1)
        cv2.putText(overlay, "RED  = detected line",  (6, h-30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 220), 1)
        cv2.putText(overlay, "BLUE = green excluded", (6, h-12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 80, 0), 1)
        cv2.imshow("Detection overlay", overlay)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' '):
            cal = detector.calibrate(frame)
            if cal["contrast"] <30:
                print(f"[Calibration] Warning Contrast too low ({cal['contrast']}) - improve lighting and recalibrate") 
            else:
                print(f"[Calibration] Done: {cal}")    
            Calibrated = True   
        elif key == ord('d'):
            debug_on = not debug_on
        elif key == ord('s'):
            saved_values = {
                "GREEN_H_LOW": gh_low, "GREEN_H_HIGH": gh_high,
                "GREEN_S_LOW": gs_low, "GREEN_V_LOW": gv_low,
                "TAPE_V_LOW":  tv_low, "TAPE_S_HIGH": ts_high,
                "ADAPTIVE_C":  adapt_c,
                "darkness_bias": detector._darkness_bias,
            }
            with open("calibration.json", "w") as f:
                json.dump(saved_values, f, indent=2)
            print(f"[Calibration] Saved: {saved_values}")

    cap.release()
    cv2.destroyAllWindows()
    if saved_values:
        print("\n--- Paste into line_detector.py ---")
        for k, v in saved_values.items():
            print(f"  {k:20s} = {v}")


if __name__ == "__main__":
    main()
