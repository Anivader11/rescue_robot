"""
calibrate_spill_webcam.py
==========================
Calibration helper (laptop webcam) to measure the actual difference
between the silver spill tape and the white tile, so spill_detector.py's
thresholds can be set from real numbers instead of guessing.

A box is drawn in the middle of the frame. Put the material inside the
box and press 'c' to capture a sample -- it prints the average
brightness, RGB, and HSV (hue/saturation/value) over everything inside
the box. Do this once for the silver tape and once for the white tile,
and it prints the difference between the two at the end.

    pip install opencv-python
    python calibrate_spill_webcam.py

Controls:
    c = capture a sample for the current step
    r = redo the current step (capture again)
    q = quit
"""

import cv2
import numpy as np

CAM_INDEX = 0

BOX_SIZE = 150   # pixels, the sample box is BOX_SIZE x BOX_SIZE in the center


def sample_stats(frame, box):
    x1, y1, x2, y2 = box
    roi = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    b, g, r = cv2.mean(roi)[:3]
    h, s, v = cv2.mean(hsv)[:3]
    brightness = float(np.mean(gray))
    # Foil/crinkled tape has specular hotspots next to darker creases, so it
    # is much less UNIFORM than a matte tile even when the average color is
    # similar. std_dev captures that texture difference.
    std_dev = float(np.std(gray))
    max_val = float(np.max(gray))
    min_val = float(np.min(gray))

    return {
        "R": r, "G": g, "B": b,
        "H": h, "S": s, "V": v,
        "brightness": brightness,
        "std_dev": std_dev,
        "max": max_val,
        "min": min_val,
        "range": max_val - min_val,
    }


def print_stats(label, stats):
    print(f"\n--- {label} ---")
    print(f"  RGB:        R={stats['R']:.1f}  G={stats['G']:.1f}  B={stats['B']:.1f}")
    print(f"  HSV:        H={stats['H']:.1f}  S={stats['S']:.1f}  V={stats['V']:.1f}")
    print(f"  Brightness: {stats['brightness']:.1f}")


def main():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"Could not open webcam at index {CAM_INDEX}")
        return

    steps = [
        ("SILVER TAPE", None),
        ("WHITE TILE", None),
    ]
    step_idx = 0

    print("Place the SILVER TAPE inside the box, then press 'c' to capture.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame.")
            break

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        half = BOX_SIZE // 2
        box = (cx - half, cy - half, cx + half, cy + half)

        cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)

        if step_idx < len(steps):
            label = steps[step_idx][0]
            cv2.putText(frame, f"Step {step_idx+1}/2: place {label} in box, press 'c'",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            cv2.putText(frame, "Done -- press 'q' to quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Spill calibration (c=capture, r=redo, q=quit)", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c') and step_idx < len(steps):
            label, _ = steps[step_idx]
            stats = sample_stats(frame, box)
            steps[step_idx] = (label, stats)
            print_stats(label, stats)
            step_idx += 1
            if step_idx < len(steps):
                print(f"\nNow place the {steps[step_idx][0]} inside the box, then press 'c' to capture.")
            else:
                tape_stats = steps[0][1]
                tile_stats = steps[1][1]
                print("\n=== DIFFERENCE (tape - tile) ===")
                for key_name in ("R", "G", "B", "H", "S", "V", "brightness", "std_dev", "max", "min", "range"):
                    diff = tape_stats[key_name] - tile_stats[key_name]
                    print(f"  {key_name:10s}: tape={tape_stats[key_name]:.1f}  "
                          f"tile={tile_stats[key_name]:.1f}  diff={diff:+.1f}")
                print("\nUse whichever channel has the biggest, most reliable gap "
                      "as the threshold in spill_detector.py.")
        elif key == ord('r') and step_idx > 0:
            step_idx -= 1
            label, _ = steps[step_idx]
            steps[step_idx] = (label, None)
            print(f"\nRedo: place the {label} inside the box, then press 'c' to capture.")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
