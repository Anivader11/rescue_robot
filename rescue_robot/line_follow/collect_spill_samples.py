"""
collect_spill_samples.py
=========================
Step 1 of the trained spill-tape classifier. Same idea as
calibrate_spill_webcam.py (a box in the center of the frame, put a
material in it), but instead of just printing one measurement, this
saves a labeled row of features to a CSV every time you capture a
sample -- so you can build up a real training set instead of eyeballing
one threshold from one sample.

Run on your laptop:
    pip install opencv-python numpy
    python collect_spill_samples.py

Controls:
    s = save current box content as a SILVER TAPE sample
    t = save current box content as a WHITE TILE sample
    q = quit

Move the tape/tile around, change the angle, change the lighting, and
capture MANY samples of each (aim for 20-30+ each) before training --
more variety here means the trained model works on more real conditions
instead of just one exact spot/lighting you tested.
"""

import csv
import os
import cv2
import numpy as np

CAM_INDEX = 0
BOX_SIZE = 150
CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "spill_samples.csv")

FEATURE_NAMES = ["R", "G", "B", "H", "S", "V", "brightness", "std_dev", "max", "min", "range"]


def extract_features(roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    b, g, r = cv2.mean(roi)[:3]
    h, s, v = cv2.mean(hsv)[:3]
    brightness = float(np.mean(gray))
    std_dev = float(np.std(gray))
    max_val = float(np.max(gray))
    min_val = float(np.min(gray))

    return {
        "R": r, "G": g, "B": b,
        "H": h, "S": s, "V": v,
        "brightness": brightness,
        "std_dev": std_dev,
        "max": max_val, "min": min_val,
        "range": max_val - min_val,
    }


def main():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"Could not open webcam at index {CAM_INDEX}")
        return

    file_exists = os.path.exists(CSV_PATH)
    csv_file = open(CSV_PATH, "a", newline="")
    writer = csv.writer(csv_file)
    if not file_exists:
        writer.writerow(FEATURE_NAMES + ["label"])

    counts = {"silver": 0, "tile": 0}
    if file_exists:
        with open(CSV_PATH, newline="") as f:
            for row in csv.DictReader(f):
                if row["label"] in counts:
                    counts[row["label"]] += 1

    print(f"Saving to {CSV_PATH}")
    print(f"Existing samples: silver={counts['silver']}  tile={counts['tile']}")
    print("Move the material around / change lighting between captures for variety.")
    print("Press 's' = save SILVER sample, 't' = save TILE sample, 'q' = quit.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame.")
            break

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        half = BOX_SIZE // 2
        x1, y1, x2, y2 = cx - half, cy - half, cx + half, cy + half

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"silver={counts['silver']}  tile={counts['tile']}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(frame, "s=save silver  t=save tile  q=quit",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        cv2.imshow("Collect spill samples", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key in (ord('s'), ord('t')):
            label = "silver" if key == ord('s') else "tile"
            roi = frame[y1:y2, x1:x2]
            feats = extract_features(roi)
            writer.writerow([feats[name] for name in FEATURE_NAMES] + [label])
            csv_file.flush()
            counts[label] += 1
            print(f"Saved {label} sample #{counts[label]}")

    csv_file.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
