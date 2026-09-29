"""
test_spill_webcam.py
=====================
Standalone test for silver spill-tape detection, using a normal laptop
webcam via OpenCV.

Detection method: measured with calibrate_spill_webcam.py -- average
color/brightness is nearly identical between tape and tile, but the
brightness STD DEVIATION over a whole patch is very different (tape
~37, tile ~13 in the real measurement) because the tape's crinkled,
specular surface is much less uniform than a flat tile.

A per-pixel sliding-window version of this (checking every tiny patch of
the whole frame) ended up just detecting every edge in a cluttered scene,
since any sharp edge also has high local variance. The robot's camera
will only ever be looking at the track floor directly ahead of it though
-- not a cluttered room -- so instead this watches ONE fixed region (a
box representing "the floor right ahead") and checks the std-dev of
brightness inside just that region, matching exactly how
calibrate_spill_webcam.py measured it.

    pip install opencv-python
    python test_spill_webcam.py

Controls:
    [ / ] = lower/raise STD_DEV_THRESHOLD live
    q     = quit
"""

import cv2
import numpy as np

CAM_INDEX = 0

BOX_SIZE = 150   # same size box as calibrate_spill_webcam.py, centered in frame

STD_DEV_THRESHOLD = 22.0   # measured: tape ~37, tile ~13 -- 22 sits between them


def main():
    global STD_DEV_THRESHOLD
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"Could not open webcam at index {CAM_INDEX}")
        return

    print("Webcam opened. Put the region to check (tape/tile) inside the box.")
    print("Press 'q' to quit, '[' / ']' to lower/raise the threshold.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame.")
            break

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        half = BOX_SIZE // 2
        x1, y1, x2, y2 = cx - half, cy - half, cx + half, cy + half

        roi = frame[y1:y2, x1:x2]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        std_dev = float(np.std(gray))
        seen = std_dev >= STD_DEV_THRESHOLD

        box_color = (0, 0, 255) if seen else (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)

        cv2.putText(frame, f"seen={seen}  std_dev={std_dev:.1f}  threshold={STD_DEV_THRESHOLD:.0f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 0) if seen else (0, 0, 255), 2)
        cv2.putText(frame, "[ / ] to lower/raise threshold",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        cv2.imshow("Spill tape test (press q to quit)", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('['):
            STD_DEV_THRESHOLD = max(1.0, STD_DEV_THRESHOLD - 1.0)
            print(f"STD_DEV_THRESHOLD = {STD_DEV_THRESHOLD}")
        elif key == ord(']'):
            STD_DEV_THRESHOLD += 1.0
            print(f"STD_DEV_THRESHOLD = {STD_DEV_THRESHOLD}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
