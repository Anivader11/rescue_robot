"""
test_green_webcam.py
=====================
Standalone test for the green-marker detection logic, using a normal
laptop webcam via OpenCV instead of the Pi's Picamera2. Use this to check
the HSV thresholds and left/right side logic actually work, independent
of the Pi camera hardware.

This is NOT part of the robot -- it's just for testing on your laptop.
Run it on Windows/Mac/Linux with a normal USB/built-in webcam:

    pip install opencv-python
    python test_green_webcam.py

Press 'q' to quit. A window shows the camera feed with the detected
green contour outlined and which side it's on printed in the corner.
"""

import cv2

CAM_INDEX = 0   # change if you have multiple webcams and the wrong one opens

MIN_GREEN_AREA = 300

GREEN_H_LOW,  GREEN_H_HIGH  = 38,  85
GREEN_S_LOW,  GREEN_S_HIGH  = 60, 255
GREEN_V_LOW,  GREEN_V_HIGH  = 40, 255


def main():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"Could not open webcam at index {CAM_INDEX}")
        return

    print("Webcam opened. Press 'q' in the video window to quit.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame.")
            break

        h, w = frame.shape[:2]
        half_w = w / 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            (GREEN_H_LOW, GREEN_S_LOW, GREEN_V_LOW),
            (GREEN_H_HIGH, GREEN_S_HIGH, GREEN_V_HIGH)
        )
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_GREEN_AREA or area <= best_area:
                continue
            best_area = area
            best = cnt

        side = None
        if best is not None:
            M = cv2.moments(best)
            cx = M["m10"] / M["m00"] if M["m00"] > 0 else half_w
            cy = M["m01"] / M["m00"] if M["m00"] > 0 else h / 2
            side = "right" if cx > half_w else "left"

            cv2.drawContours(frame, [best], -1, (0, 255, 0), 2)
            cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)

        cv2.line(frame, (int(half_w), 0), (int(half_w), h), (255, 255, 255), 1)
        cv2.putText(frame, f"side={side}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.imshow("Green marker test (press q to quit)", frame)
        cv2.imshow("mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
