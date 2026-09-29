"""
test_spill_classifier_webcam.py
=================================
Step 3 (laptop test): live-test the TRAINED classifier from
train_spill_classifier.py on your webcam, before trusting it on the Pi.

Requires spill_model_weights.py to already exist (run
collect_spill_samples.py then train_spill_classifier.py first).

    python test_spill_classifier_webcam.py

Press 'q' to quit.
"""

import cv2
from spill_classifier import extract_features, predict_silver

CAM_INDEX = 0
BOX_SIZE = 150


def main():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"Could not open webcam at index {CAM_INDEX}")
        return

    print("Webcam opened. Press 'q' to quit.")

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
        feats = extract_features(roi)
        is_silver, confidence = predict_silver(feats)

        box_color = (0, 0, 255) if is_silver else (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
        cv2.putText(frame, f"silver={is_silver}  confidence={confidence*100:.1f}%",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, box_color, 2)

        cv2.imshow("Trained spill classifier test (press q to quit)", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
