"""
test_line_detector.py
=====================
Unit tests for LineDetector using synthetic frames.
No camera required.

Run:
    python3 tests/test_line_detector.py
"""

import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from vision.line_detector import LineDetector, LineState

W, H = 640, 480
HALF_W = W // 2
TILE_BRIGHTNESS = 225
LINE_DARKNESS   = 30

def base_frame():
    return np.full((H, W, 3), TILE_BRIGHTNESS, dtype=np.uint8)

def make_line_frame(x_centre, line_width=20):
    frame = base_frame()
    x1 = max(0, x_centre - line_width // 2)
    x2 = min(W, x_centre + line_width // 2)
    frame[:, x1:x2] = LINE_DARKNESS
    return frame

def make_intersection_frame(x_centre):
    frame = make_line_frame(x_centre)
    y_bar = int(H * 0.55)
    frame[y_bar-10:y_bar+10, :] = LINE_DARKNESS
    return frame

def make_gap_frame(x_centre):
    frame = make_line_frame(x_centre)
    frame[int(H*0.70):, :] = TILE_BRIGHTNESS
    return frame

def make_green_marker_frame(x_centre):
    frame = make_line_frame(x_centre)
    frame[50:110, 80:140] = (50, 180, 50)
    return frame

def make_reflective_tape_frame():
    frame = make_line_frame(HALF_W)
    frame[int(H*0.78):int(H*0.85), int(W*0.1):int(W*0.9)] = 252
    return frame

def make_empty_frame():
    return base_frame()


class TestLineDetector:

    def setup_method(self):
        self.det = LineDetector(W, H, debug=False)
        self.det.calibrate(make_line_frame(HALF_W))

    def test_centred_line_error_near_zero(self):
        r = self.det.detect(make_line_frame(HALF_W))
        assert r.state == LineState.TRACKING
        assert abs(r.error) < 0.10
        print(f"  centred: err={r.error:+.4f} ✓")

    def test_line_left_gives_negative_error(self):
        r = self.det.detect(make_line_frame(HALF_W - 120))
        assert r.state == LineState.TRACKING
        assert r.error < -0.25
        print(f"  left: err={r.error:+.4f} ✓")

    def test_line_right_gives_positive_error(self):
        r = self.det.detect(make_line_frame(HALF_W + 120))
        assert r.state == LineState.TRACKING
        assert r.error > 0.25
        print(f"  right: err={r.error:+.4f} ✓")

    def test_error_bounded(self):
        for x in [5, 50, W-50, W-5]:
            r = self.det.detect(make_line_frame(x))
            assert -1.0 <= r.error <= 1.0
        print("  error in [-1,1] ✓")

    def test_empty_frame_eventually_lost(self):
        from vision.line_detector import LOST_PATIENCE_FRAMES
        r = None
        for _ in range(LOST_PATIENCE_FRAMES + 2):
            r = self.det.detect(make_empty_frame())
        assert r.state == LineState.LOST
        print(f"  LOST after {LOST_PATIENCE_FRAMES} frames ✓")

    def test_last_error_held_when_lost(self):
        from vision.line_detector import LOST_PATIENCE_FRAMES
        r = None
        for _ in range(5):
            r = self.det.detect(make_line_frame(HALF_W + 100))
        last_valid = r.error
        for _ in range(LOST_PATIENCE_FRAMES + 1):
            r = self.det.detect(make_empty_frame())
        assert r.state == LineState.LOST
        assert abs(r.last_error - last_valid) < 0.12
        print(f"  last_error held: {r.last_error:+.4f} ✓")

    def test_gap_frame_recovery(self):
        r = self.det.detect(make_gap_frame(HALF_W))
        assert r.state in (LineState.TRACKING, LineState.RECOVERING)
        print(f"  gap → {r.state.name} ✓")

    def test_confidence_nonzero_when_tracking(self):
        r = self.det.detect(make_line_frame(HALF_W))
        assert r.confidence > 0.0
        print(f"  confidence={r.confidence:.3f} ✓")

    def test_confidence_zero_when_lost(self):
        from vision.line_detector import LOST_PATIENCE_FRAMES
        r = None
        for _ in range(LOST_PATIENCE_FRAMES + 2):
            r = self.det.detect(make_empty_frame())
        assert r.confidence == 0.0
        print("  confidence=0 when LOST ✓")

    def test_intersection_detected(self):
        r = self.det.detect(make_intersection_frame(HALF_W))
        assert r.intersection
        assert r.state == LineState.INTERSECTION
        print("  intersection detected ✓")

    def test_no_false_intersection(self):
        r = self.det.detect(make_line_frame(HALF_W))
        assert not r.intersection
        print("  no false intersection ✓")

    def test_green_marker_detected(self):
        r = self.det.detect(make_green_marker_frame(HALF_W))
        assert r.green_marker
        print("  green marker detected ✓")

    def test_no_false_green_marker(self):
        r = self.det.detect(make_line_frame(HALF_W))
        assert not r.green_marker
        print("  no false green marker ✓")

    def test_spill_tape_detected(self):
        r = self.det.detect(make_reflective_tape_frame())
        assert r.spill_tape
        assert r.state == LineState.ENTERING_SPILL
        print("  spill tape detected ✓")

    def test_spill_tape_overrides_line(self):
        r = self.det.detect(make_reflective_tape_frame())
        assert r.state == LineState.ENTERING_SPILL
        print("  tape overrides line state ✓")


def run_all():
    t = TestLineDetector()
    tests = sorted(m for m in dir(t) if m.startswith("test_"))
    passed, failed = 0, []
    print(f"\nRunning {len(tests)} tests...\n")
    for name in tests:
        t.setup_method()
        try:
            getattr(t, name)()
            passed += 1
        except AssertionError as e:
            print(f"  FAIL  {name}: {e}")
            failed.append(name)
        except Exception as e:
            print(f"  ERROR {name}: {type(e).__name__}: {e}")
            failed.append(name)
    print(f"\n{'='*40}")
    print(f"Results: {passed}/{len(tests)} passed")
    if failed:
        print(f"Failed: {failed}")
    else:
        print("All tests passed ✓")
    print(f"{'='*40}\n")
    return len(failed) == 0

if __name__ == "__main__":
    ok = run_all()
    sys.exit(0 if ok else 1)
