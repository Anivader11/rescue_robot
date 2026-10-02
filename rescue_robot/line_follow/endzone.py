"""
endzone.py
===========
Evacuation-zone behaviour for RCJA 2026 Open Rescue Line
(2 silver "alive" balls -> GREEN triangle, 1 black "dead" ball -> RED
triangle; triangles sit in corners with 60 mm walls).

Flow per ball (the claw is on the BACK, camera + ToF on the FRONT):
    1. SEARCH    spin in small steps, look for a ball with the camera.
                 Full turn with nothing -> drive forward to a new spot.
    2. APPROACH  steer at the ball; stop when ToF <= BALL_STOP_MM.
                 (If the ball drops out of the bottom of the frame
                 first, keep creeping straight for up to BLIND_CREEP_S.)
    3. PAUSE     sleep BALL_PAUSE_S at that distance.
    4. TURN 180  timed spin (tune TURN_180_S on your floor).
    5. GRAB      on the spot, no reversing: open claw, lower, close, lift.
                 (BALL_STOP_MM must put the ball right where the claw
                 lands after the 180.)
    6. FIND DROP search for the matching triangle (green/red), approach
                 until ToF <= DROP_STOP_MM (same as the ball), pause,
                 turn 180 on the spot (no reversing), lower the claw
                 about halfway, open, lift back.
    7. Drive clear, repeat until MAX_BALLS done or the time budget runs out.

Exiting the zone and reacquiring the line isn't built yet -- next stage.

Entry point:
    run_endzone(motors, cam_watcher, tof_detector, max_balls=3)
Called from pid_line_follow.py once the silver tape is confirmed.
Standalone test: line_follow/test_endzone.py
"""

import time
import logging

from zone_vision import find_balls, pick_target, find_evac_point
from claw import Claw

log = logging.getLogger(__name__)

FRAME_WIDTH = 320          # must match camera_watcher.py
FRAME_HEIGHT = 240

# --- Overall ---
MAX_BALLS = 3
TIME_BUDGET_S = 200.0      # of the 300 s run -- leaves time for the exit later
ENTRY_DRIVE_S = 0.6        # drive forward off the silver tape into the zone first
ENTRY_SPEED = 25.0
USE_CAMERA_TILT = True     # tilt camera with camera_servo.look_up() while in the zone

# --- Search ---
SEARCH_TURN_SPEED = 22.0
SEARCH_STEP_S = 0.25       # spin this long per step...
SETTLE_S = 0.25            # ...then stop this long so the camera gets a clean frame
SEARCH_STEPS_PER_REV = 14  # steps that make roughly a full 360 (tune)
WANDER_SPEED = 25.0
WANDER_MAX_S = 1.5         # drive to a new spot after a full turn with nothing seen
WALL_MM = 120              # stop wandering when ToF says a wall is this close
MAX_SEARCH_ROUNDS = 4

# --- Approach (shared by ball + drop) ---
APPROACH_SPEED = 20.0
STEER_GAIN = 18.0          # % speed difference at full left/right of frame
FRAME_STALE_S = 0.3
TOF_STALE_S = 0.5
APPROACH_TIMEOUT_S = 8.0
LOST_TIMEOUT_S = 0.8       # target gone this long (and not near bottom) -> search again

# --- Ball ---
BALL_STOP_MM = 60          # TUNE: ToF distance that leaves the ball under the claw after the 180
BALL_PAUSE_S = 0.5
NEAR_BOTTOM_Y = FRAME_HEIGHT * 0.8   # ball this low in frame = about to leave the view
BLIND_CREEP_S = 0.6

# --- 180 turn + grab ---
TURN_SPEED = 30.0
TURN_180_S = 1.2           # TUNE: timed spin that gives exactly 180 deg

# --- Drop ---
DROP_STOP_MM = BALL_STOP_MM   # same ToF stop distance as the ball
DROP_LOWER_FRACTION = 0.5     # only lower the claw about halfway before letting go
DROP_CLEAR_S = 0.8         # drive forward away from the corner afterwards


class _Timeout(Exception):
    pass


class Endzone:
    def __init__(self, motors, cam, tof, max_balls=MAX_BALLS):
        self.m = motors
        self.cam = cam
        self.tof = tof
        self.max_balls = max_balls
        self.claw = Claw()
        self.deadline = time.monotonic() + TIME_BUDGET_S
        self.cam_servo = None

    # ---------- helpers ----------
    def _check_time(self):
        if time.monotonic() > self.deadline:
            raise _Timeout()

    def _frame(self):
        frame, at = self.cam.get_frame()
        if frame is None or time.monotonic() - at > FRAME_STALE_S:
            return None
        return frame

    def _dist(self):
        d, at = self.tof.get_latest()
        if d is None or time.monotonic() - at > TOF_STALE_S:
            return None
        return d

    def _drive_for(self, left, right, seconds):
        self.m.set_speeds(left, right)
        time.sleep(seconds)
        self.m.stop()

    def _turn_180(self):
        print("[endzone] turning 180")
        self._drive_for(TURN_SPEED, -TURN_SPEED, TURN_180_S)
        time.sleep(0.2)

    def _steer(self, cx):
        err = (cx - FRAME_WIDTH / 2) / (FRAME_WIDTH / 2)   # -1 left .. +1 right
        turn = STEER_GAIN * err
        self.m.set_speeds(APPROACH_SPEED + turn, APPROACH_SPEED - turn)

    def _search(self, look):
        """Spin in steps until look(frame) returns something. Returns it or None."""
        for _round in range(MAX_SEARCH_ROUNDS):
            for _ in range(SEARCH_STEPS_PER_REV):
                self._check_time()
                time.sleep(SETTLE_S)
                frame = self._frame()
                if frame is not None:
                    hit = look(frame)
                    if hit is not None:
                        return hit
                self._drive_for(SEARCH_TURN_SPEED, -SEARCH_TURN_SPEED, SEARCH_STEP_S)
            # nothing after a full turn: move somewhere new
            print("[endzone] nothing seen after full turn -- wandering")
            end = time.monotonic() + WANDER_MAX_S
            self.m.set_speeds(WANDER_SPEED, WANDER_SPEED)
            while time.monotonic() < end:
                d = self._dist()
                if d is not None and d <= WALL_MM:
                    break
                time.sleep(0.02)
            self.m.stop()
        return None

    def _approach(self, locate, stop_mm, near_fn):
        """Drive at whatever locate(frame) returns (needs .cx / [0]) until the
        ToF says stop_mm. near_fn(target) -> True if target is about to leave
        the frame / is close enough to finish blind."""
        start = time.monotonic()
        last_seen = start
        last_target = None
        while time.monotonic() - start < APPROACH_TIMEOUT_S:
            self._check_time()
            d = self._dist()
            if d is not None and d <= stop_mm:
                self.m.stop()
                print(f"[endzone] reached target, ToF={d} mm")
                return True

            frame = self._frame()
            target = locate(frame) if frame is not None else None
            if target is not None:
                last_target, last_seen = target, time.monotonic()
                self._steer(target[1] if hasattr(target, "kind") else target[0])
            elif last_target is not None and near_fn(last_target):
                # dropped out of the bottom of the view: creep straight, wait for the ToF
                print("[endzone] target below view -- creeping")
                end = time.monotonic() + BLIND_CREEP_S
                self.m.set_speeds(APPROACH_SPEED * 0.7, APPROACH_SPEED * 0.7)
                while time.monotonic() < end:
                    d = self._dist()
                    if d is not None and d <= stop_mm:
                        break
                    time.sleep(0.02)
                self.m.stop()
                return True      # close enough even if the ToF never tripped
            elif time.monotonic() - last_seen > LOST_TIMEOUT_S:
                self.m.stop()
                print("[endzone] lost target")
                return False
            time.sleep(0.02)
        self.m.stop()
        print("[endzone] approach timed out")
        return False

    # ---------- per-ball steps ----------
    def _get_ball(self):
        target = self._search(lambda f: pick_target(find_balls(f)))
        if target is None:
            return None
        kind = target.kind
        print(f"[endzone] found {kind} ball at x={target.cx:.0f} r={target.r:.0f}")

        ok = self._approach(
            locate=lambda f: pick_target(find_balls(f), kind),
            stop_mm=BALL_STOP_MM,
            near_fn=lambda b: b.cy >= NEAR_BOTTOM_Y,
        )
        if not ok:
            return None

        time.sleep(BALL_PAUSE_S)
        self._turn_180()

        print("[endzone] grabbing")
        self.claw.open()
        self.claw.lower()
        self.claw.close()
        self.claw.lift()
        return kind

    def _drop(self, kind):
        colour = "green" if kind == "alive" else "red"
        print(f"[endzone] looking for {colour} evacuation point")
        hit = self._search(lambda f: find_evac_point(f, colour))
        if hit is None:
            print(f"[endzone] couldn't find {colour} point")
            return False

        ok = self._approach(
            locate=lambda f: find_evac_point(f, colour),
            stop_mm=DROP_STOP_MM,
            near_fn=lambda e: e[1] >= NEAR_BOTTOM_Y,
        )
        if not ok:
            return False

        time.sleep(BALL_PAUSE_S)
        self._turn_180()
        print("[endzone] dropping")
        self.claw.lower(DROP_LOWER_FRACTION)
        self.claw.open()
        time.sleep(0.5)
        self.claw.lift(DROP_LOWER_FRACTION)
        self._drive_for(APPROACH_SPEED, APPROACH_SPEED, DROP_CLEAR_S)
        return True

    # ---------- main ----------
    def run(self):
        rescued = 0
        try:
            if USE_CAMERA_TILT:
                try:
                    from camera_servo import CameraServo
                    self.cam_servo = CameraServo()
                    self.cam_servo.look_up()
                except Exception:
                    log.exception("[endzone] camera tilt failed -- carrying on")

            print("[endzone] entering zone")
            self._drive_for(ENTRY_SPEED, ENTRY_SPEED, ENTRY_DRIVE_S)

            attempts = 0
            while rescued < self.max_balls and attempts < self.max_balls + 3:
                attempts += 1
                kind = self._get_ball()
                if kind is None:
                    print("[endzone] no ball found this attempt")
                    continue
                if self._drop(kind):
                    rescued += 1
                    print(f"[endzone] rescued {rescued}/{self.max_balls}")
        except _Timeout:
            print("[endzone] time budget used up")
        finally:
            self.m.stop()
            self.claw.release()
            if self.cam_servo is not None:
                try:
                    from camera_servo import LINE_VALUE
                    self.cam_servo.set_value(LINE_VALUE)
                    self.cam_servo.release()
                except Exception:
                    pass
        print(f"[endzone] done -- {rescued} ball(s) rescued")
        return rescued


def run_endzone(motors, cam_watcher, tof_detector, max_balls=MAX_BALLS):
    return Endzone(motors, cam_watcher, tof_detector, max_balls).run()
