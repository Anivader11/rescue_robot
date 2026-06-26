"""
state_machine.py
================
Main control loop at 50 Hz. Implements the line-following flowchart.

States:
    LINE_FOLLOWING  — PID on line error
    RECOVERING      — Line in mid/far ROI only (gap / Logical Pool 2)
    LOST            — No line anywhere, slow rotate to search
    INTERSECTION    — Cross-line detected, check green marker
    MARKER_TURN     — Executing marker-directed turn
    OBSTACLE_AVOID  — Arcing around obstacle
    ENTERING_SPILL  — Reflective tape detected, crossing threshold
    DONE            — Hand off to endzone logic
"""

import time
import logging
from enum import Enum, auto
from control.shared_vision import SharedVision
from control.motors import MotorDriver, PIDController
from vision.line_detector import LineState

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Tunables
# ---------------------------------------------------------------------------
BASE_SPEED           = 40.0
RECOVERY_SPEED       = 35.0
SEARCH_ROTATE_SPEED  = 28.0
OBSTACLE_ARC_SPEED   = 40.0
SPILL_APPROACH_SPEED = 45.0

OBSTACLE_THRESHOLD_CM   = 18.0
OBSTACLE_ARC_DURATION_S = 0.6

MARKER_TURN_DURATION_S  = 0.35
MARKER_TURN_SPEED       = 38.0

SPILL_CROSS_DURATION_S  = 0.5
LOST_SEARCH_TIMEOUT_S   = 4.0

LOOP_HZ    = 50
LOOP_SLEEP = 1.0 / LOOP_HZ


# ---------------------------------------------------------------------------
# Robot states
# ---------------------------------------------------------------------------

class RobotState(Enum):
    LINE_FOLLOWING  = auto()
    RECOVERING      = auto()
    LOST            = auto()
    INTERSECTION    = auto()
    MARKER_TURN     = auto()
    OBSTACLE_AVOID  = auto()
    ENTERING_SPILL  = auto()
    DONE            = auto()


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

class StateMachine:

    def __init__(self, vision: SharedVision, motors: MotorDriver):
        self.vision = vision
        self.motors = motors
        self.pid = PIDController(kp=0.55, ki=0.001, kd=0.20)
        self.state  = RobotState.LINE_FOLLOWING
        self._state_entered_at   = time.monotonic()
        self._obstacle_arc_start = 0.0
        self._lost_since         = 0.0
        self._search_direction   = 1.0
        self._marker_turn_dir    = 1.0
        self._marker_turn_start  = 0.0
        self._spill_cross_start  = 0.0
        self._running            = False

    def run(self) -> None:
        log.info("[StateMachine] Starting at %d Hz", LOOP_HZ)
        self._running = True
        try:
            while self._running:
                tick_start = time.monotonic()
                v = self.vision.snapshot()
                self._tick(v)
                elapsed = time.monotonic() - tick_start
                time.sleep(max(0.0, LOOP_SLEEP - elapsed))
        except KeyboardInterrupt:
            log.info("[StateMachine] Interrupted")
        finally:
            self.motors.stop()
            log.info("[StateMachine] Motors stopped")

    def stop(self) -> None:
        self._running = False

    # ------------------------------------------------------------------
    # Tick
    # ------------------------------------------------------------------

    def _tick(self, v: SharedVision) -> None:
        if v.spill_tape and self.state not in (RobotState.ENTERING_SPILL, RobotState.DONE):
            self._transition(RobotState.ENTERING_SPILL)

        handlers = {
            RobotState.LINE_FOLLOWING: self._state_line_following,
            RobotState.RECOVERING:     self._state_recovering,
            RobotState.LOST:           self._state_lost,
            RobotState.INTERSECTION:   self._state_intersection,
            RobotState.MARKER_TURN:    self._state_marker_turn,
            RobotState.OBSTACLE_AVOID: self._state_obstacle_avoid,
            RobotState.ENTERING_SPILL: self._state_entering_spill,
            RobotState.DONE:           self._state_done,
        }
        handler = handlers.get(self.state)
        if handler:
            handler(v)

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------

    def _state_line_following(self, v):
        if v.obstacle_detected and v.obstacle_distance_cm < OBSTACLE_THRESHOLD_CM:
            self._obstacle_arc_start = time.monotonic()
            self._transition(RobotState.OBSTACLE_AVOID)
            return
        if v.line_state == LineState.LOST:
            self._lost_since = time.monotonic()
            self._search_direction = 1.0 if v.last_valid_error > 0 else -1.0
            self._transition(RobotState.LOST)
            return
        if v.line_state == LineState.RECOVERING:
            self._transition(RobotState.RECOVERING)
            return
        if v.intersection:
            self._transition(RobotState.INTERSECTION)
            return
        turn = self.pid.compute(v.line_error)
        self.motors.drive(BASE_SPEED, turn)

    def _state_recovering(self, v):
        if v.line_state == LineState.TRACKING:
            self.pid.reset()
            self._transition(RobotState.LINE_FOLLOWING)
            return
        if v.line_state == LineState.LOST:
            self._lost_since = time.monotonic()
            self._search_direction = 1.0 if v.last_valid_error > 0 else -1.0
            self._transition(RobotState.LOST)
            return
        turn = self.pid.compute(v.line_error)
        self.motors.drive(RECOVERY_SPEED, turn)

    def _state_lost(self, v):
        if v.line_state != LineState.LOST:
            self.pid.reset()
            self._transition(RobotState.LINE_FOLLOWING)
            return
        if time.monotonic() - self._lost_since > LOST_SEARCH_TIMEOUT_S:
            log.warning("[LOST] Search timeout — stopping")
            self.motors.stop()
            return
        if self._search_direction > 0:
            self.motors.rotate_right(SEARCH_ROTATE_SPEED)
        else:
            self.motors.rotate_left(SEARCH_ROTATE_SPEED)

def _state_intersection(self, v):
    self.motors.stop()
    if v.green_marker:
        self._marker_turn_dir = 1.0 if v.green_marker_x > 0 else -1.0
        direction = "right" if self._marker_turn_dir > 0 else "left"
        self._marker_turn_start = time.monotonic()
        self._transition(RobotState.MARKER_TURN)
        log.info(f"[INTERSECTION] Green marker at x={v.green_marker_x:+.2f} → turn {direction}")
    else:
        self.pid.reset()
        self._transition(RobotState.LINE_FOLLOWING)
        log.info("[INTERSECTION] No marker → straight")

    def _state_marker_turn(self, v):
        elapsed = time.monotonic() - self._marker_turn_start
        if elapsed < MARKER_TURN_DURATION_S:
            speed = MARKER_TURN_SPEED * self._marker_turn_dir
            self.motors.set_speeds(-speed, speed)
        else:
            self.pid.reset()
            self._transition(RobotState.LINE_FOLLOWING)

    def _state_obstacle_avoid(self, v):
        elapsed = time.monotonic() - self._obstacle_arc_start
        if elapsed < OBSTACLE_ARC_DURATION_S:
            self.motors.set_speeds(OBSTACLE_ARC_SPEED, OBSTACLE_ARC_SPEED * 0.3)
        else:
            self.pid.reset()
            self._transition(RobotState.LINE_FOLLOWING)

    def _state_entering_spill(self, v):
        if self._spill_cross_start == 0.0:
            self._spill_cross_start = time.monotonic()
        elapsed = time.monotonic() - self._spill_cross_start
        if elapsed < SPILL_CROSS_DURATION_S:
            self.motors.drive_straight(SPILL_APPROACH_SPEED)
        else:
            self.motors.stop()
            self._transition(RobotState.DONE)

    def _state_done(self, v):
        self.motors.stop()
        self._running = False
        log.info("[DONE] Course complete")

    # ------------------------------------------------------------------

    def _transition(self, new_state: RobotState) -> None:
        if new_state != self.state:
            log.info("[STATE] %s → %s", self.state.name, new_state.name)
            self.state = new_state
            self._state_entered_at = time.monotonic()
