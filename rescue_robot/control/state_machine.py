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
BASE_SPEED           = 80.0
RECOVERY_SPEED       = 35.0
SEARCH_ROTATE_SPEED  = 28.0
OBSTACLE_ARC_SPEED   = 40.0
SPILL_APPROACH_SPEED = 45.0

OBSTACLE_THRESHOLD_CM   = 18.0
OBSTACLE_ARC_DURATION_S = 3.5   # MAX arc time — normally exits early on line reacquisition
OBSTACLE_ARC_MIN_S      = 1.2   # min arc time before reacquisition exit is allowed
                                 # (must be long enough to have left the original line)
MOTOR_ERROR_CHECK_S     = 1.0   # how often to poll BLDC ERROR1/ERROR2 registers

MARKER_TURN_DURATION_S  = 0.35
MARKER_TURN_SPEED       = 38.0
U_TURN_DURATION_S       = 0.70   # tune on floor — should be ~2x MARKER_TURN_DURATION_S
MARKER_COOLDOWN_S       = 1.5    # ignore markers for this long after a turn completes

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
        self._lost_timeout_logged = False
        self._search_direction   = 1.0
        self._marker_turn_dir    = 1.0
        self._marker_is_uturn    = False
        self._marker_cooldown_until = 0.0
        # True once the marker has been observed ABSENT since the last turn.
        # A new turn requires BOTH: cooldown elapsed AND marker seen to
        # disappear at least once — a timer alone re-fires on a marker that
        # simply stays in view longer than the cooldown.
        self._marker_rearmed        = True
        self._marker_turn_start  = 0.0
        self._spill_cross_start  = 0.0
        self._running            = False

    def run(self) -> None:
        log.info("[StateMachine] Starting at %d Hz", LOOP_HZ)
        self._running = True
        next_error_check = time.monotonic() + MOTOR_ERROR_CHECK_S
        can_check_errors = hasattr(self.motors, "check_errors")
        try:
            while self._running:
                tick_start = time.monotonic()
                v = self.vision.snapshot()
                self._tick(v)
                # Periodic BLDC fault poll (overcurrent/overtemp). A stalled
                # M2006 commanded at cruise speed will overheat — this is the
                # only place the ERROR registers are ever read.
                if can_check_errors and tick_start >= next_error_check:
                    try:
                        self.motors.check_errors()
                    except Exception as e:
                        log.warning("[StateMachine] check_errors failed: %s", e)
                    next_error_check = tick_start + MOTOR_ERROR_CHECK_S
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
        if not v.green_marker:
            self._marker_rearmed = True   # marker gone — allow the next one
        elif self._marker_rearmed and time.monotonic() > self._marker_cooldown_until:
            self._transition(RobotState.INTERSECTION)
            return
        turn = self.pid.compute(v.line_error)
        self.motors.drive(BASE_SPEED, turn)

    def _state_recovering(self, v):
        if v.obstacle_detected and v.obstacle_distance_cm < OBSTACLE_THRESHOLD_CM:
            self._obstacle_arc_start = time.monotonic()
            self._transition(RobotState.OBSTACLE_AVOID)
            return
        if v.line_state == LineState.TRACKING:
            self.pid.reset()
            self._transition(RobotState.LINE_FOLLOWING)
            return
        if v.line_state == LineState.LOST:
            self._lost_since = time.monotonic()
            self._search_direction = 1.0 if v.last_valid_error > 0 else -1.0
            self._transition(RobotState.LOST)
            return
        if not v.green_marker:
            self._marker_rearmed = True   # marker gone — allow the next one
        elif self._marker_rearmed and time.monotonic() > self._marker_cooldown_until:
            self._transition(RobotState.INTERSECTION)
            return
        turn = self.pid.compute(v.line_error)
        self.motors.drive(RECOVERY_SPEED, turn)

    def _state_lost(self, v):
        if v.line_state != LineState.LOST:
            self.pid.reset()
            self._lost_timeout_logged = False
            self._transition(RobotState.LINE_FOLLOWING)
            return
        if time.monotonic() - self._lost_since > LOST_SEARCH_TIMEOUT_S:
            # Do NOT exit the program. Under the rules a Lack of Progress means
            # the Robot Handler repositions the robot at a Start Location and
            # restarts it (6.4.3 / 2.8.5) — the program must still be alive and
            # watching for the line when that happens. Stop the motors and wait;
            # the line reappearing under the camera re-enters LINE_FOLLOWING
            # via the check at the top of this handler.
            if not self._lost_timeout_logged:
                log.warning("[LOST] Search timeout — motors stopped, waiting "
                            "for handler reposition (LoP)")
                self._lost_timeout_logged = True
            self.motors.stop()
            return
        if self._search_direction > 0:
            self.motors.rotate_right(SEARCH_ROTATE_SPEED)
        else:
            self.motors.rotate_left(SEARCH_ROTATE_SPEED)

    def _state_intersection(self, v):
        if v.double_marker:
            # Both sides — U-turn (180°)
            self.motors.stop()
            self._marker_turn_dir   = 1.0   # U-turn direction arbitrary, pick right
            self._marker_is_uturn   = True
            self._marker_turn_start = time.monotonic()
            self._transition(RobotState.MARKER_TURN)
            log.info("[INTERSECTION] Double marker → U-turn")
        elif v.green_marker:
            # Single marker — turn toward marker side
            self.motors.stop()
            self._marker_turn_dir   = 1.0 if v.green_marker_x > 0 else -1.0
            self._marker_is_uturn   = False
            self._marker_turn_start = time.monotonic()
            direction = "right" if self._marker_turn_dir > 0 else "left"
            self._transition(RobotState.MARKER_TURN)
            log.info(f"[INTERSECTION] Single marker x={v.green_marker_x:+.2f} → {direction}")
        else:
            self.pid.reset()
            self._transition(RobotState.LINE_FOLLOWING)
            log.info("[INTERSECTION] No marker → straight")

            
    def _state_marker_turn(self, v):
        elapsed  = time.monotonic() - self._marker_turn_start
        duration = U_TURN_DURATION_S if self._marker_is_uturn else MARKER_TURN_DURATION_S

        if elapsed < duration:
            if self._marker_turn_dir > 0:
                self.motors.rotate_right(MARKER_TURN_SPEED)
            else:
                self.motors.rotate_left(MARKER_TURN_SPEED)
        else:
            self.pid.reset()
            self._marker_cooldown_until = time.monotonic() + MARKER_COOLDOWN_S
            self._marker_rearmed = False   # require the marker to disappear
                                            # before another turn can trigger
            self._transition(RobotState.LINE_FOLLOWING)

    def _state_obstacle_avoid(self, v):
        elapsed = time.monotonic() - self._obstacle_arc_start
        # Closed-loop exit (rule 2.4.7: reacquire the line within 30cm of the
        # obstacle): once past a minimum arc time — long enough to have left
        # the original line so we don't instantly "reacquire" the line we
        # were already on — exit as soon as the detector sees the line again.
        if elapsed >= OBSTACLE_ARC_MIN_S and v.line_state in (
                LineState.TRACKING, LineState.RECOVERING):
            self.pid.reset()
            self._transition(RobotState.LINE_FOLLOWING)
            log.info("[OBSTACLE] Line reacquired after %.1fs arc", elapsed)
            return
        if elapsed < OBSTACLE_ARC_DURATION_S:
            self.motors.set_speeds(OBSTACLE_ARC_SPEED, OBSTACLE_ARC_SPEED * 0.3)
        else:
            # Max arc time exhausted without seeing the line — fall back to
            # line following; the LOST handler will take over if needed.
            self.pid.reset()
            self._transition(RobotState.LINE_FOLLOWING)
            log.warning("[OBSTACLE] Arc timed out without reacquiring line")

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
