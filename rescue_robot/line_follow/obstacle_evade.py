"""
obstacle_evade.py
===================
Scripted, timed evasion maneuver run when the ToF sensor (tof_sensor.py /
tof_detector.py) sees an obstacle within OBSTACLE_TRIGGER_MM. Each step
in EVADE_SEQUENCE is (left_speed, right_speed, duration_s) -- same
signs/convention as control/motors.py's BLDCMotorDriver.set_speeds()
(right side positive, left side negative = forward).

Sequence (as requested): little bit back, right, up (forward), left,
straight, left, then right -- then resume normal line following. This
is a fixed timed sequence, not sensor-guided, so the durations/speeds
below are a starting point -- tune EVADE_SEQUENCE against your actual
arena spacing once you've tried it once or twice.

Usage:
    from obstacle_evade import run_evade_sequence
    run_evade_sequence(motors)   # blocks until the whole sequence is done
"""

import time

# --- Speeds (%) -- how fast each kind of move runs ---
EVADE_SPEED = 30.0     # speed for straight moves (back/up/straight)
TURN_SPEED = 30.0      # speed for turn-in-place moves (left/right)

# --- Durations (seconds) -- how LONG each step runs, one variable per
# step so you can tune them individually. Longer duration = more
# distance travelled / more degrees turned, at a given speed above. ---
BACK_DURATION_S = 0.3        # step 1: little bit back, away from the obstacle
RIGHT_TURN_1_DURATION_S = 0.4   # step 2: turn right
FORWARD_1_DURATION_S = 0.5      # step 3: forward, around the obstacle
LEFT_TURN_1_DURATION_S = 0.4    # step 4: turn left (back toward line heading)
FORWARD_2_DURATION_S = 0.5      # step 5: forward, past the obstacle
LEFT_TURN_2_DURATION_S = 0.4    # step 6: turn left
RIGHT_TURN_2_DURATION_S = 0.4   # step 7: turn right, straighten back onto the line

# (name, left_speed, right_speed, duration_s) -- built from the
# variables above, so edit those rather than this list directly.
EVADE_SEQUENCE = [
    ("back",     -EVADE_SPEED, -EVADE_SPEED, BACK_DURATION_S),
    ("right",     TURN_SPEED,  -TURN_SPEED,  RIGHT_TURN_1_DURATION_S),
    ("up",        EVADE_SPEED,  EVADE_SPEED, FORWARD_1_DURATION_S),
    ("left",     -TURN_SPEED,   TURN_SPEED,  LEFT_TURN_1_DURATION_S),
    ("straight",  EVADE_SPEED,  EVADE_SPEED, FORWARD_2_DURATION_S),
    ("left",     -TURN_SPEED,   TURN_SPEED,  LEFT_TURN_2_DURATION_S),
    ("right",     TURN_SPEED,  -TURN_SPEED,  RIGHT_TURN_2_DURATION_S),
]


def run_evade_sequence(motors, sequence=EVADE_SEQUENCE):
    """Blocking: drives through each timed step, then stops. Line
    following resumes on the caller's next loop iteration."""
    print("obstacle detected -- running evade sequence")
    for name, left, right, duration in sequence:
        print(f"  evade step: {name} (L={left:+.1f} R={right:+.1f} for {duration}s)")
        motors.set_speeds(left, right)
        time.sleep(duration)
    motors.stop()
    print("evade sequence complete -- resuming line following")
