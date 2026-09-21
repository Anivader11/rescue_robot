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

EVADE_SPEED = 30.0     # base speed (%) used for straight moves in the sequence
TURN_SPEED = 30.0      # speed (%) used for the turn-in-place steps

# (left_speed, right_speed, duration_s)
EVADE_SEQUENCE = [
    ("back",     -EVADE_SPEED, -EVADE_SPEED, 0.3),   # little bit back
    ("right",     TURN_SPEED,  -TURN_SPEED,  0.4),   # turn right
    ("up",        EVADE_SPEED,  EVADE_SPEED, 0.5),   # forward, around the obstacle
    ("left",     -TURN_SPEED,   TURN_SPEED,  0.4),   # turn left (back toward line heading)
    ("straight",  EVADE_SPEED,  EVADE_SPEED, 0.5),   # forward past the obstacle
    ("left",     -TURN_SPEED,   TURN_SPEED,  0.4),   # turn left
    ("right",     TURN_SPEED,  -TURN_SPEED,  0.4),   # turn right, straighten back onto the line
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
