"""
pid_line_follow.py
===================
Simple PID line follower for 2-wheel drive, using 2 digital IR line
sensors (Parallax 28034-style) instead of the camera.

Sensor power: the line sensor board has an EN (enable) pin -- BCM22 --
that must be driven HIGH before the two OUT pins will report anything
real. Without this, the sensors are simply unpowered.

Sensor pins are claimed with an internal pull-up (lgpio.SET_PULL_UP) --
this is the exact setup confirmed working in sensor_tests/test_line.py.
WHITE -> LOW (0), BLACK -> HIGH (1).

Sensor spacing: the two sensors sit far enough apart that when the
robot is centered on the line, BOTH see white (0, 0) -- the line runs
between them, not under either one. Only one sensor reads black (1)
when the robot has drifted enough for that sensor to catch the line's
edge. So (0,0) and (1,1) both mean "centered, drive straight" (error
0), and there's no separate "line lost" failsafe -- the plain PID math
already does the right thing for every sensor combination.

Green marker handling:
    The shared camera thread (camera_watcher.py) watches for a green
    marker off to one side. When seen, the robot ignores the line
    sensors for a short fixed turn, then resumes normal PID following.

Spill/endzone handling:
    The shared camera thread (camera_watcher.py) also runs a trained
    classifier (see spill_classifier.py / spill_model_weights.py) that
    watches for the silver spill-zone tape. Endzone logic itself isn't
    built yet -- for now, as soon as it's seen, the robot stops and
    double-checks for SPILL_VERIFY_S before trusting it: if the
    classifier still says spill after that pause, it's treated as
    confirmed (prints a message, stops for good, run ends there). If it
    was a false alarm (e.g. a shadow or the black line fooling the
    classifier), the robot just resumes normal line following.
    Once confirmed, endzone.py's run_endzone() takes over (find ball ->
    ToF distance -> pause -> 180 -> grab -> matching triangle -> drop,
    x3). Leaving the zone isn't built yet, so the run still ends after.

Obstacle handling:
    A background thread (tof_detector.py) polls the SteelBar ToF
    distance sensor (tof_sensor.py). The ToF alone can't tell a ramp
    apart from an actual obstacle -- both can read within
    OBSTACLE_TRIGGER_MM -- so as soon as it triggers, the robot grabs
    the latest camera frame (from the shared camera_watcher.py thread)
    and runs bottle_detector.py's shape check on it before doing
    anything. Only if that confirms it looks like the bottle (not a
    flat/angled ramp surface) does it run the fixed timed evasion
    maneuver (obstacle_evade.py: back, right, forward, left, straight,
    left, right). If the camera doesn't confirm it, it's treated as a
    ramp and the ToF trigger is ignored -- normal line following (with
    the accelerometer ramp speed boost below) just continues.

Ramp/IMU handling:
    A background thread (accel_speed_boost.py) polls the BNO085's raw
    Y acceleration (imu_sensor.py). Whenever Y drops below
    ACCEL_Y_BOOST_THRESHOLD (-15 m/s^2 by default -- flat/level sits
    around -9.8 from gravity, so this fires once the robot noses
    upward enough on a ramp), BASE_SPEED is scaled up by
    ACCEL_SPEED_BOOST_MULTIPLIER. Returns to normal speed the instant
    Y comes back above the threshold.
"""

import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import lgpio
from control.motors import BLDCMotorDriver
from camera_watcher import CameraWatcher
from tof_detector import ToFDetector
from obstacle_evade import run_evade_sequence
from bottle_detector import is_bottle
from accel_speed_boost import AccelSpeedBoost
from endzone import run_endzone

EN_PIN = 22    # line sensor array's enable line -- must be driven HIGH or the sensors don't power up at all
LEFT_PIN = 24
RIGHT_PIN = 23

BASE_SPEED = 30.0
Kp = 20.0
Kd = 10.0

GREEN_ACT_DELAY_S = 0.3
GREEN_TURN_DURATION_S = 0.6
GREEN_TURN_SPEED = 30.0
GREEN_COOLDOWN_S = 2.0

# --- Spill/endzone verification tunables ---
SPILL_VERIFY_S = 1.0
SPILL_FALSE_ALARM_COOLDOWN_S = 1.0

# --- Obstacle (ToF) tunables ---
OBSTACLE_TRIGGER_MM = 50
OBSTACLE_STALE_S = 0.5           # ignore ToF readings older than this (sensor stuck/disconnected)
OBSTACLE_COOLDOWN_S = 1.0        # after an evade (or a ramp false-alarm), ignore new triggers briefly
OBSTACLE_FRAME_STALE_S = 0.5     # ignore camera frames older than this when confirming

# --- Ramp speed boost (accelerometer) tunable ---
ACCEL_SPEED_BOOST_MULTIPLIER = 1.5   # +50% speed while accel_speed_boost.py reports boosted

LOOP_HZ = 50
LOOP_SLEEP = 1.0 / LOOP_HZ


def main():
    h = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_output(h, EN_PIN, 1)   # EN HIGH -- enable the sensor array before we try reading it
    # Internal pull-up on both sensor pins -- confirmed working in
    # sensor_tests/test_line.py. WHITE -> LOW (0), BLACK -> HIGH (1).
    lgpio.gpio_claim_input(h, LEFT_PIN, lgpio.SET_PULL_UP)
    lgpio.gpio_claim_input(h, RIGHT_PIN, lgpio.SET_PULL_UP)

    motors = BLDCMotorDriver()

    cam_watcher = CameraWatcher()
    cam_watcher.start()

    tof_detector = ToFDetector()
    tof_detector.start()

    accel_boost = AccelSpeedBoost()
    accel_boost.start()

    handled_seen_at = 0.0
    turn_until = 0.0
    turn_side = None
    cooldown_until = 0.0
    spill_cooldown_until = 0.0
    obstacle_cooldown_until = 0.0

    last_error = 0.0

    print("Starting PID line follow with green marker turns, spill-zone stop, and obstacle evasion. Ctrl-C to stop.")
    try:
        while True:
            now = time.monotonic()

            # --- ToF within range: confirm with the camera before evading ---
            # (the ToF alone can't tell a ramp from an actual obstacle)
            if now >= obstacle_cooldown_until:
                distance_mm, seen_at = tof_detector.get_latest()
                fresh = distance_mm is not None and (now - seen_at) <= OBSTACLE_STALE_S
                if fresh and distance_mm <= OBSTACLE_TRIGGER_MM:
                    frame, frame_seen_at = cam_watcher.get_frame()
                    frame_fresh = frame is not None and (now - frame_seen_at) <= OBSTACLE_FRAME_STALE_S

                    if frame_fresh and is_bottle(frame):
                        run_evade_sequence(motors)
                    else:
                        print("ToF triggered but camera didn't confirm a bottle -- treating as ramp")

                    obstacle_cooldown_until = time.monotonic() + OBSTACLE_COOLDOWN_S
                    last_error = 0.0
                    continue

            # --- Possible spill/endzone tape: stop and verify before trusting it ---
            if now >= spill_cooldown_until:
                spill_seen, _ = cam_watcher.get_spill()
                if spill_seen:
                    print("possible spill detected -- stopping to verify...")
                    motors.stop()
                    time.sleep(SPILL_VERIFY_S)

                    spill_seen_confirmed, _ = cam_watcher.get_spill()
                    if spill_seen_confirmed:
                        print("endzone detected -- running evacuation zone logic")
                        motors.stop()
                        run_endzone(motors, cam_watcher, tof_detector)
                        motors.stop()
                        break   # exit + line reacquire not built yet
                    else:
                        print("false alarm -- resuming")
                        spill_cooldown_until = time.monotonic() + SPILL_FALSE_ALARM_COOLDOWN_S
                        continue

            # --- Currently mid-turn: ignore line sensors entirely ---
            if turn_until > 0.0:
                if now < turn_until:
                    if turn_side == "right":
                        motors.set_speeds(GREEN_TURN_SPEED, -GREEN_TURN_SPEED)
                    else:
                        motors.set_speeds(-GREEN_TURN_SPEED, GREEN_TURN_SPEED)
                    time.sleep(LOOP_SLEEP)
                    continue
                else:
                    turn_until = 0.0
                    turn_side = None
                    cooldown_until = now + GREEN_COOLDOWN_S
                    last_error = 0.0

            if now >= cooldown_until:
                side, seen_at = cam_watcher.get_green()
                if side is not None and seen_at != handled_seen_at:
                    if now - seen_at >= GREEN_ACT_DELAY_S:
                        handled_seen_at = seen_at
                        turn_side = side
                        turn_until = now + GREEN_TURN_DURATION_S
                        continue

            left_val = lgpio.gpio_read(h, LEFT_PIN)
            right_val = lgpio.gpio_read(h, RIGHT_PIN)

            error = float(left_val - right_val)
            correction = Kp * error + Kd * (error - last_error)
            last_error = error

            is_boosted, _accel_y = accel_boost.get_boost()
            base_speed = BASE_SPEED * ACCEL_SPEED_BOOST_MULTIPLIER if is_boosted else BASE_SPEED

            left_speed = base_speed + correction
            right_speed = base_speed - correction
            motors.set_speeds(left_speed, right_speed)

            time.sleep(LOOP_SLEEP)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        motors.stop()
        cam_watcher.stop()
        tof_detector.stop()
        accel_boost.stop()
        lgpio.gpio_write(h, EN_PIN, 0)   # EN LOW -- power the sensor array back down on exit
        lgpio.gpiochip_close(h)


if __name__ == "__main__":
    main()
