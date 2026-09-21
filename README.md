# Rescue Line Robot

RoboCup Junior Rescue Line robot running on a Raspberry Pi 5. Two-wheel
differential drive on RoboMaster M2006 P36 brushless motors, with digital
IR line sensors for line following and a Pi Camera Module doing the rest
of the perception (green direction markers, silver spill/endzone tape,
obstacle confirmation).

The robot used to be camera-only for line following (see
[`old_camera_version/`](old_camera_version/)); it's since moved to
dedicated IR line sensors, with the camera now doing everything that
isn't line position: markers, spill tape, and obstacle confirmation.

## Hardware

- **Compute:** Raspberry Pi 5, SSH reachable at `nigesh@rescue.local`
- **Drive:** 2x RoboMaster M2006 P36 brushless motors via Powerful BLDC
  Driver boards over I2C (front-left `0x20`, front-right `0x19`) — rear
  motors are disconnected, robot runs 2-wheel drive
- **Line sensors:** 2x digital IR line sensors (Parallax 28034-style),
  GPIO24 (left) / GPIO23 (right), spaced so that when centered on the
  line **both** read white — the line runs between them, not under
  either one
- **Camera:** Raspberry Pi Camera Module (via `picamera2`) — green
  marker detection, silver spill-zone detection, obstacle confirmation
- **ToF sensor:** SteelBar Time-of-Flight distance sensor, I2C address
  `0x51` (confirmed via `i2cdetect -y 1` — the vendor example code says
  `0x50`, that's wrong for this board)
- **IMU:** Adafruit BNO085 9-DOF orientation sensor, I2C address `0x4A`
- All I2C devices share bus 1 (GPIO2/SDA, GPIO3/SCL)

## Repo layout

```
rescue_robot/
├── line_follow/              -- active robot code (see below)
├── control/
│   └── motors.py             -- BLDC motor driver + PID controller class
├── sensor_tests/              -- standalone IR line sensor test scripts
├── old_camera_version/        -- archived camera-only line-following system
├── switch_watcher.py          -- GPIO21 physical switch: turns main.py on/off
├── rescue-robot-switch.service -- systemd unit for switch_watcher.py
├── setup_pi.sh                -- Pi provisioning script
├── requirements.txt
└── FILE_MAP.md                -- more detailed file-by-file breakdown
```

## `line_follow/` — active system

**Entry point:** `pid_line_follow.py`. Run with:
```
python3 line_follow/pid_line_follow.py
```

### Core line following

Two digital IR sensors, read once per loop tick (`LEFT_PIN=24`,
`RIGHT_PIN=23`). `error = left_val - right_val` (-1, 0, or +1 — sensors
are binary, not analog). A PD controller (`Kp=20`, `Kd=10`, no I term —
not useful with a 3-valued error signal) turns that into a differential
steering correction applied to both wheels around `BASE_SPEED`. Both
`(0,0)` (centered — line between the sensors) and `(1,1)` (on an
intersection) correctly resolve to error `0` and drive straight; there's
no separate "line lost" failsafe.

### Camera subsystem — `camera_watcher.py`

Only one physical camera exists on the robot, so a single shared
`Picamera2` thread runs every camera-based check on the same captured
frame, instead of each detector opening its own camera (which would
conflict):

- **Green marker detection** — HSV threshold + largest-contour check,
  reports which side (`left`/`right`) a green marker was seen on. Resets
  to `None` when nothing's detected that frame (doesn't get stuck on the
  last reading).
- **Spill/endzone tape detection** — `spill_classifier.py`, a trained
  Random Forest classifier (95.6% test accuracy) running on features
  extracted from a fixed center box each frame. See "Spill-zone ML
  pipeline" below for how it was built. Classical CV approaches
  (brightness threshold, saturation threshold, local std-dev, top-hat
  filter) were all tried first and failed — the tape and tile are close
  to indistinguishable pixel-by-pixel; only a trained classifier over
  whole-region features worked reliably.
- **`get_frame()`** — exposes the latest raw frame so other code
  (`bottle_detector.py`) can run its own one-off check on demand without
  opening a second camera.

`green_detector.py` and `spill_detector.py` are kept as standalone
single-purpose versions of the same logic (useful for isolated testing)
but aren't imported by `pid_line_follow.py` anymore — `camera_watcher.py`
replaced both.

### Obstacle handling — ToF + camera confirmation

`tof_sensor.py` is a Python port of the SteelBar ToF sensor's I2C
protocol (register `0x10`, 5-byte read: sequence number + signed
little-endian distance in mm). `tof_detector.py` polls it in a background
thread.

The ToF alone can't tell a ramp from an actual obstacle (both can read
within 50mm), so when it triggers, `bottle_detector.py` checks the
latest camera frame before committing to anything: since the obstacle
(a 1.25L clear water bottle) has no reliable color to threshold on, it
looks for shape instead — Canny edges + a Hough line transform, checking
whether the scene is dominated by near-vertical lines (a bottle's two
side edges + its cylindrical highlight) rather than the diagonal/
horizontal edges a ramp's sloped face would produce. Only if the camera
confirms it does `obstacle_evade.py` run its fixed timed evasion
maneuver (back → right → forward → left → straight → left → right, then
resume line following); otherwise it's treated as a ramp and normal line
following just continues.

`bottle_detector.py`'s shape heuristic is untested on real hardware —
tune `MIN_VERTICAL_LINES` / `VERTICAL_LINE_RATIO_THRESHOLD` against the
actual bottle and ramp once you can test on the Pi (standalone test mode:
`python3 line_follow/bottle_detector.py`, using a laptop webcam).

### Ramp speed boost — IMU

`imu_sensor.py` wraps the Adafruit BNO085 (`adafruit-circuitpython-bno08x`)
and reports raw accelerometer/gyroscope/magnetometer values. A background
thread, `accel_speed_boost.py`, watches raw Y acceleration; level ground
reads around `Y=-9.8` (gravity), and once `Y` drops below `-15.0`
(nosing upward onto a ramp), `pid_line_follow.py` multiplies `BASE_SPEED`
by 1.5 (+50%) until `Y` comes back above the threshold.

### Spill-zone ML pipeline (how `spill_model_weights.py` was built)

1. `collect_spill_samples.py` (laptop webcam) / a Pi-side collector —
   capture labelled samples (silver tape vs. tile, including tricky
   negatives like shadows/the black line) into `spill_samples.csv`.
   Features per sample: mean R/G/B, mean H/S/V, brightness, std_dev,
   max, min, range — all measured over a fixed 150x150px center box.
2. `train_spill_classifier.py` (laptop, needs `scikit-learn`) — trains
   both `LogisticRegression` and `RandomForestClassifier`, picks whichever
   scores higher on a held-out split, and exports the winner as plain
   Python data (`spill_model_weights.py`) with **no sklearn dependency**.
3. `spill_classifier.py` — pure-numpy inference on the Pi, no sklearn
   needed at runtime. Supports both model types the trainer might export.

To retrain: collect more samples into `spill_samples.csv`, rerun
`train_spill_classifier.py`, copy the regenerated `spill_model_weights.py`
onto the Pi.

### Test/calibration utilities

- `test_forward.py` — drives both wheels forward briefly, for verifying
  `control/motors.py`'s `MOTOR_CONFIG` calibration values after a power
  cycle (the BLDC driver boards' `elec_angle_offset`/`sincos_centre`
  reset when they lose power — re-run a calibration tool and paste fresh
  values into `MOTOR_CONFIG` when that happens)
- `test_green_webcam.py`, `test_spill_webcam.py`,
  `test_spill_classifier_webcam.py` — standalone webcam test harnesses
  for the camera detectors, without needing the full robot loop running
- `calibrate_spill_webcam.py` — measures brightness/color/std-dev
  differences between the silver tape and white tile, used while
  designing the spill detection approach

## `control/motors.py`

`BLDCMotorDriver` — real I2C driver for the two front M2006 motors via
the `steelbar_powerful_bldc_driver` library. Sign convention: right side
positive, left side negative = forward. `MOTOR_CONFIG` holds per-motor
I2C address + calibration values that must be refreshed after any power
cycle of the driver boards.

Also includes `PIDController` (a more general PID used elsewhere) and
`StubMotorDriver` (prints commands instead of driving hardware, for
testing without the robot).

## `sensor_tests/`

Standalone scripts for bringing up and verifying the IR line sensors in
isolation, before they're wired into the main loop.

## `old_camera_version/`

Archived: the original all-camera line-following system (HSV-threshold
line detection instead of IR sensors, PID over camera-estimated line
position). Kept for reference; not run anymore.

## `switch_watcher.py` + `rescue-robot-switch.service`

Watches a physical GPIO21 switch: ON starts `line_follow/pid_line_follow.py`
as a subprocess, OFF stops it. Installed as a systemd service so it
starts on boot:
```
sudo cp rescue-robot-switch.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rescue-robot-switch.service
```

## Setup

```
bash setup_pi.sh                 # Pi provisioning
pip install -r requirements.txt
pip install adafruit-circuitpython-bno08x adafruit-blinka   # for imu_sensor.py
pip install git+https://github.com/Aw3someAndrew/SteelBar_CircuitPython_powerful_bldc_driver.git
```

See `FILE_MAP.md` for a more granular file-by-file reference.
