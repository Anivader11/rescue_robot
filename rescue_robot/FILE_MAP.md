# File Map — Rescue Robot Project

Current layout after pivoting from camera-based line following to digital IR
line sensors. Kept in the project root as a quick reference.

```
rescue_robot/
│
├── control/
│   └── motors.py                  ← BLDCMotorDriver, unchanged, still used
│
├── line_follow/                   ← ACTIVE line-following system
│   ├── pid_line_follow.py         ← entry point, run this to start the robot
│   └── green_detector.py          ← background camera thread, green marker side detection
│
├── sensor_tests/                  ← GPIO/sensor diagnostic scripts (reference only)
│   ├── test_28034_line_sensor.py
│   ├── test_28034_line_sensor_rawlgpio.py
│   └── test_minimal.py            ← simplest confirmed-working sensor read
│
├── switch_watcher.py              ← boot switch watcher, launches line_follow/pid_line_follow.py
├── rescue-robot-switch.service    ← systemd unit for switch_watcher.py
├── requirements.txt
├── setup_pi.sh
├── README.md
├── FILE_MAP.md                    ← this file
│
└── old_camera_version/            ← ARCHIVE, not run anymore
    ├── main.py                    ← old entry point
    ├── vision/                    ← old camera line/marker/obstacle detection
    ├── tests/
    ├── control/
    │   ├── state_machine.py       ← old robot state machine (LINE_FOLLOWING/LOST/etc)
    │   └── shared_vision.py
    ├── calib.py
    ├── calibrate.py
    ├── calibrate_obstacle.py
    ├── calibrate_once.py
    ├── calibration_offsets.json
    ├── example.py
    ├── test_drive_calibration.py
    ├── test_forward.py
    ├── test_forwardreal.py
    ├── test_forwardv2.py
    ├── test_individual.py
    ├── test_individual_livecal.py
    ├── test_turn.py
    ├── push_test_forwardv2.sh
    ├── Rescue_Line_Robot_Code_Documentation.docx
    ├── __pycache__/
    └── _to_delete/
```

## Running the robot

```bash
python3 line_follow/pid_line_follow.py
```

or just flip the physical switch (systemd runs `switch_watcher.py`, which
launches the above automatically).

## Pushing updated files to the Pi

From your laptop, inside the `rescue_robot` folder (Windows example):
```
scp "path\to\rescue_robot\line_follow\pid_line_follow.py" nigesh@rescue.local:~/rescue_robot/line_follow/pid_line_follow.py
scp "path\to\rescue_robot\line_follow\green_detector.py" nigesh@rescue.local:~/rescue_robot/line_follow/green_detector.py
```

Make sure the destination folder exists first:
```
ssh nigesh@rescue.local "mkdir -p ~/rescue_robot/line_follow ~/rescue_robot/sensor_tests"
```

## Verifying the structure is correct

```bash
find rescue_robot -name "*.py" -not -path "*/old_camera_version/*" | sort
```
Should list exactly: `control/motors.py`, `line_follow/pid_line_follow.py`,
`line_follow/green_detector.py`, `sensor_tests/test_28034_line_sensor.py`,
`sensor_tests/test_28034_line_sensor_rawlgpio.py`,
`sensor_tests/test_minimal.py`, `switch_watcher.py`.
