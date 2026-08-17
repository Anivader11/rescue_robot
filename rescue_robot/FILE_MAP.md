# File Map — Rescue Robot Project

Where every file goes. Keep this in the project root as a quick reference
when copying files out of chat downloads.

```
rescue_robot/
│
├── main.py                    ← entry point, run this to start the robot
├── calibrate.py                ← on-site vision calibration tool
├── setup_pi.sh                  ← run once on a fresh Pi to install everything
├── FILE_MAP.md         ← this file
|-- Identify_motors.py         
│
├── vision/
│   ├── __init__.py               ← empty file, makes this a package
│   ├── line_detector.py          ← LineDetector class, line/marker/tape detection
│   └── vision_thread.py          ← camera threads (line cam + forward cam)
│
├── control/
│   ├── __init__.py               ← empty file, makes this a package
│   ├── shared_vision.py          ← thread-safe bridge between vision and control
│   ├── motors.py                 ← PID controller + BLDC motor driver (I2C)
│   └── state_machine.py          ← the main control loop / robot states
│
└── tests/
    ├── __init__.py               ← empty file, makes this a package
    └── test_line_detector.py     ← unit tests, run with:
                                       python3 tests/test_line_detector.py
```

## Quick copy reference

| File downloaded from chat | Goes in |
|---|---|
| `main.py` | `rescue_robot/` |
| `calibrate.py` | `rescue_robot/` |
| `setup_pi.sh` | `rescue_robot/` |
| `line_detector.py` | `rescue_robot/vision/` |
| `vision_thread.py` | `rescue_robot/vision/` |
| `shared_vision.py` | `rescue_robot/control/` |
| `motors.py` | `rescue_robot/control/` |
| `state_machine.py` | `rescue_robot/control/` |
| `test_line_detector.py` | `rescue_robot/tests/` |

## Creating the three empty `__init__.py` files

These don't get downloaded from chat — they're always empty, just create them
directly wherever you're working:

**On the Pi (SSH):**
```bash
touch ~/rescue_robot/vision/__init__.py
touch ~/rescue_robot/control/__init__.py
touch ~/rescue_robot/tests/__init__.py
```

**On your laptop (VS Code):** right-click each folder → New File →
name it `__init__.py` → leave empty → save.

## Pushing updated files to the Pi

From your laptop, inside the `rescue_robot` folder:
```bash
scp -r vision control tests main.py calibrate.py nigesh@rescue.local:~/rescue_robot/
```

## Verifying the structure is correct

Run this on either machine — should list exactly 9 `.py` files:
```bash
find rescue_robot -name "*.py" | sort
```
