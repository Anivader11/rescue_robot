## Setup
Run once on the Pi after flashing:
```bash
bash setup_pi.sh
```

## Running
```bash
# Real hardware
python3 main.py

# Stub motors (safe test, no movement)
python3 main.py --stub

# No hardware at all (logic test)
python3 main.py --stub --no-cam
```

## Competition Day
1. Turn iPhone hotspot on first
2. Power the Pi and wait 90 seconds
3. SSH in: `ssh nigesh@rescue.local`
4. Calibrate vision on the tile: `python3 calibrate.py --camera 0`
   - Point camera at tile → press `SPACE` → press `S` to save → press `Q` to quit
5. Place robot on tile and run: `python3 main.py`

## How It Works
Three threads run concurrently:
- **Line camera thread** — captures 30 fps from CAM1, detects the black line, intersection markers, green markers and spill tape
- **Forward camera thread** — captures 15 fps from CAM0, detects obstacles and estimates distance
- **Control loop** — runs at 50 Hz, reads vision data, runs PID controller, sends speed commands to all four motors over I2C

## Tests
```bash
python3 tests/test_line_detector.py
```
Expected: 15/15 passed

## Pushing Code to Pi
From your laptop in the rescue_robot folder:
```bash
scp -r rescue_robot nigesh@rescue.local:~/
```

## Motor I2C Addresses
| Motor | Address |
|-------|---------|
| Front left | 0x1A |
| Front right | 0x19 |
| Back left | 0x1B |
| Back right | 0x1C |

## Tuning
Key values to tune on the floor before competition in `state_machine.py`:
- `BASE_SPEED` — cruising speed
- `MARKER_TURN_DURATION_S` — time to turn at intersection marker
- `OBSTACLE_ARC_DURATION_S` — time to arc around obstacle

And in `motors.py`:
- `LEFT_INVERTED` / `RIGHT_INVERTED` — flip if wheels spin backwards
- `BASE_SPEED_UNITS` — motor speed in BLDC units
