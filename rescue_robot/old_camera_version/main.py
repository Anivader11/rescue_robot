"""
main.py
=======
Entry point for the Rescue Line robot.

Hardware:
    - Raspberry Pi 5
    - 1x Pi Camera Module V2 (CSI, CAM0) — the forward/obstacle camera
      broke, so the surviving camera has been moved into the line camera's
      port and is used for line following only. Obstacle detection is
      disabled (see vision/vision_thread.py, FWD_CAM = None) until a
      replacement camera is fitted.
    - 2x RoboMaster M2006 P36 via Powerful BLDC Driver boards (I2C) —
      front left + front right only. Two wheel drive: the rear right
      motor failed and rear left is now unused too (see control/motors.py).
    - I2C1 on Pi GPIO pins 3 (SDA) and 5 (SCL) at 1MHz

Run modes:
    python3 main.py              # real hardware
    python3 main.py --stub       # stub motors, real cameras (safe test)
    python3 main.py --stub --no-cam  # no hardware at all (pure logic test)

Ctrl-C stops everything cleanly.

Before first run:
    1. Add to /boot/firmware/config.txt:
           dtoverlay=i2c1,pins_2_3,baudrate=1000000
    2. Install motor library:
           pip install git+https://github.com/Aw3someAndrew/SteelBar_CircuitPython_powerful_bldc_driver.git
    3. Install camera library:
           sudo apt install python3-picamera2 -y
    4. Set I2C addresses in control/motors.py to match your boards
    5. Run calibrate.py to tune vision thresholds for competition venue
    6. Run calibrate_once.py to (re)generate calibration_offsets.json —
       motors.py ONLY reads this file, it never calibrates live. Rerun
       calibrate_once.py after any power loss to the driver boards, or
       after swapping/rewiring a motor.
"""

import argparse
import logging
import os
import sys
import time

# calibration.json lives next to this script, NOT in whatever CWD the process
# was launched from (systemd / SSH from another dir would silently miss it).
CALIBRATION_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "calibration.json")

# When started with no interactive terminal (e.g. by switch_watcher.py at
# boot), we can't block on input() -- it would hang forever (or raise
# EOFError) with stdin redirected from /dev/null under systemd. Give this
# long a window to place the robot on the tile instead, then continue.
NO_TTY_CALIBRATION_DELAY_S = 5

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(description="Rescue Line robot")
    parser.add_argument("--stub",    action="store_true",
                        help="Use stub motors instead of real BLDC drivers")
    parser.add_argument("--no-cam", action="store_true",
                        help="Skip cameras entirely (stub vision)")
    parser.add_argument("--debug",  action="store_true",
                        help="Enable verbose vision debug output")
    args = parser.parse_args()

    from control.shared_vision import SharedVision
    from vision.line_detector import load_calibration

    # Apply any on-site calibration saved by calibrate.py, before any
    # LineDetector is constructed. Without this, pressing "s" in calibrate.py
    # tunes nothing that the actual robot run ever sees.
    load_calibration(CALIBRATION_PATH)
    from control.state_machine  import StateMachine

    shared = SharedVision()

    # --- Motor driver ---
    if args.stub:
        from control.motors import StubMotorDriver
        motors = StubMotorDriver(verbose=args.debug)
        log.info("Using StubMotorDriver")
    else:
        try:
            from control.motors import BLDCMotorDriver
            motors = BLDCMotorDriver()
            log.info("BLDCMotorDriver initialised")
        except Exception as e:
            log.error("BLDC init failed: %s", e)
            log.error("If this is a missing/incomplete calibration_offsets.json, "
                       "run calibrate_once.py first. Run with --stub to test "
                       "without motors.")
            sys.exit(1)

    # --- Vision threads ---
    vision = None
    if not args.no_cam:
        from vision.vision_thread import VisionThread
        vision = VisionThread(shared, debug=args.debug)

        # Calibration window — robot should be placed on tile before calibrating.
        # This ensures the first calibration frame is a real tile, not a hand or desk.
        print("\n" + "="*50)
        if sys.stdin.isatty():
            print("Place robot on the WHITE TILE, then press ENTER to calibrate cameras.")
            print("="*50)
            input()
        else:
            # No interactive terminal -- e.g. started by switch_watcher.py at
            # boot. Assume the Robot Handler already placed the robot on the
            # tile per competition setup, give a short window to finish that,
            # then continue rather than blocking forever on input().
            print(f"No terminal attached — place robot on the WHITE TILE now, "
                  f"calibrating cameras in {NO_TTY_CALIBRATION_DELAY_S}s...")
            print("="*50)
            log.warning("No interactive terminal — skipped ENTER prompt, "
                        "waited %ds instead", NO_TTY_CALIBRATION_DELAY_S)
            time.sleep(NO_TTY_CALIBRATION_DELAY_S)

        vision.start()
        log.info("Waiting 1s for cameras to settle...")
        time.sleep(1.0)
    else:
        log.info("--no-cam: running without cameras")

    # --- State machine ---
    sm = StateMachine(shared, motors)

    # --- Run ---
    log.info("Starting control loop — Ctrl-C to stop")
    try:
        sm.run()
    finally:
        if vision:
            vision.stop()
        motors.stop()
        if hasattr(motors, "cleanup"):
            motors.cleanup()
        log.info("Shutdown complete")


if __name__ == "__main__":
    main()
