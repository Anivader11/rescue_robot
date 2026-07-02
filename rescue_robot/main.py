"""
main.py
=======
Entry point for the Rescue Line robot.

Hardware:
    - Raspberry Pi 5
    - 2x Pi Camera Module V2 (CSI — CAM0=line, CAM1=forward)
    - 4x RoboMaster M2006 P36 via Powerful BLDC Driver boards (I2C)
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
"""

import argparse
import logging
import sys
import time

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
            log.error("Run with --stub to test without motors")
            sys.exit(1)

    # --- Vision threads ---
    vision = None
    if not args.no_cam:
        from vision.vision_thread import VisionThread
        vision = VisionThread(shared, debug=args.debug)

        # Calibration window — robot should be placed on tile before pressing Enter
        # This ensures the first calibration frame is a real tile, not a hand or desk
        print("\n" + "="*50)
        print("Place robot on the WHITE TILE, then press ENTER to calibrate cameras.")
        print("="*50)
        input()

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
