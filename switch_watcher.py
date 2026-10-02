#!/usr/bin/env python3
"""
switch_watcher.py
=================
Push button toggles (press = start, press again = stop) line_follow/pid_line_follow.py.
Run at boot by rescue-robot-switch.service.

Wiring (BCM):  GPIO15 (pin 10) -- push button (2 terminals) -- GPIO14 (pin 8)
    GPIO15 is driven HIGH as the button's "power" side.
    GPIO14 is an input with internal pull-down:
        released -> LOW,  pressed -> HIGH
    (GPIO14/15 are the UART pins -- serial console must be OFF, see README.)

Behaviour (momentary push button, toggles):
  * Press once  -> starts the run script.
  * Press again -> stops it: SIGINT (runs pid_line_follow.py's finally:
    block), SIGKILL after STOP_GRACE_S.
  * Only the PRESS counts (not the release), debounced.
  * After every run ends (button, crash, endzone finished) the watcher
    sends its own motors.stop() as a safety net.
  * If the run ends by itself, the next press starts a fresh run.
  * Each run's output is saved to logs/run_YYYYmmdd_HHMMSS.log.

Test by hand (stop the service first so they don't fight over the pins):
    sudo systemctl stop rescue-robot-switch.service
    python3 switch_watcher.py                    # default script
    python3 switch_watcher.py line_follow/pid_line_follow_only.py
"""

import os
import signal
import subprocess
import sys
import time
from datetime import datetime

import lgpio

# --------------------------------------------------------------------------
# Config
# --------------------------------------------------------------------------
SWITCH_PIN = 14       # input  (pin 8),  pull-down
DRIVE_PIN = 15        # output (pin 10), held HIGH
GPIO_CHIP = 0            # same chip pid_line_follow.py opens
DEBOUNCE_S = 0.08        # switch must read the same this long to count
POLL_S = 0.02
STOP_GRACE_S = 4.0

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SCRIPT = os.path.join("line_follow", "pid_line_follow_no_endzone.py")
LOG_DIR = os.path.join(PROJECT_DIR, "logs")


def log(msg):
    print(f"{datetime.now():%H:%M:%S}  [switch] {msg}", flush=True)


# --------------------------------------------------------------------------
# Motor safety net
# --------------------------------------------------------------------------
def safety_stop_motors():
    """Open the BLDC driver and send stop, in case the run died hard."""
    try:
        sys.path.insert(0, PROJECT_DIR)
        from control.motors import BLDCMotorDriver
        BLDCMotorDriver().stop()
        log("safety stop sent to motors")
    except Exception as e:  # never let this kill the watcher
        log(f"safety stop failed (ok if motors already stopped): {e}")


# --------------------------------------------------------------------------
# Run process
# --------------------------------------------------------------------------
class Run:
    def __init__(self, script):
        self.script = os.path.join(PROJECT_DIR, script)
        self.proc = None
        self.logf = None

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self):
        if not os.path.isfile(self.script):
            log(f"ERROR: can't find {self.script}")
            return
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, f"run_{datetime.now():%Y%m%d_%H%M%S}.log")
        self.logf = open(path, "w", buffering=1)
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        log(f"ON  -> starting {os.path.relpath(self.script, PROJECT_DIR)}  (log: {path})")
        # Run from the script's own folder, exactly like typing
        # `python3 pid_line_follow.py` inside line_follow/.
        self.proc = subprocess.Popen(
            [sys.executable, "-u", self.script],
            cwd=os.path.dirname(self.script),
            stdout=self.logf, stderr=subprocess.STDOUT,
            env=env, start_new_session=True,
        )

    def stop(self):
        if not self.alive():
            return
        log("OFF -> stopping run (SIGINT)")
        try:
            pgid = os.getpgid(self.proc.pid)
            os.killpg(pgid, signal.SIGINT)
            try:
                self.proc.wait(timeout=STOP_GRACE_S)
            except subprocess.TimeoutExpired:
                log(f"no exit after {STOP_GRACE_S}s -> SIGKILL")
                os.killpg(pgid, signal.SIGKILL)
                self.proc.wait(timeout=3)
        except ProcessLookupError:
            pass
        self.finish()

    def finish(self):
        code = self.proc.returncode if self.proc else None
        log(f"run ended (exit code {code})")
        self.proc = None
        if self.logf:
            self.logf.close()
            self.logf = None
        safety_stop_motors()


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def main():
    script = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SCRIPT
    h = lgpio.gpiochip_open(GPIO_CHIP)
    lgpio.gpio_claim_output(h, DRIVE_PIN, 1)                  # button source HIGH
    lgpio.gpio_claim_input(h, SWITCH_PIN, lgpio.SET_PULL_DOWN)
    time.sleep(0.05)

    def pressed():
        return lgpio.gpio_read(h, SWITCH_PIN) == 1   # HIGH = button held

    run = Run(script)
    stable = pressed()
    raw_prev, raw_since = stable, time.monotonic()

    log(f"watching button GPIO{DRIVE_PIN}->GPIO{SWITCH_PIN}; script = {script}")
    log("press = start run, press again = stop run")
    if stable:
        log("button held at boot -- release it first (ignored)")

    stopping = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stopping.update(flag=True))

    try:
        while not stopping["flag"]:
            raw = pressed()
            now = time.monotonic()
            press_edge = False
            if raw != raw_prev:
                raw_prev, raw_since = raw, now
            elif raw != stable and now - raw_since >= DEBOUNCE_S:
                stable = raw
                press_edge = stable          # only act on press, not release

            # Run ended by itself (finished, crashed, import error...)
            if run.proc is not None and not run.alive():
                run.finish()
                log("press the button for a new run")

            if press_edge:
                if run.alive():
                    log("button -> stop")
                    run.stop()
                else:
                    log("button -> start")
                    run.start()

            time.sleep(POLL_S)
    except KeyboardInterrupt:
        pass
    finally:
        run.stop()
        lgpio.gpiochip_close(h)
        log("watcher stopped")


if __name__ == "__main__":
    main()
