#!/usr/bin/env python3
"""
switch_watcher.py
==================
Watches a physical ON/OFF toggle switch on a GPIO pin and starts/stops
main.py (the real line-following run) accordingly. Meant to run as a
systemd service (see rescue-robot-switch.service) so it's armed the
moment the Pi finishes booting -- no SSH or keyboard needed. Flip the
switch after the Pi's power switch, and it starts the robot.

Wiring (BCM numbering):
    3.3V (physical pin 1 or 17) -- switch -- GPIO21 (physical pin 40)

    No physical pull resistor needed -- GPIO21 is configured with the
    Pi's internal pull-down below, so it reads LOW when the switch is
    open and HIGH when closed.

    Switch OPEN  (off) -> GPIO21 LOW  -> main.py stopped
    Switch CLOSED (on) -> GPIO21 HIGH -> main.py started

    If your switch is wired differently (e.g. to GND instead of 3.3V),
    change pull_up=False to pull_up=True below and the logic still works
    (gpiozero's Button.is_pressed follows whichever wiring you configure).

Behaviour:
    - Switch flips ON  -> launches `python3 main.py` (real hardware, not
      --stub) in its own process group.
    - Switch flips OFF -> sends SIGINT to that process group, so
      main.py's normal Ctrl-C shutdown path runs (state machine's
      finally: block stops the motors cleanly), escalating to SIGKILL if
      it doesn't exit within STOP_GRACE_S.
    - If main.py exits on its own (course finished, crashed, hardware
      init failed) while the switch is still ON, it is NOT auto-restarted
      -- flip the switch off then on again to start a fresh run. This
      avoids a crash-restart loop if something is actually broken.

Run manually for testing (Ctrl-C to stop the watcher itself):
    python3 switch_watcher.py
"""

import logging
import os
import signal
import subprocess
import sys
import time

from gpiozero import Button

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

SWITCH_PIN   = 21     # BCM numbering -- change if wired to a different pin
DEBOUNCE_S   = 0.05
POLL_S       = 0.2
STOP_GRACE_S = 5.0    # time to let main.py shut down cleanly before SIGKILL

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
MAIN_PY     = os.path.join(PROJECT_DIR, "line_follow", "pid_line_follow.py")   # updated: pivoted off main.py


class RobotProcess:
    """Starts/stops main.py, and tracks whether it exited on its own so
    it isn't auto-restarted mid-ON without an OFF/ON cycle first."""

    def __init__(self):
        self._proc  = None
        self._armed = True

    def _alive(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def tick(self, switch_on: bool) -> None:
        if self._proc is not None and not self._alive():
            log.info("[switch] main.py exited on its own (code %s) -- "
                      "flip the switch off then on again to restart",
                      self._proc.returncode)
            self._proc = None
            self._armed = False   # require an OFF edge before restarting

        if not switch_on:
            self._armed = True
            if self._alive():
                self._stop()
        elif self._armed and not self._alive():
            self._start()

    def _start(self) -> None:
        log.info("[switch] ON -> starting main.py")
        self._proc = subprocess.Popen(
            [sys.executable, MAIN_PY],
            cwd=PROJECT_DIR,
            start_new_session=True,   # own process group -- lets us
                                       # signal main.py without also
                                       # killing this watcher
        )

    def _stop(self) -> None:
        log.info("[switch] OFF -> stopping main.py")
        try:
            pgid = os.getpgid(self._proc.pid)
            os.killpg(pgid, signal.SIGINT)   # triggers main.py's normal
                                              # KeyboardInterrupt shutdown,
                                              # which stops the motors
            deadline = time.monotonic() + STOP_GRACE_S
            while self._alive() and time.monotonic() < deadline:
                time.sleep(0.1)
            if self._alive():
                log.warning("[switch] main.py did not exit within %.1fs -- "
                            "sending SIGKILL", STOP_GRACE_S)
                os.killpg(pgid, signal.SIGKILL)
                self._proc.wait(timeout=5)
        except ProcessLookupError:
            pass   # already exited between the check and the signal
        finally:
            self._proc = None

    def shutdown(self) -> None:
        if self._alive():
            self._stop()


def main():
    switch = Button(SWITCH_PIN, pull_up=False, bounce_time=DEBOUNCE_S)
    robot  = RobotProcess()

    log.info("[switch] Watching GPIO%d -- ON starts main.py, OFF stops it",
              SWITCH_PIN)

    try:
        while True:
            robot.tick(switch.is_pressed)
            time.sleep(POLL_S)
    except KeyboardInterrupt:
        pass
    finally:
        robot.shutdown()
        log.info("[switch] Watcher stopped")


if __name__ == "__main__":
    main()
