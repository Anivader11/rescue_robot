"""
shared_vision.py
================
Thread-safe container that vision threads write to and the control loop reads from.
"""
import threading
from dataclasses import dataclass, field
from unittest import result
from vision.line_detector import LineState

@dataclass
class SharedVision:
    line_state:           LineState = LineState.LOST
    line_error:           float     = 0.0
    line_confidence:      float     = 0.0
    line_width_px:        float     = 0.0
    intersection:         bool      = False
    green_marker:         bool      = False
    spill_tape:           bool      = False
    last_valid_error:     float     = 0.0
    obstacle_detected:    bool      = False
    obstacle_distance_cm: float     = 999.0
    green_marker:     bool  = False
    green_marker_x:   float = 0.0    # normalised -1..1, negative=left, positive=right
    lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def update_line(self, result) -> None:
        with self.lock:
            self.line_state       = result.state
            self.line_error       = result.error
            self.line_confidence  = result.confidence
            self.line_width_px    = result.line_width_px
            self.intersection     = result.intersection
            self.green_marker     = result.green_marker
            self.spill_tape       = result.spill_tape
            self.last_valid_error = result.last_error
            self.green_marker   = result.green_marker
            self.green_marker_x = result.green_marker_x

    def update_forward(self, obstacle: bool, distance_cm: float) -> None:
        with self.lock:
            self.obstacle_detected    = obstacle
            self.obstacle_distance_cm = distance_cm

    def snapshot(self) -> "SharedVision":
        with self.lock:
            return SharedVision(
                line_state           = self.line_state,
                line_error           = self.line_error,
                line_confidence      = self.line_confidence,
                line_width_px        = self.line_width_px,
                intersection         = self.intersection,
                green_marker         = self.green_marker,
                spill_tape           = self.spill_tape,
                last_valid_error     = self.last_valid_error,
                obstacle_detected    = self.obstacle_detected,
                obstacle_distance_cm = self.obstacle_distance_cm,
                green_marker   = self.green_marker,
                green_marker_x = self.green_marker_x,
            )
