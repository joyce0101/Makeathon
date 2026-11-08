"""Dual digital IR sensing for the Pico; no motor commands are issued.

Wiring: left OUT -> GP14, right OUT -> GP15, VCC -> 3V3, common GND.
Left/right are defined from the robot's perspective while facing forward.
For the proposed mounting, both probes sit inside the black tape when centered.
BOTH_BLACK does not prove a junction, and NO_LINE does not prove a finish.

Use ir_calibration.py to measure raw levels; run this file for filtered states.
The competition entry point applies config.py sampling settings without changing
this sensor implementation. Calibrate on blue/wood/red/green/grey surfaces at
the final mounting height; both probes remain on tape when the robot is centered.

Integration: call init_ir() once, then update_ir() frequently. When it returns
True, get_ir_snapshot() returns a new sample. A state_changed flag describes
that update only; repeatedly reading a snapshot is not a new event.
"""

from machine import Pin
from time import sleep_ms, ticks_diff, ticks_ms


LEFT_GPIO = 14
RIGHT_GPIO = 15
LEFT_BLACK_LEVEL = 1  # Measured: black tape = 1, background = 0.
RIGHT_BLACK_LEVEL = 1  # Measured: black tape = 1, background = 0.
SAMPLE_INTERVAL_MS = 5
CONFIRM_SAMPLES = 3
LINE_MISSING_MS = 100
PRINT_INTERVAL_MS = 100

_pins = None
_levels = (None, None)
_snapshot = {}
_candidate = None
_candidate_count = 0
_state_since = None


def read_raw_ir():
    """Return (left, right) digital inputs without assuming black polarity."""
    global _pins
    if _pins is None:
        _pins = (Pin(LEFT_GPIO, Pin.IN), Pin(RIGHT_GPIO, Pin.IN))
    return (_pins[0].value(), _pins[1].value())


def init_ir(left_black_level=None, right_black_level=None):
    """Set calibrated black levels (0 or 1) and reset filtering before use."""
    global _pins, _levels, _snapshot, _candidate, _candidate_count, _state_since
    left = LEFT_BLACK_LEVEL if left_black_level is None else left_black_level
    right = RIGHT_BLACK_LEVEL if right_black_level is None else right_black_level
    _pins = (Pin(LEFT_GPIO, Pin.IN), Pin(RIGHT_GPIO, Pin.IN))
    _levels = (left, right)
    _candidate = None
    _candidate_count = 0
    _state_since = None
    _snapshot = {
        "timestamp_ms": None,
        "raw": None,
        "black": None,
        "state": "UNSTABLE",
        "state_duration_ms": 0,
        "last_seen_side": None,
        "last_side_ms": None,
        "line_missing": False,
        "state_changed": False,
    }


def update_ir(now_ms=None):
    """Take at most one due sample without sleeping; return whether sampled.

    now_ms, if supplied, must use the ticks_ms clock. The caller must keep
    calling this function: it cannot sample while other code blocks.
    Durations are measured at the latest sample, using wrap-safe ticks_diff.
    """
    global _candidate, _candidate_count, _state_since
    now = ticks_ms() if now_ms is None else now_ms
    _snapshot["state_changed"] = False
    last = _snapshot["timestamp_ms"]
    if last is not None and ticks_diff(now, last) < SAMPLE_INTERVAL_MS:
        return False

    raw = read_raw_ir()
    black = (raw[0] == _levels[0], raw[1] == _levels[1])
    if black == (True, False):
        _snapshot["last_seen_side"] = "LEFT"
        _snapshot["last_side_ms"] = now
    elif black == (False, True):
        _snapshot["last_seen_side"] = "RIGHT"
        _snapshot["last_side_ms"] = now

    if black == _candidate:
        _candidate_count = min(_candidate_count + 1, CONFIRM_SAMPLES)
    else:
        _candidate = black
        _candidate_count = 1

    if _candidate_count >= CONFIRM_SAMPLES:
        if black == (True, True):
            state = "BOTH_BLACK"
        elif black == (True, False):
            state = "LINE_LEFT"
        elif black == (False, True):
            state = "LINE_RIGHT"
        else:
            state = "NO_LINE"

        if state != _snapshot["state"]:
            _snapshot["state"] = state
            _snapshot["state_changed"] = True
            _state_since = now

    duration = 0 if _state_since is None else ticks_diff(now, _state_since)
    _snapshot["timestamp_ms"] = now
    _snapshot["raw"] = raw
    _snapshot["black"] = black
    _snapshot["state_duration_ms"] = duration
    # Count from the confirmed NO_LINE transition; clear on confirmed recovery.
    _snapshot["line_missing"] = (
        _snapshot["state"] == "NO_LINE" and duration >= LINE_MISSING_MS
    )
    return True


def get_ir_snapshot():
    """Return a copy of the latest sample; this does not read GPIO again."""
    return _snapshot.copy()


def main():
    """Print filtered readings; use ir_calibration.py for raw calibration."""
    init_ir()
    print("Filtered IR test. Ctrl+C to stop.")
    last_print = None
    while True:
        now = ticks_ms()
        update_ir(now)
        if last_print is None or ticks_diff(now, last_print) >= PRINT_INTERVAL_MS:
            print(get_ir_snapshot())
            last_print = now
        sleep_ms(1)


if __name__ == "__main__":
    main()
