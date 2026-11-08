"""Line following and obstacle-triggered side-line search.

Black=1, background=0; both IR probes must be on the tape when centered.
Front distance <=5 cm starts an in-place search. A new clear measurement is
required before resuming. A search timeout latches STOP until reset.
Upload beside motor.py, ir_sensors.py and ultrasonic.py.
"""

from time import sleep, sleep_ms, ticks_diff, ticks_ms

import ir_sensors
import motor
import ultrasonic


BASE_SPEED_PERCENT = 60  # Tune for reliable starting and controllable speed.
TURN_INNER_PERCENT = 0  # Stop the inner wheel to steer toward the line.
SEARCH_SPEED_PERCENT = 50  # Tune for reliable starting during recovery.
SEARCH_INNER_PERCENT = -25  # Slight reverse on inner wheel for tight pivot search.
SEARCH_FIRST_SWEEP_MS = 600  # Initial sweep duration toward the expected side.
SEARCH_TIMEOUT_MS = 5000  # Maximum duration of one complete two-sided search.
DEFAULT_SEARCH_SIDE = "RIGHT"  # First sweep side when no recent single-probe direction exists.
DIRECTION_MEMORY_MS = 500  # Ignore older single-probe direction observations.
CLEAR_DISTANCE_CM = 999.0  # Distance value used when ultrasonic reports NO_ECHO.
OBSTACLE_DISTANCE_CM = 5
OBSTACLE_TURN_PERCENT = 45
OBSTACLE_BRAKE_MS = 100
OBSTACLE_FIRST_SWEEP_MS = 1000  # Time limits, not calibrated turn angles.
OBSTACLE_SECOND_SWEEP_MS = 2000  # Sweep back across the original heading.
OBSTACLE_TOTAL_TIMEOUT_MS = 5000
FRONT_CHECK_TIMEOUT_MS = 500

_mode = "FOLLOW"
_last_side = None
_last_side_ms = None
_cleared_ms = None
_search_started_ms = 0
_search_initial_mode = None
_obstacle = None


def _clear_direction_memory(now_ms=None):
    """Clear stored turn direction without stopping the motors."""
    global _mode, _last_side, _last_side_ms, _cleared_ms, _search_started_ms, _search_initial_mode
    _mode = "FOLLOW"
    _last_side = None
    _last_side_ms = None
    _cleared_ms = now_ms
    _search_started_ms = 0
    _search_initial_mode = None


def reset_tracking():
    """Stop the motors and clear direction history and any timeout latch."""
    global _obstacle
    motor.stop()
    _clear_direction_memory()
    _obstacle = None


def follow_line(state):
    """Apply a forward/steering command from a confirmed IR observation."""
    if state == "BOTH_BLACK":
        motor.drive(BASE_SPEED_PERCENT, BASE_SPEED_PERCENT)
    elif state == "LINE_LEFT":
        motor.drive(TURN_INNER_PERCENT, BASE_SPEED_PERCENT)
    elif state == "LINE_RIGHT":
        motor.drive(BASE_SPEED_PERCENT, TURN_INNER_PERCENT)
    else:
        motor.stop()


def update_tracking(data, now_ms=None):
    """Apply one confirmed IR snapshot and return the controller mode.

    Call after each new IR sample. No sleeps are used here. Single-probe
    observations guide a search, but do not prove that a corner exists.
    """
    global _mode, _last_side, _last_side_ms, _search_started_ms, _search_initial_mode
    now = ticks_ms() if now_ms is None else now_ms
    state = data["state"]

    if _mode == "STOP":
        motor.stop()
        return _mode

    if _mode == "FOLLOW":
        if state in ("LINE_LEFT", "LINE_RIGHT"):
            _last_side = "LEFT" if state == "LINE_LEFT" else "RIGHT"
            _last_side_ms = now
        elif data.get("last_seen_side") in ("LEFT", "RIGHT") and data.get("last_side_ms") is not None:
            raw_ms = data["last_side_ms"]
            if _cleared_ms is None or ticks_diff(raw_ms, _cleared_ms) > 0:
                _last_side = data["last_seen_side"]
                _last_side_ms = raw_ms

        if state == "NO_LINE":
            recent_side = (
                _last_side_ms is not None
                and 0 <= ticks_diff(now, _last_side_ms) <= DIRECTION_MEMORY_MS
            )
            initial_side = _last_side if recent_side and _last_side is not None else DEFAULT_SEARCH_SIDE
            _mode = "SEARCH_LEFT" if initial_side == "LEFT" else "SEARCH_RIGHT"
            _search_initial_mode = _mode
            _search_started_ms = now
        else:
            follow_line(state)
            return _mode

    elapsed = ticks_diff(now, _search_started_ms)
    if elapsed >= SEARCH_TIMEOUT_MS:
        return _stop_tracking()

    if elapsed >= SEARCH_FIRST_SWEEP_MS and _search_initial_mode is not None:
        _mode = "SEARCH_RIGHT" if _search_initial_mode == "SEARCH_LEFT" else "SEARCH_LEFT"

    same_side = (
        (_mode == "SEARCH_LEFT" and state == "LINE_LEFT")
        or (_mode == "SEARCH_RIGHT" and state == "LINE_RIGHT")
    )
    if state == "BOTH_BLACK" or same_side:
        _clear_direction_memory(now)
        if same_side:
            _last_side = "LEFT" if state == "LINE_LEFT" else "RIGHT"
            _last_side_ms = now
        follow_line(state)
        return _mode

    # Opposite-side samples must not reverse or restart an active search.
    if _mode == "SEARCH_LEFT":
        motor.drive(SEARCH_INNER_PERCENT, SEARCH_SPEED_PERCENT)
    else:
        motor.drive(SEARCH_SPEED_PERCENT, SEARCH_INNER_PERCENT)
    return _mode


def _distance(sonar, direction):
    """Use the validity already determined by ultrasonic.py; NO_ECHO means clear."""
    status = sonar[direction + "_status"]
    if status == "VALID":
        return sonar[direction + "_cm"]
    if status == "NO_ECHO":
        return CLEAR_DISTANCE_CM
    return None


def _stop_tracking():
    """Latch any failed search until the controller is reset."""
    global _mode
    _mode = "STOP"
    motor.stop()
    return "STOP"


def _search_side_line(scan, data, sonar, now):
    """Pivot through both sides, then recheck a newly acquired line heading."""
    if ticks_diff(now, scan["started"]) >= OBSTACLE_TOTAL_TIMEOUT_MS:
        return _stop_tracking()

    if scan["mode"] == "OBSTACLE_BRAKE":
        motor.stop()
        if ticks_diff(now, scan["stage_started"]) < OBSTACLE_BRAKE_MS:
            return "OBSTACLE_BRAKE"
        scan["stage_started"] = now
        scan["mode"] = "SEARCH"

    candidate = scan["candidate"]
    if candidate is not None:
        motor.stop()
        if data["state"] != "BOTH_BLACK":
            scan["candidate"] = None
            scan["left_line"] = data["state"] == "NO_LINE"
        else:
            sample = sonar["front_sample_ms"]
            front = _distance(sonar, "front")
            fresh_heading = sample is not None and ticks_diff(sample, candidate) >= OBSTACLE_BRAKE_MS
            if fresh_heading and front is not None:
                if front > OBSTACLE_DISTANCE_CM:
                    reset_tracking()
                    return update_tracking(data, now)
                # This line still points at an obstacle; leave it and scan on.
                scan["candidate"] = None
                scan["left_line"] = False
            elif ticks_diff(now, candidate) >= FRONT_CHECK_TIMEOUT_MS:
                return _stop_tracking()
            else:
                return "CHECK_FRONT"

    limit = OBSTACLE_FIRST_SWEEP_MS if scan["stage"] == 0 else OBSTACLE_SECOND_SWEEP_MS
    if ticks_diff(now, scan["stage_started"]) >= limit:
        motor.stop()
        if scan["stage"] == 1:
            return _stop_tracking()
        scan["stage"] = 1
        scan["direction"] = "RIGHT" if scan["direction"] == "LEFT" else "LEFT"
        scan["stage_started"] = now
        scan["mode"] = "OBSTACLE_BRAKE"
        scan["left_line"] = False
        return "OBSTACLE_BRAKE"

    if data["state"] == "NO_LINE":
        scan["left_line"] = True
    elif data["state"] == "BOTH_BLACK" and scan["left_line"]:
        # Require leaving the original line before considering any new line.
        scan["candidate"] = now
        motor.stop()
        return "CHECK_FRONT"

    speed = OBSTACLE_TURN_PERCENT
    if scan["direction"] == "LEFT":
        motor.drive(-speed, speed)
        return "OBSTACLE_SEARCH_LEFT"
    motor.drive(speed, -speed)
    return "OBSTACLE_SEARCH_RIGHT"


def update_robot(data, sonar, now_ms=None):
    """Obstacle handling has priority over all forward line-follow commands."""
    global _obstacle
    now = ticks_ms() if now_ms is None else now_ms
    if _mode == "STOP":
        motor.stop()
        return "STOP"
    if _obstacle is not None:
        return _search_side_line(_obstacle, data, sonar, now)
    front = _distance(sonar, "front")
    if front is None:
        motor.stop()
        return "WAIT_FRONT"
    if front <= OBSTACLE_DISTANCE_CM:
        left = _distance(sonar, "left")
        direction = "RIGHT" if left is not None and left <= OBSTACLE_DISTANCE_CM else "LEFT"
        reset_tracking()
        _obstacle = {
            "started": now, "stage_started": now, "stage": 0,
            "direction": direction, "mode": "OBSTACLE_BRAKE",
            "left_line": False, "candidate": None,
        }
        return "OBSTACLE_BRAKE"
    return update_tracking(data, now)


def main():
    try:
        motor.init_motors()
        ir_sensors.init_ir()
        ultrasonic.init_async()
        reset_tracking()
        print("Line follower starts in 3 seconds. Ctrl+C to stop.")
        sleep(3)
        last_status = None
        while True:
            now = ticks_ms()
            ultrasonic.update_ultrasonic(now)
            if ir_sensors.update_ir(now):
                data = ir_sensors.get_ir_snapshot()
                sonar = ultrasonic.get_ultrasonic_snapshot(now)
                mode = update_robot(data, sonar, now)
                status = (data["state"], mode)
                if status != last_status:
                    print("IR: {} -> {} | {} | front={} cm".format(data["raw"], data["state"], mode, sonar["front_cm"]))
                    last_status = status
            sleep_ms(1)
    except KeyboardInterrupt:
        print("Line follower stopped.")
    finally:
        motor.stop()
        ultrasonic.close_async()


if __name__ == "__main__":
    main()
