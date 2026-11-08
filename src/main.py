"""Runtime-selected, nonblocking competition controller. See config.py/README.md."""

from time import sleep_ms, ticks_diff, ticks_ms
import config
import ir_sensors
import motor
import power
import route_select
import run_state
import ultrasonic


def deg_to_ms(degrees):
    return config.deg_to_ms(degrees)


def _side(value):
    return "L" if value in ("L", "LEFT") else "R"


def _seen(state):
    return state in ("BOTH_BLACK", "LINE_LEFT", "LINE_RIGHT")


class Controller:
    """One selected route, one state, and one shared dead-end action queue."""
    def __init__(self, route_id, now_ms=None):
        self.route_id = route_id
        self.route = dict(config.ROUTES[route_id])
        self.turn_index = 0
        self.dead_ends = list(self.route["dead_ends"])
        self.state = "FOLLOW"
        self.context = {}
        self.entered_ms = ticks_ms() if now_ms is None else now_ms
        self.finished = False
        self.run_status = "RUNNING"
        self.stop_reason = ""
        self.seq = 0
        self._stop_selector = None
        self._stop_released = False
        self._resume_countdown = None
        self._resume_choice = None
        self._resume_error_ms = None
        self._status_led = None
        self.gap_used = False
        self.gap_side = None
        self.gap_completed_ms = None
        self.ignore_obstacles_ms = None
        self._logs = []
        self._last_log_ms = None
        self._last_front_sample = None
        self._close_count = 0
        self._front_is_new = False
        self._clear_since = None
        self._wait_started = None
        self._near_seen_ms = None
        self._near_latched = False
        self._near_cleared_ms = None
        self._no_echo_since = None
        self._last_no_echo_ms = None
        self._force_slow_ms = None
        self._pivot_started_ms = None
        self._pivot_accum_ms = 0
        self.reset_tracking(self.entered_ms)

    def export_state(self):
        """Snapshot logical progress, independent of current scan phase."""
        return run_state.make_state(self.route_id, self.turn_index,
                                    len(self.route["dead_ends"]) - len(self.dead_ends),
                                    self.gap_used, self.run_status, self.stop_reason, self.seq)

    def save_state(self):
        # Route zero is a host/calibration diagnostic and is never persisted.
        if self.route_id in (1, 2, 3, 4):
            self.seq = run_state.save(self.export_state())["seq"]

    def _save_event(self):
        if config.SAVE_ON_EVENTS:
            self.save_state()

    def restore_progress(self, saved, now):
        """Restore route-local counters and discard all transient observations."""
        saved = run_state.make_state(**saved)
        if saved["route"] != self.route_id:
            raise ValueError("Resume route does not match controller")
        self.turn_index = saved["turn_index"]
        self.dead_ends = list(self.route["dead_ends"])[saved["dead_end_idx"]:]
        self.gap_used = bool(saved["gap_used"])
        self.seq = saved["seq"]
        self.finished = False
        self.run_status, self.stop_reason = "RUNNING", ""
        self.gap_side = None
        self.gap_completed_ms = None
        self._last_front_sample = None
        self._force_slow_ms = None
        self._pivot_started_ms = None
        self._pivot_accum_ms = 0
        self.reset_tracking(now)
        self.ignore_obstacles_ms = now

    def start_reacquire(self, now):
        """Find nearby tape without consuming route or dead-end progress."""
        self.reset_tracking(now)
        self.ignore_obstacles_ms = now
        self.context = {"phase": "CHECK", "tries": 0}
        self._transition("REACQUIRE", now, "resume")

    def reset_tracking(self, now_ms=None):
        """Reject all raw direction observations at or before this reset."""
        now = ticks_ms() if now_ms is None else now_ms
        motor.stop()
        self.last_side = None
        self.last_side_ms = None
        self.cleared_ms = now
        self.last_confirmed = None
        self.context = {}
        self._wait_started = None
        self._clear_near(now)
        self._transition("FOLLOW", now, "reset")

    def _clear_near(self, now):
        self._near_seen_ms = None
        self._near_latched = False
        self._near_cleared_ms = now
        self._close_count = 0
        self._no_echo_since = None
        self._last_no_echo_ms = None

    def _transition(self, state, now, reason):
        if self._pivot_started_ms is not None:
            self._pivot_accum_ms += max(0, ticks_diff(now, self._pivot_started_ms))
            self._pivot_started_ms = None
        if state in ("FOLLOW", "STOP"):
            if self._pivot_accum_ms >= config.deg_to_ms(config.TURN_CLEAR_DEG):
                self._clear_near(now)
            self._pivot_accum_ms = 0
        old = self.state
        self.state, self.entered_ms = state, now
        self._wait_started = None
        if config.DEBUG:
            self._logs.append("[{}] {} -> {} reason={} turn={}/{} dead_end_q={}".format(
                now, old, state, reason, self.turn_index, len(self.route["turns"]), self.dead_ends))
        self._flush_logs(now)

    def _flush_logs(self, now):
        if config.DEBUG and self._logs and (self._last_log_ms is None or ticks_diff(now, self._last_log_ms) >= config.LOG_INTERVAL_MS):
            print(self._logs.pop(0))
            self._last_log_ms = now

    def _brake_to(self, state, context, now, reason):
        motor.brake()
        self.context = {"next": state, "context": context, "reason": reason}
        self._transition("OBSTACLE_BRAKE", now, reason)

    def _finish(self, now, reason="FINISH_DEAD_END"):
        aliases = {"uturn_timeout": "UTURN_LIMIT", "gap_timeout": "GAP_TIMEOUT",
                   "gap_turn_no_departure": "GAP_TURN_FAIL"}
        reason = aliases.get(reason, reason)
        self.finished = reason in ("FINISH_PLAN", "FINISH_DEAD_END")
        self.run_status = "FINISHED" if self.finished else "LOST"
        self.stop_reason = reason
        self._stop_selector = None
        self._stop_released = False
        self._resume_countdown = None
        self._resume_choice = None
        self._resume_error_ms = None
        self._brake_to("STOP", {}, now, reason)

    def _resume(self, data, now, reason, front=None):
        self.last_side = None
        self.last_side_ms = None
        self.cleared_ms = now
        self.last_confirmed = None
        self.context = {}
        self._transition("FOLLOW", now, reason)
        self._follow_output(data["state"], front)

    def _remember(self, data, now):
        state = data["state"]
        if state in ("LINE_LEFT", "LINE_RIGHT"):
            self.last_side = "L" if state == "LINE_LEFT" else "R"
            self.last_side_ms = now
        raw_ms = data.get("last_side_ms")
        raw_side = data.get("last_seen_side")
        if raw_ms is not None and raw_side in ("LEFT", "RIGHT"):
            if ticks_diff(raw_ms, self.cleared_ms) > 0 and (self.last_side_ms is None or ticks_diff(raw_ms, self.last_side_ms) > 0):
                self.last_side, self.last_side_ms = _side(raw_side), raw_ms

    def _recent(self, now):
        return self.last_side_ms is not None and 0 <= ticks_diff(now, self.last_side_ms) <= config.RECENT_SIDE_MS

    def _preferred_side(self, now):
        if self.gap_completed_ms is not None and ticks_diff(now, self.gap_completed_ms) < config.GAP_BIAS_MS:
            return self.gap_side
        follow = self.route["follow"]
        if follow == "EDGE_RIGHT":
            return "L" if self.last_confirmed == "LINE_LEFT" else "R"
        if follow == "EDGE_LEFT":
            return "R" if self.last_confirmed == "LINE_RIGHT" else "L"
        turns = self.route["turns"]
        if self.turn_index < len(turns):
            return turns[self.turn_index]
        return self.last_side if self._recent(now) else _side(config.DEFAULT_SEARCH_SIDE)

    def _obstacle_action(self, now):
        if self.route["obstacle"] == "NONE" or self.turn_index < self.route["obstacle_after_turns"]:
            return "NONE"
        if self.ignore_obstacles_ms is not None and ticks_diff(now, self.ignore_obstacles_ms) < config.OBSTACLE_IGNORE_MS:
            return "NONE"
        action = self.route["obstacle"]
        return "STOP_WAIT" if self.gap_used and action in ("GAP_LEFT", "GAP_RIGHT") else action

    def _observe_front(self, sonar, now):
        """Only distinct, recent valid measurement IDs advance confirmation."""
        sample = sonar.get("front_sample_ms")
        status = sonar.get("front_status", "PENDING")
        distance = sonar.get("front_cm")
        raw = sonar.get("front_raw_cm", distance)
        fresh = sample is not None and 0 <= ticks_diff(now, sample) <= config.MAX_READING_AGE_MS
        valid = (fresh and status == "VALID" and distance is not None
                 and raw is not None and raw >= config.MIN_VALID_CM
                 and distance >= config.MIN_VALID_CM)
        self._front_is_new = (fresh and sample is not None and (self._last_front_sample is None
                             or ticks_diff(sample, self._last_front_sample) > 0))
        if self._front_is_new and sample is not None:
            self._last_front_sample = sample
            after_clear = self._near_cleared_ms is None or ticks_diff(sample, self._near_cleared_ms) >= 0
            if valid and after_clear and distance is not None:
                if distance > config.OBSTACLE_SLOW_CM + config.NEAR_CLEAR_MARGIN_CM:
                    self._clear_near(now)
                elif distance <= config.OBSTACLE_SLOW_CM:
                    self._near_seen_ms = sample
                self._close_count = self._close_count + 1 if distance <= config.OBSTACLE_STOP_CM else 0
                if self._close_count >= config.OBSTACLE_CONFIRM_READINGS:
                    self._near_latched = True
            elif after_clear:
                recent_near = (self._near_seen_ms is not None
                               and 0 <= ticks_diff(now, self._near_seen_ms) <= config.NEAR_MEMORY_MS)
                blind = status in ("NO_ECHO", "INVALID") or (raw is not None and raw < config.MIN_VALID_CM)
                if blind and (recent_near or self._near_latched):
                    self._near_latched = True
                    # A near-to-blind transition is stronger evidence than a lone Echo.
                    self._close_count = config.OBSTACLE_CONFIRM_READINGS
                else:
                    self._close_count = 0
        if not valid and not self._near_latched:
            self._close_count = 0
        if not valid:
            self._clear_since = None
        if fresh and status == "NO_ECHO" and not self._near_latched:
            # Policy-only clearance; the raw snapshot still reports no distance.
            return float("inf")
        return distance if valid else None

    def _pivot(self, side):
        if self._pivot_started_ms is None:
            self._pivot_started_ms = ticks_ms()
        speed = config.PIVOT_SPEED_PERCENT
        motor.drive(-speed, speed) if side == "L" else motor.drive(speed, -speed)

    def _follow_output(self, state, front):
        base = self.route.get("base_speed", config.BASE_SPEED_PERCENT)
        if self._force_slow_ms is not None and ticks_diff(ticks_ms(), self._force_slow_ms) < config.RECOVERY_SLOW_MS:
            base = min(base, config.SLOW_SPEED_PERCENT)
        if front is not None and front < config.OBSTACLE_SLOW_CM:
            base = min(base, config.SLOW_SPEED_PERCENT)
        inner = config.TURN_INNER_PERCENT if config.INNER_MODE == "REVERSE" else 0
        brake_zero = config.INNER_MODE == "BRAKE"
        gentle = base * config.EDGE_INNER_RATIO
        mode = self.route["follow"]
        if mode == "EDGE_RIGHT":
            speeds = {"BOTH_BLACK": (base, gentle), "LINE_LEFT": (gentle, base), "LINE_RIGHT": (base, inner)}
        elif mode == "EDGE_LEFT":
            speeds = {"BOTH_BLACK": (gentle, base), "LINE_LEFT": (inner, base), "LINE_RIGHT": (base, gentle)}
        else:
            speeds = {"BOTH_BLACK": (base, base), "LINE_LEFT": (inner, base), "LINE_RIGHT": (base, inner)}
        if state in speeds:
            motor.drive(speeds[state][0], speeds[state][1], brake_zero=brake_zero)
        else:
            motor.stop()

    def _hold_front(self, now):
        if self._wait_started is None:
            self._wait_started = now
            motor.brake()
        elif ticks_diff(now, self._wait_started) >= config.BRAKE_MS:
            motor.stop()

    def _start_search(self, now):
        side = self._preferred_side(now)
        context = {"side": side, "initial": side, "phase": 0, "accum_ms": 0}
        self._brake_to("SEARCH", context, now, "line_lost")

    def _dead_end(self, data, now):
        if self.turn_index < len(self.route["turns"]):
            self._finish(now, "SEARCH_FAIL")
            return
        action = self.dead_ends.pop(0) if self.dead_ends else "FINISH"
        # Stop the motors before a potentially blocking flash transaction.
        motor.brake()
        self._save_event()
        if action == "UTURN":
            side = self.context["side"] if self.state == "SEARCH" else ("L" if self.route["follow"] == "EDGE_LEFT" else "R")
            self._brake_to("UTURN", {"side": side, "left_line": data["state"] == "NO_LINE"}, now, "dead_end_uturn")
        else:
            self._finish(now)

    def _handle_follow(self, data, sonar, now, front):
        state = data["state"]
        self._remember(data, now)
        if state == "NO_LINE":
            if self.route["finish"] == "AFTER_TURNS" and self.turn_index >= len(self.route["turns"]):
                self._brake_to("FINISH_CHECK", {"phase": 0, "side": "L"}, now, "plan_complete")
                return
            creep = self.route.get("creep_on_loss_ms", 0)
            if creep and self.turn_index == 0 and not self._recent(now):
                self._brake_to("GAP_DRIVE", {"creep": True, "phase": "OUT", "limit": creep}, now, "optional_creep")
            else:
                self._start_search(now)
            return
        if state == "UNSTABLE":
            self._hold_front(now)
            return
        self.last_confirmed = state
        self._follow_output(state, front)

    def _handle_search(self, data, sonar, now, front):
        scan = self.context
        elapsed = ticks_diff(now, self.entered_ms)
        limit = config.deg_to_ms(config.SEARCH_SIDE_MAX_DEG * (1 if scan["phase"] == 0 else 2))
        if elapsed >= limit:
            if scan["phase"] == 0:
                reverse = "R" if scan["initial"] == "L" else "L"
                context = {"side": reverse, "initial": scan["initial"], "phase": 1, "accum_ms": limit}
                self._brake_to("SEARCH", context, now, "reverse_search")
            else:
                self._dead_end(data, now)
            return
        same_side = data["state"] == ("LINE_LEFT" if scan["side"] == "L" else "LINE_RIGHT")
        if data["state"] == "BOTH_BLACK" or same_side:
            first = config.deg_to_ms(config.SEARCH_SIDE_MAX_DEG)
            offset = elapsed if scan["phase"] == 0 else first - elapsed
            heading = offset if scan["initial"] == "R" else -offset
            turns = self.route["turns"]
            if self.turn_index < len(turns):
                planned_heading = heading if turns[self.turn_index] == "R" else -heading
                if scan["side"] == turns[self.turn_index] and planned_heading >= config.deg_to_ms(config.CORNER_MIN_DEG):
                    self.turn_index += 1
                    motor.stop()
                    self._save_event()
            self._resume(data, now, "search_recovered", front)
            return
        self._pivot(scan["side"])

    def _handle_finish_check(self, data, sonar, now, front):
        if _seen(data["state"]):
            self._resume(data, now, "finish_line_recovered")
            return
        elapsed = ticks_diff(now, self.entered_ms)
        phase = self.context["phase"]
        limit = config.deg_to_ms(config.FINISH_CHECK_DEG * (1 if phase == 0 else 2))
        if elapsed >= limit:
            if phase == 0:
                self._brake_to("FINISH_CHECK", {"phase": 1, "side": "R"}, now, "finish_reverse")
            else:
                self._finish(now, "FINISH_PLAN")
            return
        self._pivot(self.context["side"])

    def _handle_uturn(self, data, sonar, now, front):
        elapsed = ticks_diff(now, self.entered_ms)
        limit = config.deg_to_ms(min(config.UTURN_MAX_DEG, 180 * config.MANEUVER_TIME_MARGIN))
        if elapsed >= limit:
            self._finish(now, "uturn_timeout")
        elif self.context["left_line"] and _seen(data["state"]) and elapsed >= config.deg_to_ms(config.CORNER_MIN_DEG):
            self._clear_near(now)
            self.ignore_obstacles_ms = now
            self._close_count = 0
            self._resume(data, now, "uturn_line_recovered")
        else:
            if data["state"] == "NO_LINE":
                self.context["left_line"] = True
            self._pivot(self.context["side"])

    def _handle_obstacle_brake(self, data, sonar, now, front):
        if ticks_diff(now, self.entered_ms) < config.BRAKE_MS:
            return
        pending = self.context
        motor.stop()
        self.context = pending["context"]
        self._transition(pending["next"], now, pending["reason"])
        if self.state == "STOP":
            motor.stop()
            self.save_state()

    def _handle_stop_wait(self, data, sonar, now, front):
        motor.stop()
        sample = sonar.get("front_sample_ms")
        no_echo = (sonar.get("front_status") == "NO_ECHO" and sample is not None
                   and 0 <= ticks_diff(now, sample) <= config.MAX_READING_AGE_MS)
        if no_echo and self._front_is_new:
            if self._last_no_echo_ms is None or ticks_diff(now, self._last_no_echo_ms) > config.MAX_READING_AGE_MS:
                self._no_echo_since = now
            self._last_no_echo_ms = now
            if self._no_echo_since is not None and ticks_diff(now, self._no_echo_since) >= config.STOP_WAIT_NO_ECHO_MS:
                self._clear_near(now)
                self._force_slow_ms = now
                self._resume(data, now, "no_echo_slow_escape")
                return
        elif not no_echo:
            self._no_echo_since = None
            self._last_no_echo_ms = None
        clear_limit = (config.OBSTACLE_SLOW_CM + config.NEAR_CLEAR_MARGIN_CM
                       if self._near_latched else config.OBSTACLE_STOP_CM)
        if front is None or front <= clear_limit:
            self._clear_since = None
        elif self._front_is_new:
            if self._clear_since is None:
                self._clear_since = now
            elif ticks_diff(now, self._clear_since) >= config.CLEAR_HOLD_MS:
                self._close_count = 0
                self._clear_since = None
                self._resume(data, now, "front_clear", front)

    def _gap_recovered(self, data, now):
        self._clear_near(now)
        self.gap_completed_ms = now
        self.gap_side = self.context.get("side", self.gap_side)
        self._close_count = 0
        self._resume(data, now, "gap_line_recovered")

    def _handle_gap_turn(self, data, sonar, now, front):
        elapsed = ticks_diff(now, self.entered_ms)
        if data["state"] == "NO_LINE":
            self.context["left_line"] = True
        if self.context["left_line"] and _seen(data["state"]) and elapsed >= config.deg_to_ms(config.CORNER_MIN_DEG):
            self._clear_near(now)
            self._gap_recovered(data, now)
        elif elapsed >= config.deg_to_ms(config.GAP_TURN_DEG):
            if not self.context["left_line"]:
                self._finish(now, "gap_turn_no_departure")
            else:
                self._clear_near(now)
                self._brake_to("GAP_DRIVE", {"side": self.context["side"], "creep": False}, now, "gap_heading")
        else:
            self._pivot(self.context["side"])

    def _handle_gap_drive(self, data, sonar, now, front):
        if _seen(data["state"]):
            if self.context.get("creep"):
                self._resume(data, now, "creep_line_recovered")
            else:
                self._gap_recovered(data, now)
            return
        elapsed = ticks_diff(now, self.entered_ms)
        if self.context.get("creep"):
            if elapsed >= self.context["limit"]:
                if self.context["phase"] == "OUT":
                    context = {"creep": True, "phase": "BACK", "limit": self.context["limit"]}
                    self._brake_to("GAP_DRIVE", context, now, "creep_return")
                else:
                    self._start_search(now)
            else:
                speed = config.CREEP_SPEED_PERCENT * (1 if self.context["phase"] == "OUT" else -1)
                motor.drive(speed, speed)
        elif elapsed >= config.GAP_MAX_MS:
            self._finish(now, "gap_timeout")
        else:
            motor.drive(config.GAP_SPEED_PERCENT, config.GAP_SPEED_PERCENT)

    def _handle_reacquire(self, data, sonar, now, front):
        if _seen(data["state"]):
            self._clear_near(now)
            self.ignore_obstacles_ms = now
            self._resume(data, now, "reacquired")
            return
        context = self.context
        phase = context["phase"]
        tries = context["tries"]
        if phase == "CHECK":
            side = _side(config.DEFAULT_SEARCH_SIDE)
            self._brake_to("REACQUIRE", {"phase": "SCAN", "tries": tries,
                           "scan": 0, "side": side}, now, "reacquire_scan")
        elif phase == "CREEP":
            if ticks_diff(now, self.entered_ms) >= config.REACQUIRE_CREEP_MS:
                self._brake_to("REACQUIRE", {"phase": "CHECK", "tries": tries}, now, "reacquire_rescan")
            else:
                motor.drive(config.REACQUIRE_CREEP_PERCENT, config.REACQUIRE_CREEP_PERCENT)
        else:
            limit = config.deg_to_ms(config.SEARCH_SIDE_MAX_DEG * (1 if context["scan"] == 0 else 2))
            if ticks_diff(now, self.entered_ms) < limit:
                self._pivot(context["side"])
            elif context["scan"] == 0:
                reverse = "R" if context["side"] == "L" else "L"
                self._brake_to("REACQUIRE", {"phase": "SCAN", "tries": tries,
                               "scan": 1, "side": reverse}, now, "reacquire_reverse")
            elif tries < config.REACQUIRE_TRIES:
                self._brake_to("REACQUIRE", {"phase": "CREEP", "tries": tries + 1}, now, "reacquire_creep")
            else:
                self._finish(now, "REACQUIRE_FAIL")

    def _stop_led(self, now):
        if self._status_led is None:
            from machine import Pin
            try:
                self._status_led = Pin(config.LED_PIN, Pin.OUT)
            except Exception:
                self._status_led = Pin(config.LED_FALLBACK_GPIO, Pin.OUT)
        if self._resume_error_ms is not None and ticks_diff(now, self._resume_error_ms) < config.SELECT_ERROR_MS:
            value = (ticks_diff(now, self._resume_error_ms) // config.SELECT_FAST_BLINK_MS) % 2 == 0
        elif self._resume_countdown is not None:
            value = True
        else:
            flashes = 3 if self.finished else 2
            pulse = config.RESUME_LED_FLASH_MS
            phase = ticks_diff(now, self.entered_ms) % (2 * flashes * pulse + config.RESUME_LED_PAUSE_MS)
            value = phase < 2 * flashes * pulse and (phase // pulse) % 2 == 0
        self._status_led.value(int(value))

    def _handle_stop(self, data, sonar, now, front):
        motor.stop()
        self._stop_led(now)
        if ticks_diff(now, self.entered_ms) < config.RESUME_ARM_MS:
            return
        if self._stop_selector is None:
            self._stop_selector = route_select.RouteSelector()
        status = sonar.get("front_status", "PENDING")
        distance = sonar.get("front_cm")
        sample = sonar.get("front_sample_ms")
        fresh = sample is not None and 0 <= ticks_diff(now, sample) <= config.MAX_READING_AGE_MS
        clear = bool(fresh and (status == "NO_ECHO" or
                     (status == "VALID" and distance is not None and
                      distance > config.SELECT_NEAR_CM + config.SELECT_HYST_CM)))
        # A stationary obstacle must not masquerade as a two-second hand hold.
        if not self._stop_released:
            self._stop_released = clear
            return
        selector = self._stop_selector
        selector._selected = None
        selector.update(now, status, distance, sample)
        if selector.event == "RESET":
            import machine
            run_state.delete()
            machine.reset()
            self._stop_released = False
            self._stop_selector = None
            self._resume_countdown = None
            return
        if selector.event in ("GESTURE", "OVERFLOW"):
            try:
                self._resume_choice = route_select.decide_resume(self.export_state(), selector.count)
            except ValueError:
                self._resume_countdown = None
                self._resume_error_ms = now
                self._stop_selector = route_select.RouteSelector()
                return
            self._resume_countdown = now
        if self._resume_countdown is not None and not clear:
            # Unknown Echo or a hand still in front cannot complete placement time.
            self._resume_countdown = now
        if (self._resume_countdown is not None and not selector._armed
                and ticks_diff(now, self._resume_countdown) >= config.RESUME_COUNTDOWN_MS):
            self.restore_progress(self._resume_choice, now)
            self.start_reacquire(now)
            self.save_state()
            if self._status_led is not None:
                self._status_led.value(0)

    def update(self, data, sonar, now_ms=None):
        now = ticks_ms() if now_ms is None else now_ms
        if self.state == "STOP":
            self._handle_stop(data, sonar, now, None)
            self._flush_logs(now)
            return self.state
        action = self._obstacle_action(now)
        front = self._observe_front(sonar, now) if action != "NONE" else None
        if self.state in ("FOLLOW", "SEARCH", "FINISH_CHECK") and action != "NONE":
            if self._close_count >= config.OBSTACLE_CONFIRM_READINGS:
                self._close_count = 0
                if action == "DEAD_END":
                    self._dead_end(data, now)
                elif action in ("GAP_LEFT", "GAP_RIGHT"):
                    self.gap_used = True
                    motor.brake()
                    self._save_event()
                    side = "L" if action == "GAP_LEFT" else "R"
                    self.gap_side = side
                    self._brake_to("GAP_TURN", {"side": side, "left_line": data["state"] == "NO_LINE"}, now, "wall_gap")
                else:
                    self._clear_since = None
                    self._brake_to("STOP_WAIT", {}, now, "front_obstacle")
                return self.state
            if self.state == "FOLLOW":
                if self._wait_started is not None and ticks_diff(now, self._wait_started) < config.BRAKE_MS:
                    return self.state
                if front is None:
                    self._hold_front(now)
                    self._flush_logs(now)
                    return self.state
                self._wait_started = None
        handlers = {
            "FOLLOW": self._handle_follow, "SEARCH": self._handle_search,
            "FINISH_CHECK": self._handle_finish_check, "UTURN": self._handle_uturn,
            "OBSTACLE_BRAKE": self._handle_obstacle_brake,
            "STOP_WAIT": self._handle_stop_wait, "GAP_TURN": self._handle_gap_turn,
            "GAP_DRIVE": self._handle_gap_drive, "STOP": self._handle_stop,
            "REACQUIRE": self._handle_reacquire,
        }
        handlers[self.state](data, sonar, now, front)
        self._flush_logs(now)
        return self.state


_controller = None


def reset_tracking(now_ms=None):
    if _controller is not None:
        _controller.reset_tracking(now_ms)
    else:
        motor.stop()


def configure_ir():
    # Keep ir_sensors.py behaviour intact, with all adjustable values in config.
    ir_sensors.SAMPLE_INTERVAL_MS = config.IR_SAMPLE_MS
    ir_sensors.CONFIRM_SAMPLES = config.IR_CONFIRM_SAMPLES
    ir_sensors.LINE_MISSING_MS = config.IR_MISSING_MS
    ir_sensors.PRINT_INTERVAL_MS = config.IR_PRINT_MS
    ir_sensors.init_ir(1, 1)


def main():
    global _controller
    try:
        motor.init_motors()
        power.init_power()
        ultrasonic.init_async()
        selected, progress = route_select.startup()
        configure_ir()
        _controller = Controller(selected, ticks_ms())
        if progress is not None:
            _controller.restore_progress(progress, ticks_ms())
            _controller.start_reacquire(ticks_ms())
        _controller.save_state()
        while True:
            now = ticks_ms()
            power.update(now)
            motor.update(now)
            ultrasonic.update_ultrasonic(now)
            if ir_sensors.update_ir(now):
                _controller.update(ir_sensors.get_ir_snapshot(), ultrasonic.get_ultrasonic_snapshot(now), now)
            sleep_ms(config.MAIN_LOOP_MS)
    finally:
        motor.stop()
        ultrasonic.close_async()


if __name__ == "__main__":
    main()
