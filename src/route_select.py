"""Motor-stopped startup selection and a hardware-independent gesture counter."""

import config

try:
    from time import ticks_diff
except ImportError:
    # CPython host tests use ordinary monotonic integer timestamps.
    def ticks_diff(ticks1, ticks2):
        return ticks1 - ticks2


class RouteSelector:
    """Count fresh near/release pairs; expose count and one-update event feedback."""

    def __init__(self):
        self.count = 0
        self.event = None
        self._last_sample = None
        self._last_near_sample = None
        self._near_started = None
        self._armed = False
        self._hold_cleared = False
        self._released_since = None
        self._selected = None

    def update(self, now_ms, front_status, front_cm, sample_ms):
        self.event = None
        if self._selected is not None:
            return self._selected
        age = None if sample_ms is None else ticks_diff(now_ms, sample_ms)
        valid = (front_status == "VALID" and front_cm is not None
                 and front_cm >= config.MIN_VALID_CM)
        usable = ((valid or front_status == "NO_ECHO") and age is not None
                  and 0 <= age <= config.MAX_READING_AGE_MS)
        if not usable or sample_ms is None:
            # Unknown data cannot establish either a hold or a clear interval.
            self._near_started = None
            self._last_near_sample = None
            self._released_since = None
            return None
        if self._last_sample is not None and ticks_diff(sample_ms, self._last_sample) <= 0:
            return None
        self._last_sample = sample_ms
        near = valid and front_cm is not None and front_cm <= config.SELECT_NEAR_CM
        release = (front_status == "NO_ECHO"
                   or (valid and front_cm is not None and front_cm > config.SELECT_NEAR_CM + config.SELECT_HYST_CM))
        if near:
            self._released_since = None
            self._armed = True
            if (self._near_started is None or self._last_near_sample is None
                    or ticks_diff(sample_ms, self._last_near_sample) > config.MAX_READING_AGE_MS):
                self._near_started = sample_ms
            self._last_near_sample = sample_ms
            if (not self._hold_cleared
                    and ticks_diff(sample_ms, self._near_started) >= config.SELECT_RESET_MS):
                self.count = 0
                self._hold_cleared = True
                self.event = "RESET"
        elif release:
            self._near_started = None
            self._last_near_sample = None
            if self._armed:
                if not self._hold_cleared:
                    self.count += 1
                    self.event = "GESTURE"
                self._armed = False
                self._hold_cleared = False
                self._released_since = now_ms
                if self.count > 4:
                    self.count = 0
                    self.event = "OVERFLOW"
            elif self._released_since is None:
                self._released_since = now_ms
            if (self.count and self._released_since is not None
                    and ticks_diff(now_ms, self._released_since) >= config.SELECT_CONFIRM_MS):
                self._selected = self.count
                self.event = "SELECTED"
                return self._selected
        else:
            # The hysteresis band preserves the armed gesture, but proves no hold/clear.
            self._near_started = None
            self._last_near_sample = None
            self._released_since = None
        return None


class _StartupUI:
    """Poll hardware even while displaying startup feedback; motors remain off."""

    def __init__(self, motor, ultrasonic, power, led, clock, sleeper):
        self.motor = motor
        self.ultrasonic = ultrasonic
        self.power = power
        self.led = led
        self.clock = clock
        self.sleeper = sleeper
        self.seen_valid = False
        self.snapshot = {}

    def poll(self, selector):
        now = self.clock()
        self.motor.stop()
        self.power.update(now)
        self.ultrasonic.update_ultrasonic(now)
        self.snapshot = self.ultrasonic.get_ultrasonic_snapshot(now)
        status = self.snapshot["front_status"]
        distance = self.snapshot["front_cm"]
        sample = self.snapshot["front_sample_ms"]
        if status == "VALID" and distance is not None and distance >= config.MIN_VALID_CM:
            self.seen_valid = True
        selection = selector.update(now, status, distance, sample)
        return now, selection

    def wait(self, duration_ms, selector, clear=False):
        started = self.clock()
        while True:
            now, unused = self.poll(selector)
            # Confirmation monitors must keep accepting a later cancellation hold.
            selector._selected = None
            if selector.event == "RESET":
                return False
            if clear:
                status = self.snapshot["front_status"]
                distance = self.snapshot["front_cm"]
                sample = self.snapshot["front_sample_ms"]
                age = None if sample is None else ticks_diff(now, sample)
                is_clear = (age is not None and 0 <= age <= config.MAX_READING_AGE_MS
                            and (status == "NO_ECHO" or (status == "VALID" and distance is not None
                                 and distance > config.SELECT_NEAR_CM + config.SELECT_HYST_CM)))
                if not is_clear:
                    started = now
            if ticks_diff(now, started) >= duration_ms:
                return True
            self.sleeper(config.MAIN_LOOP_MS)

    def flashes(self, count, period_ms, selector):
        for unused in range(count):
            self.led.value(1)
            if not self.wait(period_ms, selector):
                return False
            self.led.value(0)
            if not self.wait(period_ms, selector):
                return False
        return True


def select_route(_selector=None):
    """Wait indefinitely for an explicit route; a two-second hold cancels startup."""
    import motor
    led = None
    try:
        from machine import Pin
        from time import ticks_ms, sleep_ms
        import power
        import ultrasonic

        motor.stop()
        try:
            led = Pin(config.LED_PIN, Pin.OUT)
        except Exception:
            led = Pin(config.LED_FALLBACK_GPIO, Pin.OUT)
        led.value(0)
        power.init_power()
        ultrasonic.init_async()
        ui = _StartupUI(motor, ultrasonic, power, led, ticks_ms, sleep_ms)
        selector = RouteSelector() if _selector is None else _selector
        ui.flashes(config.SELECT_BOOT_FLASHES, config.SELECT_FAST_BLINK_MS, selector)
        started = ticks_ms()
        fault_reported = False
        feedback_started = None
        feedback_kind = None
        fault_started = None
        pins = None
        selection_pins = config.ROUTE_SELECT_PINS  # type: tuple[int, int] | None
        if selection_pins is not None:
            pins = [Pin(number, Pin.IN, Pin.PULL_UP) for number in selection_pins]
        while True:
            now, route = ui.poll(selector)
            if pins is not None:
                route = 1 + sum((0 if pin.value() else 1) << bit for bit, pin in enumerate(pins))
            if (pins is None and not ui.seen_valid and not fault_reported
                    and ticks_diff(now, started) >= config.SELECT_SENSOR_FAULT_MS):
                fault_reported = True
                fault_started = now
            if selector.event in ("RESET", "GESTURE", "OVERFLOW"):
                feedback_started = now
                feedback_kind = selector.event
                if feedback_kind == "OVERFLOW":
                    print("Route selection overflow; count cleared")
            if route is None:
                elapsed = None if feedback_started is None else ticks_diff(now, feedback_started)
                if feedback_kind == "RESET" and elapsed is not None and elapsed < config.SELECT_CLEAR_LED_MS:
                    led.value(1)
                elif feedback_kind == "OVERFLOW" and elapsed is not None and elapsed < config.SELECT_ERROR_MS:
                    led.value((elapsed // config.SELECT_FAST_BLINK_MS) % 2 == 0)
                elif feedback_kind == "GESTURE" and elapsed is not None and elapsed < config.SELECT_FEEDBACK_MS:
                    led.value(1)
                elif (fault_started is not None and ticks_diff(now, fault_started)
                      < 2 * config.SELECT_SENSOR_FAULT_FLASHES * config.SELECT_FAST_BLINK_MS):
                    led.value((ticks_diff(now, fault_started) // config.SELECT_FAST_BLINK_MS) % 2 == 0)
                else:
                    led.value(0)
                sleep_ms(config.MAIN_LOOP_MS)
                continue
            # A separate counter watches for the same reset gesture throughout confirmation.
            monitor = RouteSelector()
            cancelled = False
            if pins is not None and not ui.seen_valid and not fault_reported:
                # Jumpers skip gestures, but retain the sonar startup diagnostic.
                remaining = config.SELECT_SENSOR_FAULT_MS - ticks_diff(ticks_ms(), started)
                if remaining > 0:
                    cancelled = not ui.wait(remaining, monitor)
                if not cancelled and not ui.seen_valid:
                    fault_reported = True
                    cancelled = not ui.flashes(config.SELECT_SENSOR_FAULT_FLASHES,
                                               config.SELECT_FAST_BLINK_MS, monitor)
            snapshot = power.get_snapshot()
            if (not cancelled and snapshot.get("vsys") is not None
                    and snapshot["vsys"] < config.VSYS_LOW):
                print("Warning: low VSYS {:.2f} V".format(snapshot["vsys"]))
                led.value(1)
                cancelled = not ui.wait(config.LOW_BATTERY_LED_MS, monitor)
            if not cancelled:
                print("ROUTE = {}".format(route))
                for unused in range(2):
                    if not ui.flashes(route, config.SELECT_SLOW_BLINK_MS, monitor):
                        cancelled = True
                        break
            if not cancelled:
                led.value(1)
                cancelled = not ui.wait(config.START_DELAY_MS, monitor, clear=True)
            if not cancelled:
                return route
            print("Route selection cancelled; count cleared")
            # Reuse the monitor so releasing the reset hold cannot count as a new gesture.
            selector = monitor
            feedback_kind = "RESET"
            feedback_started = ticks_ms()
            if pins is not None:
                # Configured jumpers are reread after cancellation and remain authoritative.
                led.value(1)
                ui.wait(config.SELECT_CLEAR_LED_MS, selector)
    finally:
        motor.stop()
        if led is not None:
            led.value(0)


def decide_resume(saved, count):
    """Return detached running progress, None for a new round, or reject a gesture."""
    route_id = saved["route"]
    if route_id not in (1, 2, 3, 4) or route_id not in config.ROUTES:
        raise ValueError("Invalid saved route")
    route = config.ROUTES[route_id]
    if count == 0 and saved["status"] == "FINISHED":
        return None
    if count in (0, 1):
        result = dict(saved)
        if count == 1 and saved["status"] == "FINISHED" and route["finish"] == "DEAD_END":
            result["dead_end_idx"] = max(0, saved["dead_end_idx"] - 1)
        result["status"] = "RUNNING"
        result["stop_reason"] = ""
        return result
    import run_state
    if count == 2:
        return run_state.make_state(route_id)
    if count == 3:
        checkpoint = None
        for item in route.get("checkpoints", ()):
            if item[1] <= saved["turn_index"]:
                checkpoint = item
        if checkpoint is not None:
            return run_state.make_state(route_id, turn_index=checkpoint[1],
                                        dead_end_idx=checkpoint[2], gap_used=checkpoint[3])
    raise ValueError("Invalid resume gesture or no completed checkpoint")


def _resume_error(ui, selector):
    started = ui.clock()
    while True:
        now, unused = ui.poll(selector)
        selector._selected = None
        if selector.event == "RESET":
            return False
        elapsed = ticks_diff(now, started)
        if elapsed >= config.SELECT_ERROR_MS:
            return True
        ui.led.value((elapsed // config.SELECT_FAST_BLINK_MS) % 2 == 0)
        ui.sleeper(config.MAIN_LOOP_MS)


def _confirm_resume(ui, route, selector):
    """Give resumed runs the same voltage warning and placement time as a new run."""
    if not ui.seen_valid:
        ui.led.value(0)
        if not ui.flashes(config.SELECT_SENSOR_FAULT_FLASHES, config.SELECT_FAST_BLINK_MS, selector):
            return False
    snapshot = ui.power.get_snapshot()
    if snapshot.get("vsys") is not None and snapshot["vsys"] < config.VSYS_LOW:
        print("Warning: low VSYS {:.2f} V".format(snapshot["vsys"]))
        ui.led.value(1)
        if not ui.wait(config.LOW_BATTERY_LED_MS, selector):
            return False
    print("ROUTE = {}".format(route))
    for unused in range(2):
        if not ui.flashes(route, config.SELECT_SLOW_BLINK_MS, selector):
            return False
    ui.led.value(1)
    return ui.wait(config.START_DELAY_MS, selector, clear=True)


def startup():
    """Select a new route or offer a saved-progress resume before any movement."""
    import motor
    led = None
    try:
        motor.stop()
        import run_state
        saved = run_state.load()
        if saved is None:
            return select_route(), None
        from machine import Pin
        from time import ticks_ms, sleep_ms
        import power
        import ultrasonic

        try:
            led = Pin(config.LED_PIN, Pin.OUT)
        except Exception:
            led = Pin(config.LED_FALLBACK_GPIO, Pin.OUT)
        led.value(0)
        power.init_power()
        ultrasonic.init_async()
        ui = _StartupUI(motor, ultrasonic, power, led, ticks_ms, sleep_ms)
        selector = RouteSelector()
        if not ui.flashes(config.SELECT_BOOT_FLASHES, config.SELECT_FAST_BLINK_MS, selector):
            run_state.delete()
            return select_route(_selector=selector), None
        # Only gestures inside the advertised resume window choose an action.
        selector = RouteSelector()
        started = ticks_ms()
        flashes = 3 if saved["status"] == "FINISHED" else 2
        pulse_ms = config.RESUME_LED_FLASH_MS
        cycle_ms = 2 * flashes * pulse_ms + config.RESUME_LED_PAUSE_MS
        while True:
            now, unused = ui.poll(selector)
            selector._selected = None
            if selector.event == "RESET":
                run_state.delete()
                led.value(1)
                ui.wait(config.SELECT_CLEAR_LED_MS, selector)
                return select_route(_selector=selector), None
            if selector.event == "OVERFLOW":
                if not _resume_error(ui, selector):
                    run_state.delete()
                    return select_route(_selector=selector), None
                selector = RouteSelector()
                started = ticks_ms()
                continue
            elapsed = ticks_diff(now, started)
            phase = elapsed % cycle_ms
            led.value(phase < 2 * flashes * pulse_ms and (phase // pulse_ms) % 2 == 0)
            if elapsed < config.RESUME_WINDOW_MS:
                sleep_ms(config.MAIN_LOOP_MS)
                continue
            try:
                progress = decide_resume(saved, selector.count)
            except ValueError:
                print("Invalid resume gesture; window restarted")
                if not _resume_error(ui, selector):
                    run_state.delete()
                    return select_route(_selector=selector), None
                selector = RouteSelector()
                started = ticks_ms()
                continue
            if progress is None:
                return select_route(), None
            if _confirm_resume(ui, saved["route"], selector):
                return saved["route"], progress
            # A held-hand cancellation also removes the archive before fresh selection.
            run_state.delete()
            return select_route(_selector=selector), None
    finally:
        motor.stop()
        if led is not None:
            led.value(0)
