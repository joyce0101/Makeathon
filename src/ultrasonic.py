"""IRQ-driven F/F/L ranging; front distances use a three-reading median.

GPIO and timing are defined in config.py. Call init_async() once and poll
update_ultrasonic() without waiting for Echo. IRQs only store small integers.
"""

from machine import Pin, disable_irq, enable_irq
from time import sleep_ms, sleep_us, ticks_diff, ticks_ms, ticks_us
import config


front_trigger = Pin(config.FRONT_ULTRASONIC_PINS[0], Pin.OUT, value=0)
front_echo = Pin(config.FRONT_ULTRASONIC_PINS[1], Pin.IN)
left_trigger = Pin(config.LEFT_ULTRASONIC_PINS[0], Pin.OUT, value=0)
left_echo = Pin(config.LEFT_ULTRASONIC_PINS[1], Pin.IN)
_triggers = (front_trigger, left_trigger)
_echoes = (front_echo, left_echo)
_distances = [None, None]  # type: list[float | None]
_raw_distances = [None, None]  # type: list[float | None]
_sample_times = [None, None]  # type: list[int | None]
_statuses = ["PENDING", "PENDING"]
_front_window = []
_active = None
_phase = 0
_rise_us = 0
_pulse_us = None
_active_started_us = 0
_last_trigger_ms = None
_schedule_index = 0
_running = False


def _echo_edge(index, pin):
    global _phase, _rise_us, _pulse_us
    if index != _active or _phase == 0:
        return
    now = ticks_us()
    if ticks_diff(now, _active_started_us) >= config.ECHO_TIMEOUT_US:
        _pulse_us = -1
        _phase = 0
    elif _phase == 1 and pin.value():
        _rise_us = now
        _phase = 2
    elif _phase == 2 and not pin.value():
        _pulse_us = ticks_diff(now, _rise_us)
        _phase = 0


def _front_edge(pin):
    _echo_edge(0, pin)


def _left_edge(pin):
    _echo_edge(1, pin)


def init_async():
    global _last_trigger_ms, _schedule_index, _running
    close_async()
    _distances[:] = [None, None]
    _raw_distances[:] = [None, None]
    _sample_times[:] = [None, None]
    _statuses[:] = ["PENDING", "PENDING"]
    _front_window.clear()
    _last_trigger_ms = None
    _schedule_index = 0
    edges = Pin.IRQ_RISING | Pin.IRQ_FALLING
    front_echo.irq(handler=_front_edge, trigger=edges, hard=True)
    left_echo.irq(handler=_left_edge, trigger=edges, hard=True)
    _running = True


def _publish(index, width):
    distance = width * 0.0343 / 2 if width > 0 else None
    status = "NO_ECHO" if distance is None else "VALID"
    if distance is not None and distance < config.MIN_VALID_CM:
        distance = None
        status = "INVALID"
    _raw_distances[index] = distance
    if index == 0:
        if distance is None:
            _front_window.clear()
        else:
            _front_window.append(distance)
            if len(_front_window) > config.FRONT_MEDIAN_SAMPLES:
                del _front_window[0]
            values = sorted(_front_window)
            middle = len(values) // 2
            distance = values[middle] if len(values) % 2 else (values[middle - 1] + values[middle]) / 2
    _distances[index] = distance
    _statuses[index] = status
    _sample_times[index] = _last_trigger_ms


def update_ultrasonic(now_ms=None):
    """Process one completion and optionally trigger one due scheduled sensor."""
    global _active, _phase, _pulse_us, _active_started_us, _last_trigger_ms, _schedule_index
    if not _running:
        return False
    now = ticks_ms() if now_ms is None else now_ms
    updated = False
    if _active is not None:
        irq_state = disable_irq()
        width = _pulse_us
        if width is None and ticks_diff(ticks_us(), _active_started_us) >= config.ECHO_TIMEOUT_US:
            width = -1
            _phase = 0
        enable_irq(irq_state)
        if width is not None:
            _publish(_active, width)
            _active = None
            updated = True
    if _active is None and (_last_trigger_ms is None or ticks_diff(now, _last_trigger_ms) >= config.ASYNC_TRIGGER_GAP_MS):
        _active = 0 if config.SCHEDULE[_schedule_index] == "F" else 1
        _schedule_index = (_schedule_index + 1) % len(config.SCHEDULE)
        _last_trigger_ms = now
        _active_started_us = ticks_us()
        _pulse_us = None
        _phase = 1
        trigger = _triggers[_active]
        trigger.value(0)
        sleep_us(2)
        trigger.value(1)
        sleep_us(10)
        trigger.value(0)
    return updated


def get_ultrasonic_snapshot(now_ms=None):
    now = ticks_ms() if now_ms is None else now_ms
    result = {}
    for index, name in enumerate(("front", "left")):
        sample = _sample_times[index]
        age = None if sample is None else ticks_diff(now, sample)
        status = _statuses[index]
        if age is not None and age > config.MAX_READING_AGE_MS:
            status = "STALE"
        result[name + "_cm"] = _distances[index]
        result[name + "_raw_cm"] = _raw_distances[index]
        result[name + "_status"] = status
        result[name + "_age_ms"] = age
        result[name + "_sample_ms"] = sample
    return result


def close_async():
    global _running, _active, _phase
    _running = False
    _active = None
    _phase = 0
    for echo in _echoes:
        echo.irq(handler=None)
    for trigger in _triggers:
        trigger.value(0)


def main():
    last_print = None
    try:
        init_async()
        while True:
            now = ticks_ms()
            update_ultrasonic(now)
            if last_print is None or ticks_diff(now, last_print) >= config.ULTRASONIC_PRINT_INTERVAL_MS:
                print(get_ultrasonic_snapshot(now))
                last_print = now
            sleep_ms(config.MAIN_LOOP_MS)
    finally:
        close_async()


if __name__ == "__main__":
    main()
