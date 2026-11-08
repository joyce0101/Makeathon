"""Nonblocking front/left HC-SR04 measurements for the Pico.

Front TRIG/ECHO -> GP10/GP11; left -> GP12/GP13.
Use 5 V power, common ground and a separate Echo divider per sensor.
Call init_async() once, update_ultrasonic() in the loop, and read snapshots.
Run this file directly to print both sensors without starting the motors.
"""

from machine import Pin, disable_irq, enable_irq
from time import sleep_ms, sleep_us, ticks_diff, ticks_ms, ticks_us


FRONT_TRIG_GPIO = 10
FRONT_ECHO_GPIO = 11
LEFT_TRIG_GPIO = 12
LEFT_ECHO_GPIO = 13
ECHO_TIMEOUT_US = 30000
ASYNC_TRIGGER_GAP_MS = 70
MAX_READING_AGE_MS = 300

front_trigger = Pin(FRONT_TRIG_GPIO, Pin.OUT, value=0)
front_echo = Pin(FRONT_ECHO_GPIO, Pin.IN)
left_trigger = Pin(LEFT_TRIG_GPIO, Pin.OUT, value=0)
left_echo = Pin(LEFT_ECHO_GPIO, Pin.IN)
_triggers = (front_trigger, left_trigger)
_echoes = (front_echo, left_echo)

_distances = [None, None]  # type: list[float | None]
_sample_times = [None, None]  # type: list[int | None]
_active = None
_phase = 0  # 0=idle, 1=waiting for rise, 2=waiting for fall.
_rise_us = 0
_pulse_us = None
_active_started_us = 0
_last_trigger_ms = None
_next_sensor = 0
_running = False


def _echo_edge(index, pin):
    """IRQ: capture integer timestamps only; calculate distance in the loop."""
    global _phase, _rise_us, _pulse_us
    if index != _active or _phase == 0:
        return
    now = ticks_us()
    if ticks_diff(now, _active_started_us) >= ECHO_TIMEOUT_US:
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
    """Reset readings and attach both Echo edge callbacks."""
    global _last_trigger_ms, _next_sensor, _running
    close_async()
    _distances[:] = [None, None]
    _sample_times[:] = [None, None]
    _last_trigger_ms = None
    _next_sensor = 0
    edges = Pin.IRQ_RISING | Pin.IRQ_FALLING
    front_echo.irq(handler=_front_edge, trigger=edges, hard=True)
    left_echo.irq(handler=_left_edge, trigger=edges, hard=True)
    _running = True


def update_ultrasonic(now_ms=None):
    """Finish one reading or send a due trigger; never wait for an Echo."""
    global _active, _phase, _pulse_us, _active_started_us, _last_trigger_ms, _next_sensor
    if not _running:
        return False
    now = ticks_ms() if now_ms is None else now_ms
    updated = False
    if _active is not None:
        irq_state = disable_irq()
        width = _pulse_us
        if width is None and ticks_diff(ticks_us(), _active_started_us) >= ECHO_TIMEOUT_US:
            width = -1
            _phase = 0
        enable_irq(irq_state)
        if width is not None:
            _distances[_active] = width * 0.0343 / 2 if width > 0 else None
            # Use the trigger time so main.py can verify a new heading reading.
            _sample_times[_active] = _last_trigger_ms
            _active = None
            updated = True

    if _active is None and (_last_trigger_ms is None or ticks_diff(now, _last_trigger_ms) >= ASYNC_TRIGGER_GAP_MS):
        _active = _next_sensor
        _next_sensor = 1 - _next_sensor
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
    """Return front/left distances, status, age and trigger timestamps."""
    now = ticks_ms() if now_ms is None else now_ms
    result = {}
    for index, name in enumerate(("front", "left")):
        sample = _sample_times[index]
        distance = _distances[index]
        if sample is None:
            age = None
            status = "PENDING"
        else:
            age = ticks_diff(now, sample)
            status = "VALID" if distance is not None else "NO_ECHO"
            if age > MAX_READING_AGE_MS:
                status = "STALE"
        result[name + "_cm"] = distance
        result[name + "_status"] = status
        result[name + "_age_ms"] = age
        result[name + "_sample_ms"] = sample
    return result


def close_async():
    """Detach callbacks and leave both trigger pins LOW."""
    global _running, _active, _phase
    _running = False
    _active = None
    _phase = 0
    for echo in _echoes:
        echo.irq(handler=None)
    for trigger in _triggers:
        trigger.value(0)


def main():
    print("Front/left ultrasonic test. Distances in cm. Ctrl+C to stop.")
    try:
        init_async()
        last_print = None
        while True:
            now = ticks_ms()
            update_ultrasonic(now)
            if last_print is None or ticks_diff(now, last_print) >= 200:
                print(get_ultrasonic_snapshot(now))
                last_print = now
            sleep_ms(1)
    finally:
        close_async()


if __name__ == "__main__":
    main()
