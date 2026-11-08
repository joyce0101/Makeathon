"""Nonblocking L298N drive, dead-zone mapping and wheel startup ramps.

Left IN1/IN2/ENA are GP2/GP3/GP4; right IN3/IN4/ENB are GP6/GP7/GP8.
Remove ENA/ENB jumpers and share ground. Importing does not initialize pins.
"""

from machine import Pin, PWM
from time import sleep_ms, ticks_diff, ticks_ms

import config
import power


_enables = []
_directions = []
_targets = [0, 0]
_signs = [0, 0]
_ramp_started = [None, None]  # type: list[int | None]
_last_outputs = [None, None]  # type: list[tuple[int, int, int] | None]
_mode = "STOP"
_brake_zero = False


def init_motors():
    """Initialize both PWM channels at zero, safely stopping an old setup."""
    global _enables, _directions, _last_outputs
    stop()
    for enable in _enables:
        enable.deinit()
    _directions = [Pin(gpio, Pin.OUT, value=0) for gpio in (
        config.LEFT_MOTOR_PINS[0], config.LEFT_MOTOR_PINS[1],
        config.RIGHT_MOTOR_PINS[0], config.RIGHT_MOTOR_PINS[1],
    )]
    _enables = [PWM(Pin(pins[2], Pin.OUT, value=0),
                    freq=config.PWM_FREQUENCY, duty_u16=0)
                for pins in (config.LEFT_MOTOR_PINS, config.RIGHT_MOTOR_PINS)]
    _last_outputs = [None, None]
    stop()


def _sign(value):
    return 1 if value > 0 else -1 if value < 0 else 0


def _write_wheel(index, signed_percent, braking=False):
    """Disable PWM before any direction-pin transition."""
    if braking:
        direction_a, direction_b, duty = 1, 1, 65535
    else:
        direction_a = 1 if signed_percent > 0 else 0
        direction_b = 1 if signed_percent < 0 else 0
        duty = int(min(100, abs(signed_percent)) * 65535 / 100)
    output = (direction_a, direction_b, duty)
    if output == _last_outputs[index]:
        return
    enable = _enables[index]
    previous = _last_outputs[index]
    if previous is None or previous[:2] != output[:2]:
        enable.duty_u16(0)
        _directions[index * 2].value(direction_a)
        _directions[index * 2 + 1].value(direction_b)
    enable.duty_u16(duty)
    _last_outputs[index] = output


def drive(left_speed, right_speed, brake_zero=False):
    """Set signed targets; repeated commands preserve active startup ramps.

    Trim and voltage correction precede the linear dead-zone map. With
    brake_zero=True an idle inner wheel actively brakes.
    """
    global _mode, _brake_zero
    now = ticks_ms()
    for index, speed in enumerate((left_speed, right_speed)):
        sign = _sign(speed)
        if sign != _signs[index]:
            _ramp_started[index] = now if sign and config.SOFT_START else None
        _targets[index] = speed
        _signs[index] = sign
    _mode = "DRIVE"
    _brake_zero = brake_zero
    update(now)


def update(now_ms=None):
    """Advance soft starts and follow current voltage without sleeping."""
    if _mode != "DRIVE" or len(_enables) != 2:
        return
    now = ticks_ms() if now_ms is None else now_ms
    factor = power.get_factor()
    for index in range(2):
        speed = _targets[index]
        if not speed:
            _write_wheel(index, 0, _brake_zero)
            continue
        trim = config.LEFT_TRIM if index == 0 else config.RIGHT_TRIM
        corrected = min(100, abs(speed) * max(0, trim) * factor)
        duty = config.MIN_DRIVE_PERCENT + (100 - config.MIN_DRIVE_PERCENT) * corrected / 100
        started = _ramp_started[index]
        if config.SOFT_START and config.SOFT_START_MS > 0 and started is not None:
            elapsed = max(0, ticks_diff(now, started))
            duty *= min(1, elapsed / config.SOFT_START_MS)
            if elapsed >= config.SOFT_START_MS:
                _ramp_started[index] = None
        else:
            _ramp_started[index] = None
        reversed_direction = config.LEFT_REVERSED if index == 0 else config.RIGHT_REVERSED
        signed_duty = duty * _signs[index] * (-1 if reversed_direction else 1)
        _write_wheel(index, signed_duty)


def _clear_motion(mode):
    global _mode
    _mode = mode
    for index in range(2):
        _targets[index] = 0
        _signs[index] = 0
        _ramp_started[index] = None


def brake():
    """Apply active braking; route state machines time BRAKE_MS."""
    _clear_motion("BRAKE")
    for index in range(len(_enables)):
        _write_wheel(index, 0, True)


def stop():
    """Immediately disable both channels and cancel pending startup ramps."""
    _clear_motion("STOP")
    for index in range(len(_enables)):
        _write_wheel(index, 0)


def forward(speed=None):
    speed = config.MANUAL_SPEED_PERCENT if speed is None else speed
    drive(speed, speed)


def backward(speed=None):
    speed = config.MANUAL_SPEED_PERCENT if speed is None else speed
    drive(-speed, -speed)


def turn_left(inner_speed=None, outer_speed=None):
    inner_speed = config.MANUAL_TURN_INNER_PERCENT if inner_speed is None else inner_speed
    outer_speed = config.MANUAL_TURN_OUTER_PERCENT if outer_speed is None else outer_speed
    drive(inner_speed, outer_speed)


def turn_right(inner_speed=None, outer_speed=None):
    inner_speed = config.MANUAL_TURN_INNER_PERCENT if inner_speed is None else inner_speed
    outer_speed = config.MANUAL_TURN_OUTER_PERCENT if outer_speed is None else outer_speed
    drive(outer_speed, inner_speed)


def _run_test_interval(duration_ms):
    started = ticks_ms()
    while ticks_diff(ticks_ms(), started) < duration_ms:
        now = ticks_ms()
        power.update(now)
        update(now)
        sleep_ms(config.MAIN_LOOP_MS)


def main():
    """Optional raised-wheel test; sleeps belong only to this test runner."""
    if not config.MOTOR_TEST:
        print("Set MOTOR_TEST=True in config.py to run the standalone motor test.")
        return
    try:
        init_motors()
        power.init_power()
        print("Raise the wheels. Motor test starts in {} ms. Ctrl+C to stop.".format(config.MOTOR_TEST_START_DELAY_MS))
        sleep_ms(config.MOTOR_TEST_START_DELAY_MS)
        for name, speeds in (("Left", (config.TEST_SPEED_PERCENT, 0)),
                             ("Right", (0, config.TEST_SPEED_PERCENT))):
            print("{} motor: {} seconds.".format(name, config.RUN_SECONDS))
            drive(*speeds)
            _run_test_interval(int(config.RUN_SECONDS * 1000))
            stop()
            sleep_ms(int(config.STOP_SECONDS * 1000))
    finally:
        stop()


if __name__ == "__main__":
    main()
