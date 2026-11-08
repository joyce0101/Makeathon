"""L298N motor control based on motor1.py, compatible with main.py.

Left IN1/IN2/ENA -> GP2/GP3/GP4; right IN3/IN4/ENB -> GP6/GP7/GP8.
Remove ENA/ENB jumpers and connect common ground. Call init_motors() before
movement commands. Importing this module does not initialize the hardware.
"""

from machine import Pin, PWM
from time import sleep


PWM_FREQUENCY = 1000
left_reversed = False  # Set True if positive speed turns this wheel backward.
right_reversed = False
left_trim = 1.00  # Reduce the faster wheel's multiplier to balance the robot.
right_trim = 1.00

MOTOR_TEST = False  # Enable only for the standalone test with raised wheels.
TEST_SPEED_PERCENT = 60
RUN_SECONDS = 50
STOP_SECONDS = 1

_enables = []
_directions = []
_last_command = None


def init_motors():
    """Initialize both PWM channels at zero duty and stop both motors."""
    global _enables, _directions
    stop()
    _enables = []
    _directions = []
    left_pin = Pin(4, Pin.OUT, value=0)
    right_pin = Pin(8, Pin.OUT, value=0)
    _enables.append(PWM(left_pin, freq=PWM_FREQUENCY, duty_u16=0))
    _enables.append(PWM(right_pin, freq=PWM_FREQUENCY, duty_u16=0))
    _directions = [Pin(gpio, Pin.OUT, value=0) for gpio in (2, 3, 6, 7)]


def set_motor(in_a, in_b, enable, speed, reversed_direction=False):
    """Set one wheel: positive=forward, negative=reverse, zero=coast stop."""
    global _last_command
    _last_command = None
    speed = max(-100, min(100, int(speed)))
    if reversed_direction:
        speed = -speed

    enable.duty_u16(0)
    in_a.value(1 if speed > 0 else 0)
    in_b.value(1 if speed < 0 else 0)
    enable.duty_u16(abs(speed) * 65535 // 100)


def drive(left_speed, right_speed):
    """Apply independent trim and direction settings to signed wheel speeds."""
    global _last_command
    speeds = (
        max(-100, min(100, round(left_speed * left_trim))),
        max(-100, min(100, round(right_speed * right_trim))),
    )
    reverse = (left_reversed, right_reversed)
    command = (speeds, reverse)
    if command == _last_command:
        return

    # Disable both wheels before changing either direction.
    for enable in _enables:
        enable.duty_u16(0)
    for index, (speed, reversed_direction) in enumerate(zip(speeds, reverse)):
        in_a, in_b = _directions[index * 2:index * 2 + 2]
        set_motor(in_a, in_b, _enables[index], speed, reversed_direction)
    _last_command = command


def forward(speed=60):
    drive(speed, speed)


def backward(speed=60):
    drive(-speed, -speed)


def turn_left(inner_speed=30, outer_speed=65):
    drive(inner_speed, outer_speed)


def turn_right(inner_speed=30, outer_speed=65):
    drive(outer_speed, inner_speed)


def stop():
    """Disable both channels and clear direction pins, even during setup."""
    global _last_command
    for enable in _enables:
        enable.duty_u16(0)
    for pin in _directions:
        pin.value(0)
    _last_command = None


def main():
    """Optional standalone left/right test; main.py uses the control API."""
    if not MOTOR_TEST:
        print("Set MOTOR_TEST=True to run the standalone motor test.")
        return
    try:
        init_motors()
        print("Raise the wheels. Motor test starts in 3 seconds. Ctrl+C to stop.")
        sleep(3)
        for name, speeds in (("Left", (TEST_SPEED_PERCENT, 0)), ("Right", (0, TEST_SPEED_PERCENT))):
            print("{} motor: {} seconds.".format(name, RUN_SECONDS))
            drive(*speeds)
            sleep(RUN_SECONDS)
            stop()
            sleep(STOP_SECONDS)
    finally:
        stop()


if __name__ == "__main__":
    main()
