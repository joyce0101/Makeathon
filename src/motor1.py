from machine import Pin, PWM
from time import sleep

# Left motor (Motor A)
left_in1 = Pin(2, Pin.OUT)
left_in2 = Pin(3, Pin.OUT)
ena = PWM(Pin(4))

# Right motor (Motor B)
right_in3 = Pin(6, Pin.OUT)
right_in4 = Pin(7, Pin.OUT)
enb = PWM(Pin(8))

# PWM frequency
ena.freq(1000)
enb.freq(1000)

# Motors initially stopped
ena.duty_u16(0)
enb.duty_u16(0)


# ---------------- MOTOR SETTINGS ----------------

# Change to True if a motor spins backwards
left_reversed = False
right_reversed = False

# Adjust if one motor runs faster than the other
left_trim = 1.00
right_trim = 1.00


# ---------------- MOTOR CONTROL ----------------

def set_motor(in_a, in_b, enable, speed, reversed_direction=False):
    """
    Speed ranges from -100 to +100.

    Positive = Forward
    Negative = Reverse
    Zero     = Stop
    """

    speed = max(-100, min(100, int(speed)))

    if reversed_direction:
        speed = -speed

    # Disable motor before changing direction
    enable.duty_u16(0)

    if speed > 0:
        in_a.value(1)
        in_b.value(0)

    elif speed < 0:
        in_a.value(0)
        in_b.value(1)

    else:
        in_a.value(0)
        in_b.value(0)

    # Convert percentage to 16-bit PWM
    duty = abs(speed) * 65535 // 100
    enable.duty_u16(duty)


def drive(left_speed, right_speed):
    """Control both motors independently."""

    left_speed = round(left_speed * left_trim)
    right_speed = round(right_speed * right_trim)

    set_motor(left_in1, left_in2, ena, left_speed, left_reversed)
    set_motor(right_in3, right_in4, enb, right_speed, right_reversed)


# ---------------- MOVEMENT FUNCTIONS ----------------

def forward(speed=60):
    drive(speed, speed)


def backward(speed=60):
    drive(-speed, -speed)


def turn_left():
    drive(30, 65)


def turn_right():
    drive(65, 30)


def stop():
    drive(0, 0)


# ---------------- MOTOR TEST ----------------

# Change to True to run the motor test.
# Raise the wheels off the ground first.

MOTOR_TEST = False

if MOTOR_TEST:
    try:
        print("Motor test starting in 3 seconds...")
        sleep(3)

        print("Testing left motor")
        drive(55, 0)
        sleep(1)
        stop()
        sleep(1)

        print("Testing right motor")
        drive(0, 55)
        sleep(1)
        stop()
        sleep(1)

        print("Testing both motors")
        forward(55)
        sleep(2)
        stop()

        print("Motor test complete")

    finally:
        stop()