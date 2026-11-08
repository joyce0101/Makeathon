"""Print stationary raw IR readings every configured 200 ms; import is inert."""

from time import sleep_ms, ticks_diff, ticks_ms

import config
import ir_sensors
import motor


def main():
    """Check tape/background at the final mounting height; never drive motors."""
    try:
        motor.init_motors()
        ir_sensors.SAMPLE_INTERVAL_MS = config.IR_SAMPLE_MS
        ir_sensors.CONFIRM_SAMPLES = config.IR_CONFIRM_SAMPLES
        ir_sensors.LINE_MISSING_MS = config.IR_MISSING_MS
        ir_sensors.init_ir()
        print("Raw IR: black=1, background=0. Ctrl+C to stop.")
        last_print = None
        while True:
            now = ticks_ms()
            motor.update(now)
            ir_sensors.update_ir(now)
            if last_print is None or ticks_diff(now, last_print) >= config.IR_PRINT_MS:
                left, right = ir_sensors.read_raw_ir()
                print("LEFT={} | RIGHT={}".format(left, right))
                last_print = now
            sleep_ms(config.MAIN_LOOP_MS)
    finally:
        motor.stop()


if __name__ == "__main__":
    main()
