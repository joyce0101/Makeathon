"""Interactive Pico calibration; import is inert and never starts the motors.

Run with the REPL, choose a test, then unplug USB during START_DELAY_MS for
motor tests. Battery measurements and command history are saved on the Pico
as calibration_last.txt; wheel motion and actual angle require observation.
"""

from machine import Pin
from time import sleep_ms, ticks_diff, ticks_ms, ticks_us

import config
import ir_sensors
import motor
import power
import ultrasonic


def _led():
    try:
        return Pin(config.LED_PIN, Pin.OUT, value=0)
    except (TypeError, ValueError):
        return Pin(config.LED_FALLBACK_GPIO, Pin.OUT, value=0)


def _set_led(led, value):
    if led is not None:
        led.value(value)


def _service(now):
    power.update(now)
    motor.update(now)
    ultrasonic.update_ultrasonic(now)
    ir_sensors.update_ir(now)


def _wait(duration_ms, stats=None):
    """Cooperatively advance every driver, optionally sampling loaded VSYS."""
    started = ticks_ms()
    last_sample = None
    while ticks_diff(ticks_ms(), started) < duration_ms:
        now = ticks_ms()
        _service(now)
        if stats is not None and (last_sample is None or ticks_diff(now, last_sample) >= config.POWER_UPDATE_MS):
            snapshot = power.get_snapshot()
            if snapshot.get("usb") is not False:
                raise ValueError("Battery-only power cannot be confirmed; result is invalid")
            voltage = snapshot.get("vsys")
            if voltage is not None:
                stats["sum"] += voltage
                stats["count"] += 1
                if stats["min"] is None or voltage < stats["min"]:
                    stats["min"] = voltage
            last_sample = now
        sleep_ms(config.MAIN_LOOP_MS)


def _flash(led, count, interval_ms):
    for unused in range(count):
        _set_led(led, 1)
        _wait(interval_ms)
        _set_led(led, 0)
        _wait(interval_ms)


def _battery_ready(led):
    """Leave time to remove USB, then reject a USB-powered motor test."""
    motor.stop()
    _set_led(led, 1)
    _wait(config.START_DELAY_MS)
    _set_led(led, 0)
    # Discard a USB-era voltage window before recording battery VSYS.
    _wait(config.POWER_UPDATE_MS * 2)
    snapshot = power.get_snapshot()
    if snapshot.get("usb") is not False:
        raise ValueError("USB present or sense unavailable; no motor test was started")
    return snapshot


def _brake_stop():
    motor.brake()
    _wait(config.BRAKE_MS)
    motor.stop()


def _stats():
    return {"sum": 0.0, "count": 0, "min": None}


def _save_voltage(records, label, stats):
    snapshot = power.get_snapshot()
    average = stats["sum"] / stats["count"] if stats["count"] else None
    records.append("{} vsys_avg={} vsys_min={} usb={} compensation_enabled={}".format(
        label, average, stats["min"], snapshot.get("usb"), snapshot.get("enabled")))


def _pivot(led, records):
    """Run a timed nominal 360-degree clockwise pivot; measure angle by eye."""
    stats = _stats()
    motor.brake()
    _wait(config.BRAKE_MS)
    motor.drive(config.PIVOT_SPEED_PERCENT, -config.PIVOT_SPEED_PERCENT)
    _wait(config.PIVOT_MS_PER_90 * 4, stats)
    _brake_stop()
    _save_voltage(records, "pivot command={} duration_ms={}".format(
        config.PIVOT_SPEED_PERCENT, config.PIVOT_MS_PER_90 * 4), stats)
    records.append("Actual angle NOT measured. New PIVOT_MS_PER_90 = old * 360 / observed_degrees.")


def _duty_sweep(led, records, maximum):
    """Test raw duty with floor, compensation and soft start temporarily off."""
    saved = (config.MIN_DRIVE_PERCENT, config.VOLT_COMP, config.SOFT_START)
    config.MIN_DRIVE_PERCENT = 0
    config.VOLT_COMP = False
    config.SOFT_START = False
    try:
        command = config.CALIBRATION_START_PERCENT
        step = 1
        while command <= maximum:
            # Pulse count identifies the step while USB is unplugged.
            _flash(led, step, config.SELECT_FAST_BLINK_MS)
            stats = _stats()
            motor.drive(command, command)
            _wait(int(config.RUN_SECONDS * 1000), stats)
            _brake_stop()
            _save_voltage(records, "raw_duty={} left_trim={} right_trim={}".format(
                command, config.LEFT_TRIM, config.RIGHT_TRIM), stats)
            _wait(int(config.STOP_SECONDS * 1000))
            command += config.CALIBRATION_STEP_PERCENT
            step += 1
    finally:
        motor.stop()
        config.MIN_DRIVE_PERCENT, config.VOLT_COMP, config.SOFT_START = saved
    records.append("Starting threshold NOT measured. Record the first reliably starting observed step.")


def _run_motion(choice, led, maximum):
    records = ["test={} PIVOT_MS_PER_90={} MIN_DRIVE_PERCENT={} VSYS_REF={}".format(
        choice, config.PIVOT_MS_PER_90, config.MIN_DRIVE_PERCENT, config.VSYS_REF)]
    success = False
    try:
        snapshot = _battery_ready(led)
        records.append("battery_before={} usb={} enabled={}".format(
            snapshot.get("vsys"), snapshot.get("usb"), snapshot.get("enabled")))
        if choice == "1":
            _pivot(led, records)
        else:
            _duty_sweep(led, records, maximum)
        success = True
        records.append("status=COMPLETE; copy battery VSYS to VSYS_REF only after checking this physical test")
    except BaseException as error:
        records.append("status=FAULT {}".format(error))
        raise
    finally:
        motor.stop()
        # One small result file is replaced per run; it is never written in IRQs.
        with open("calibration_last.txt", "w") as result:
            result.write("\n".join(records) + "\n")
        _flash(led, 3 if success else config.SELECT_SENSOR_FAULT_FLASHES,
               config.SELECT_SLOW_BLINK_MS if success else config.SELECT_FAST_BLINK_MS)
        _set_led(led, 1 if success else 0)


def _diagnostic(choice):
    """Stationary serial diagnostics; Ctrl+C returns to the menu."""
    last_print = None
    interval = config.IR_PRINT_MS if choice == "3" else config.ULTRASONIC_PRINT_INTERVAL_MS
    motor.stop()
    while True:
        now = ticks_ms()
        _service(now)
        if last_print is None or ticks_diff(now, last_print) >= interval:
            if choice == "3":
                left, right = ir_sensors.read_raw_ir()
                print("LEFT={} RIGHT={} (black=1 background=0)".format(left, right))
            elif choice == "4":
                print(ultrasonic.get_ultrasonic_snapshot(now))
            else:
                snapshot = power.get_snapshot()
                print("{} ; VSYS_REF must use battery-only motor-test VSYS, not USB VSYS".format(snapshot))
            last_print = now
        sleep_ms(config.MAIN_LOOP_MS)


def _archive_path_key(path):
    """Compare Pico filesystem paths without permitting dot aliases."""
    parts = []
    for part in path.split("/"):
        if part == "..":
            if parts:
                parts.pop()
        elif part and part != ".":
            parts.append(part.lower())
    return "/".join(parts)


def _benchmark_state_writes():
    """Time complete atomic saves to a scratch file, never the race archive.

    Host tests use a simulated clock. Only running this on the Pico measures
    its filesystem write latency, and real writes briefly block this tool.
    """
    motor.stop()
    path = config.STATE_BENCHMARK_FILE
    if _archive_path_key(path) == _archive_path_key(config.RUN_STATE_FILE):
        raise ValueError("Benchmark path must differ from the real race archive")
    count = config.STATE_WRITE_TEST_SAMPLES
    if count < 1:
        raise ValueError("STATE_WRITE_TEST_SAMPLES must be positive")
    import run_state
    total_us = 0
    maximum_us = 0
    try:
        state = run_state.make_state(1)
        for unused in range(count):
            started = ticks_us()
            state = run_state.save(state, path=path)
            elapsed = ticks_diff(ticks_us(), started)
            total_us += elapsed
            maximum_us = max(maximum_us, elapsed)
        result = {"samples": count, "average_ms": total_us / count / 1000,
                  "maximum_ms": maximum_us / 1000}
        print("Atomic save: samples={} average_ms={:.3f} maximum_ms={:.3f}".format(
            result["samples"], result["average_ms"], result["maximum_ms"]))
        print("Only Pico execution measures Pico flash latency; host test timing is simulated.")
        if result["average_ms"] > config.STATE_WRITE_WARN_MS:
            print("Average exceeds {} ms: set SAVE_ON_EVENTS=False before the event.".format(
                config.STATE_WRITE_WARN_MS))
        return result
    finally:
        motor.stop()
        # delete() also removes an interrupted atomic save's temporary sibling.
        run_state.delete(path=path)


def main():
    """Offer diagnostics without moving until an explicit motion selection."""
    led = None
    try:
        motor.init_motors()
        power.init_power()
        ultrasonic.init_async()
        ir_sensors.SAMPLE_INTERVAL_MS = config.IR_SAMPLE_MS
        ir_sensors.CONFIRM_SAMPLES = config.IR_CONFIRM_SAMPLES
        ir_sensors.LINE_MISSING_MS = config.IR_MISSING_MS
        ir_sensors.init_ir()
        led = _led()
        while True:
            motor.stop()
            print("1 Pivot 360 nominal | 2 Raw duty sweep | 3 IR | 4 Sonar | 5 Power | 6 State write timing | 0 Exit")
            choice = input("Select: ").strip()
            _set_led(led, 0)
            if choice == "0":
                return
            if choice not in ("1", "2", "3", "4", "5", "6"):
                continue
            try:
                if choice in ("1", "2"):
                    maximum = config.CALIBRATION_MAX_PERCENT
                    if choice == "2":
                        value = input("WHEELS RAISED. Maximum raw duty [{}]: ".format(maximum)).strip()
                        maximum = maximum if not value else int(value)
                        if maximum < config.CALIBRATION_START_PERCENT or maximum > config.CALIBRATION_MAX_PERCENT:
                            raise ValueError("Maximum outside configured calibration range")
                    print("Battery connected; clear test area. UNPLUG USB within {} ms.".format(config.START_DELAY_MS))
                    print("Result: calibration_last.txt. COMPLETE=3 slow flashes; FAULT=10 fast flashes.")
                    _run_motion(choice, led, maximum)
                elif choice == "6":
                    _benchmark_state_writes()
                else:
                    _diagnostic(choice)
            except KeyboardInterrupt:
                motor.stop()
                print("Test interrupted; motors stopped.")
            except Exception as error:
                motor.stop()
                print("Calibration fault: {}".format(error))
    finally:
        motor.stop()
        ultrasonic.close_async()
        _set_led(led, 0)


if __name__ == "__main__":
    main()
