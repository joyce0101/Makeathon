"""Cooperative standard-Pico VSYS monitoring and bounded duty correction.

Each update performs at most one ADC conversion. Sixteen conversions are
spread over 200 ms with the normal 1 ms main loop. No pins are initialized
on import. Wireless boards skip the shared power-sensing pins entirely.
"""

import sys
from machine import ADC, Pin
from time import ticks_diff, ticks_ms

import config


_adc = None
_usb_pin = None
_supported = False
_vsys = None
_usb = None
_last_update_ms = None
_sample_credit = 0
_sample_sum = 0
_sample_count = 0


def init_power():
    """Prepare safe measurement inputs; first valid average arrives later."""
    global _adc, _usb_pin, _supported, _vsys, _usb
    global _last_update_ms, _sample_credit, _sample_sum, _sample_count
    _adc = None
    _usb_pin = None
    _supported = False
    _vsys = None
    _usb = None
    _last_update_ms = ticks_ms()
    _sample_credit = 0
    _sample_sum = 0
    _sample_count = 0
    board = getattr(sys.implementation, "_machine", "")
    if "Pico" in board and ("Pico W" in board or "Pico 2 W" in board):
        print("Voltage compensation disabled on {}: power pins are shared with wireless.".format(board))
        return
    _adc = ADC(config.VSYS_ADC_CHANNEL)
    _usb_pin = Pin(config.USB_SENSE_GPIO, Pin.IN)
    _usb = bool(_usb_pin.value())
    _supported = True


def update(now_ms=None):
    """Sample at most once when due, and publish each complete average."""
    global _last_update_ms, _sample_credit, _sample_sum, _sample_count, _vsys, _usb
    if not _supported or _adc is None or _usb_pin is None:
        return
    now = ticks_ms() if now_ms is None else now_ms
    _usb = bool(_usb_pin.value())
    if _last_update_ms is None:
        _last_update_ms = now
        return
    elapsed = max(0, ticks_diff(now, _last_update_ms))
    _last_update_ms = now
    _sample_credit += elapsed * config.POWER_SAMPLES
    if _sample_credit < config.POWER_UPDATE_MS:
        return
    # Drop missed slots after a delayed loop; never catch up with burst reads.
    _sample_credit %= config.POWER_UPDATE_MS
    _sample_sum += _adc.read_u16()
    _sample_count += 1
    if _sample_count >= config.POWER_SAMPLES:
        _vsys = (_sample_sum / _sample_count) * config.ADC_REFERENCE_V * config.VSYS_DIVIDER / 65535
        _sample_sum = 0
        _sample_count = 0


def get_factor():
    """Return unity until measured battery data can safely be used."""
    if not config.VOLT_COMP or not _supported or _usb or _vsys is None or _vsys <= 0:
        return 1.0
    return max(config.VOLT_COMP_MIN, min(config.VOLT_COMP_MAX, config.VSYS_REF / _vsys))


def get_snapshot():
    """Return a detached voltage/USB/compensation/low-battery snapshot."""
    return {
        "vsys": _vsys,
        "usb": _usb,
        "enabled": bool(config.VOLT_COMP and _supported),
        "factor": get_factor(),
        "low": _vsys is not None and _vsys < config.VSYS_LOW,
    }
