#!/usr/bin/env python3
"""GPIO17 servo power rail MOSFET — shared by new_ar.py and mosfet_test_onboard.py."""
from __future__ import annotations

MOSFET_GPIO = 17

_gpio_h = None
_lgpio = None
_mosfet_on = False
_gpio_available = False


def init_mosfet_gpio(gpio: int = MOSFET_GPIO) -> bool:
    """Open lgpio and drive the MOSFET line low (OFF). Returns True on success."""
    global _gpio_h, _lgpio, _mosfet_on, _gpio_available

    release_mosfet_gpio(gpio)

    try:
        import lgpio as lgpio_mod
    except ImportError as exc:
        _gpio_available = False
        print(f"[mosfet] lgpio not available ({exc})", flush=True)
        print("[mosfet] Install with:  sudo apt install python3-lgpio", flush=True)
        return False

    try:
        _lgpio = lgpio_mod
        _gpio_h = _lgpio.gpiochip_open(0)
        _lgpio.gpio_claim_output(_gpio_h, gpio, 0)
        _mosfet_on = False
        _gpio_available = True
        print(f"[mosfet] lgpio ready — GPIO{gpio} OFF", flush=True)
        return True
    except Exception as exc:
        _gpio_h = None
        _lgpio = None
        _gpio_available = False
        print(f"[mosfet] GPIO init failed ({exc})", flush=True)
        return False


def release_mosfet_gpio(gpio: int = MOSFET_GPIO) -> None:
    """Turn OFF and release GPIO resources."""
    global _gpio_h, _lgpio, _mosfet_on, _gpio_available

    if _gpio_h is not None and _lgpio is not None:
        try:
            _lgpio.gpio_write(_gpio_h, gpio, 0)
            _lgpio.gpio_free(_gpio_h, gpio)
            _lgpio.gpiochip_close(_gpio_h)
        except Exception:
            pass

    _gpio_h = None
    _lgpio = None
    _mosfet_on = False
    _gpio_available = False


def gpio_available() -> bool:
    return _gpio_available and _gpio_h is not None and _lgpio is not None


def mosfet_is_on() -> bool:
    return _mosfet_on


def set_mosfet(on: bool, gpio: int = MOSFET_GPIO) -> bool:
    """Drive the MOSFET line. Re-inits GPIO if a prior claim failed."""
    global _mosfet_on

    if not gpio_available():
        if not init_mosfet_gpio(gpio):
            return False

    _mosfet_on = bool(on)
    _lgpio.gpio_write(_gpio_h, gpio, 1 if _mosfet_on else 0)
    print(f"[mosfet] MOSFET {'ON' if _mosfet_on else 'OFF'}", flush=True)
    return True


def handle_mosfet_cmd(cmd: dict, gpio: int = MOSFET_GPIO) -> bool:
    """
    Apply a JSON UDP command. Matches test/mosfet_test_onboard.py.

    Returns True when the command was a MOSFET/toggle command (even if GPIO failed).
    """
    if cmd.get("cmd") == "mosfet" or "state" in cmd or "on" in cmd:
        if "state" in cmd:
            on = bool(cmd["state"])
        elif "on" in cmd:
            on = bool(cmd["on"])
        else:
            on = bool(cmd.get("state", False))
        set_mosfet(on, gpio)
        return True

    if cmd.get("toggle"):
        set_mosfet(not _mosfet_on, gpio)
        return True

    return False
