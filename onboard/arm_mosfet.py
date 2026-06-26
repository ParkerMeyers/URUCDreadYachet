"""Arm motor power MOSFET on GPIO 27 — disabled until the ROV is armed."""

from __future__ import annotations

import atexit

PIN = 27
_handle: int | None = None
_enabled: bool | None = None
_warned = False


def _warn_once(msg: str) -> None:
    global _warned
    if not _warned:
        print(f"[arm-mosfet] {msg}", flush=True)
        _warned = True


def _ensure() -> bool:
    global _handle
    if _handle is not None:
        return True
    try:
        import lgpio
    except ImportError:
        _warn_once("lgpio not available — MOSFET control skipped")
        return False
    try:
        _handle = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(_handle, PIN, 0)
    except (OSError, lgpio.error) as e:
        _warn_once(f"GPIO init failed: {e}")
        if _handle is not None:
            try:
                lgpio.gpiochip_close(_handle)
            except (OSError, lgpio.error):
                pass
        _handle = None
        return False
    return True


def init() -> None:
    """Disable arm power (safe default at startup)."""
    set_enabled(False)


def set_enabled(enabled: bool) -> None:
    global _enabled
    if not _ensure():
        return
    import lgpio

    level = 1 if enabled else 0
    try:
        lgpio.gpio_write(_handle, PIN, level)
    except (OSError, lgpio.error) as e:
        _warn_once(f"GPIO write failed: {e}")
        return
    _enabled = enabled
    print(f"[arm-mosfet] GPIO {PIN} {'ON' if enabled else 'OFF'}", flush=True)


def is_enabled() -> bool | None:
    return _enabled


def shutdown() -> None:
    global _handle
    if _handle is None:
        return
    import lgpio

    try:
        lgpio.gpio_write(_handle, PIN, 0)
        lgpio.gpiochip_close(_handle)
    except (OSError, lgpio.error):
        pass
    _handle = None


atexit.register(shutdown)
