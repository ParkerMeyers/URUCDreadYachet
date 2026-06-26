#!/usr/bin/env python3
"""
MOSFET test — ONBOARD  (runs on the Raspberry Pi)
==================================================
Listens for on/off commands from the topside test script over UDP,
then drives GPIO17 (servo power rail MOSFET) via lgpio.

Run this instead of new_ar.py when you only want to exercise the
MOSFET switch without the full arm stack.

    python3 test/mosfet_test_onboard.py
    python3 test/mosfet_test_onboard.py --gpio 17 --port 5012
"""

import argparse
import json
import socket
import sys

# ── Config ────────────────────────────────────────────────────────────────────
LISTEN_PORT = 5012
MOSFET_GPIO = 17
# ─────────────────────────────────────────────────────────────────────────────

_gpio_h = None
_lgpio = None
_mosfet_on = False


def _init_gpio(gpio: int) -> bool:
    global _gpio_h, _lgpio, _mosfet_on
    try:
        import lgpio as lgpio_mod
    except ImportError as exc:
        print(f"[mosfet-onboard] lgpio not available ({exc})", flush=True)
        print("[mosfet-onboard] Install with:  sudo apt install python3-lgpio", flush=True)
        return False

    _lgpio = lgpio_mod
    _gpio_h = _lgpio.gpiochip_open(0)
    _lgpio.gpio_claim_output(_gpio_h, gpio, 0)
    _mosfet_on = False
    print(f"[mosfet-onboard] lgpio ready — GPIO{gpio} OFF", flush=True)
    return True


def _set_mosfet(on: bool, gpio: int) -> None:
    global _mosfet_on
    _mosfet_on = on
    if _gpio_h is not None and _lgpio is not None:
        _lgpio.gpio_write(_gpio_h, gpio, 1 if on else 0)
    print(f"[mosfet-onboard] MOSFET {'ON' if on else 'OFF'}", flush=True)


def _cleanup(gpio: int) -> None:
    if _gpio_h is not None and _lgpio is not None:
        _lgpio.gpio_write(_gpio_h, gpio, 0)
        _lgpio.gpio_free(_gpio_h, gpio)
        _lgpio.gpiochip_close(_gpio_h)
    print("[mosfet-onboard] GPIO released — MOSFET OFF", flush=True)


def _handle_cmd(cmd: dict, gpio: int) -> None:
    if cmd.get("cmd") == "mosfet" or "state" in cmd or "on" in cmd:
        if "state" in cmd:
            on = bool(cmd["state"])
        elif "on" in cmd:
            on = bool(cmd["on"])
        else:
            on = bool(cmd.get("state", False))
        _set_mosfet(on, gpio)
        return

    if cmd.get("toggle"):
        _set_mosfet(not _mosfet_on, gpio)
        return

    print(f"[mosfet-onboard] Ignored command: {cmd}", flush=True)


def main():
    parser = argparse.ArgumentParser(description="ROV MOSFET test — onboard")
    parser.add_argument("--port", type=int, default=LISTEN_PORT,
                        help=f"UDP listen port (default {LISTEN_PORT})")
    parser.add_argument("--gpio", type=int, default=MOSFET_GPIO,
                        help=f"MOSFET GPIO pin (default {MOSFET_GPIO})")
    args = parser.parse_args()

    if not _init_gpio(args.gpio):
        sys.exit(1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))

    print(f"[mosfet-onboard] Listening on UDP port {args.port}")
    print("[mosfet-onboard] JSON: {\"cmd\": \"mosfet\", \"state\": true|false}")
    print("[mosfet-onboard] Press Ctrl+C to stop (MOSFET turns OFF on exit).")

    try:
        while True:
            data, addr = sock.recvfrom(512)
            try:
                cmd = json.loads(data.decode())
            except Exception:
                print(f"[mosfet-onboard] Bad JSON from {addr[0]}:{addr[1]}", flush=True)
                continue
            _handle_cmd(cmd, args.gpio)
    except KeyboardInterrupt:
        print("\n[mosfet-onboard] Stopping.", flush=True)
    finally:
        sock.close()
        _cleanup(args.gpio)


if __name__ == "__main__":
    main()
