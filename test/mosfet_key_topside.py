#!/usr/bin/env python3
"""
MOSFET key toggle — TOPSIDE  (runs on your control laptop)
============================================================
Press a key to turn the servo power rail MOSFET on or off.
Sends UDP to mosfet_test_onboard.py (or mosfet_service.py) on the Pi.

Usage:
    python test/mosfet_key_topside.py
    python test/mosfet_key_topside.py --ip 192.168.69.100

Keys:
    SPACE or M   toggle ON ↔ OFF
    O            turn ON
    F            turn OFF
    Q            turn OFF and exit
"""

import argparse
import json
import socket
import sys
import time

# ── Config ────────────────────────────────────────────────────────────────────
DEFAULT_PI_IP = "192.168.69.100"
TEST_PORT     = 5012          # Must match mosfet_test_onboard.py (use 5007 for mosfet_service)
# ─────────────────────────────────────────────────────────────────────────────


def send_state(sock, addr, on: bool):
    payload = {"cmd": "mosfet", "state": on}
    sock.sendto(json.dumps(payload).encode(), addr)


def send_toggle(sock, addr):
    sock.sendto(json.dumps({"toggle": True}).encode(), addr)


def show_state(on: bool):
    label = "ON " if on else "OFF"
    bar = "████" if on else "····"
    print(f"\r  MOSFET {label}  [{bar}]  (SPACE/M=toggle  O=on  F=off  Q=quit)  ", end="", flush=True)


def read_key_windows():
    import msvcrt

    if not msvcrt.kbhit():
        return None
    ch = msvcrt.getch()
    if ch in (b"\x00", b"\xe0"):  # arrow / function prefix on Windows
        msvcrt.getch()
        return None
    try:
        return ch.decode("utf-8", errors="ignore").lower()
    except Exception:
        return None


def read_key_unix():
    import select
    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        if not select.select([sys.stdin], [], [], 0)[0]:
            return None
        ch = sys.stdin.read(1)
        return ch.lower() if ch else None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    parser = argparse.ArgumentParser(description="ROV MOSFET toggle — keypress topside")
    parser.add_argument("--ip", default=DEFAULT_PI_IP,
                        help=f"Pi IP address (default {DEFAULT_PI_IP})")
    parser.add_argument("--port", type=int, default=TEST_PORT,
                        help=f"UDP port (default {TEST_PORT}; use 5007 for mosfet_service)")
    args = parser.parse_args()

    pi = (args.ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    read_key = read_key_windows if sys.platform == "win32" else read_key_unix

    print(f"MOSFET key toggle — sending to {args.ip}:{args.port}")
    print("Start mosfet_test_onboard.py on the Pi first (or mosfet_service on port 5007).\n")
    print("Keys:  SPACE/M = toggle   O = on   F = off   Q = quit\n")

    current_on = False
    show_state(current_on)

    try:
        while True:
            key = read_key()
            if key is None:
                time.sleep(0.05)
                continue

            if key == "q":
                break

            if key in (" ", "m"):
                current_on = not current_on
                send_toggle(sock, pi)
                show_state(current_on)
                continue

            if key == "o":
                current_on = True
                send_state(sock, pi, True)
                show_state(current_on)
                continue

            if key == "f":
                current_on = False
                send_state(sock, pi, False)
                show_state(current_on)
                continue

    except KeyboardInterrupt:
        pass

    print("\nExiting — sending MOSFET OFF to Pi.")
    send_state(sock, pi, False)
    sock.close()


if __name__ == "__main__":
    main()
