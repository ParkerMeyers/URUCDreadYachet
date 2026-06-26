#!/usr/bin/env python3
"""
MOSFET test — TOPSIDE  (runs on your control laptop)
=====================================================
Toggle the servo power rail MOSFET (GPIO17 on the Pi) by typing commands.
Sends UDP packets to mosfet_test_onboard.py running on the Pi.

Usage:
    python test/mosfet_test_topside.py
    python test/mosfet_test_topside.py --ip 192.168.69.100

Commands at the prompt:
    on          enable servo power rail  (also: 1)
    off         cut servo power rail     (also: 0)
    toggle      flip current state       (also: t)
    status      print last known state   (also: s)
    q           turn OFF then exit
"""

import argparse
import json
import socket

# ── Config ────────────────────────────────────────────────────────────────────
DEFAULT_PI_IP = "192.168.69.100"
TEST_PORT     = 5012          # Must match mosfet_test_onboard.py
# ─────────────────────────────────────────────────────────────────────────────


def send(sock, addr, on: bool):
    payload = {"cmd": "mosfet", "state": on}
    sock.sendto(json.dumps(payload).encode(), addr)


def send_toggle(sock, addr):
    sock.sendto(json.dumps({"toggle": True}).encode(), addr)


def print_state(on: bool):
    print(f"  State: MOSFET {'ON' if on else 'OFF'}")


def main():
    parser = argparse.ArgumentParser(description="ROV MOSFET test — topside")
    parser.add_argument("--ip", default=DEFAULT_PI_IP,
                        help=f"Pi IP address (default {DEFAULT_PI_IP})")
    parser.add_argument("--port", type=int, default=TEST_PORT,
                        help=f"UDP port (default {TEST_PORT})")
    args = parser.parse_args()

    pi = (args.ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"MOSFET test — sending to {args.ip}:{args.port}")
    print("Make sure mosfet_test_onboard.py is running on the Pi first.\n")
    print("Controls GPIO17 servo power rail via lgpio.")
    print("Commands:  on (1)  |  off (0)  |  toggle (t)  |  status (s)  |  q\n")

    current_on = False

    try:
        while True:
            try:
                line = input("mosfet> ").strip().lower()
            except EOFError:
                break

            if not line:
                continue

            if line in ("q", "quit", "exit"):
                break

            if line in ("on", "1", "true"):
                current_on = True
                send(sock, pi, True)
                print("  → MOSFET ON")
                print_state(current_on)
                continue

            if line in ("off", "0", "false"):
                current_on = False
                send(sock, pi, False)
                print("  → MOSFET OFF")
                print_state(current_on)
                continue

            if line in ("t", "toggle"):
                current_on = not current_on
                send_toggle(sock, pi)
                print(f"  → MOSFET {'ON' if current_on else 'OFF'} (toggle)")
                print_state(current_on)
                continue

            if line in ("s", "status"):
                print_state(current_on)
                continue

            print("  Usage:  on | off | toggle | status | q")

    except KeyboardInterrupt:
        pass

    print("\nExiting — sending MOSFET OFF to Pi.")
    send(sock, pi, False)
    sock.close()


if __name__ == "__main__":
    main()
