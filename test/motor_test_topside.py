#!/usr/bin/env python3
"""
Motor test — TOPSIDE  (runs on your control laptop)
====================================================
Lets you manually set PWM for individual Pix6 motor outputs by typing
commands.  Sends UDP packets to the onboard script on the Pi.

Usage:
    python test/motor_test_topside.py
    python test/motor_test_topside.py --ip 192.168.69.100

Commands at the prompt:
    1 1600      set motor 1 to 1600 µs  (forward)
    1 1400      set motor 1 to 1400 µs  (reverse)
    1 1500      set motor 1 back to neutral
    all         all motors forward at 1600 µs  (also: a)
    all 1600    set all motors to 1600 µs
    neutral     all motors → 1500 µs  (also: n)
    status      print current PWM state
    q           neutral all then exit

Motor map (matches SERVO1-8 RCPassThru on the Pix6):
    1  front_right_h     5  front_left_v
    2  front_right_v     6  front_left_h
    3  back_right_h      7  back_left_h
    4  back_left_v       8  back_right_v
"""

import argparse
import json
import socket
import sys

# ── Config ────────────────────────────────────────────────────────────────────
DEFAULT_PI_IP = "192.168.69.100"
TEST_PORT     = 5010          # Must match motor_test_onboard.py
NEUTRAL_US    = 1500
MIN_US        = 1100
MAX_US        = 1900
NUM_CHANNELS  = 8
# ─────────────────────────────────────────────────────────────────────────────

MOTOR_NAMES = {
    1: "front_right_h",
    2: "front_right_v",
    3: "back_right_h",
    4: "back_left_v",
    5: "front_left_v",
    6: "front_left_h",
    7: "back_left_h",
    8: "back_right_v",
}


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def send(sock, addr, payload):
    sock.sendto(json.dumps(payload).encode(), addr)


def send_all_motors(sock, addr, us):
    """Set every motor — one UDP packet per channel (works with any onboard version)."""
    for ch in range(1, NUM_CHANNELS + 1):
        send(sock, addr, {"motor": ch, "pwm": us})


def print_state(current):
    cols = "  ".join(f"M{ch}={current[ch]}" for ch in sorted(current))
    print(f"  State: {cols}")


def main():
    parser = argparse.ArgumentParser(description="ROV motor PWM test — topside")
    parser.add_argument("--ip",   default=DEFAULT_PI_IP,
                        help=f"Pi IP address (default {DEFAULT_PI_IP})")
    parser.add_argument("--port", type=int, default=TEST_PORT,
                        help=f"UDP port (default {TEST_PORT})")
    args = parser.parse_args()

    pi   = (args.ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"Motor test — sending to {args.ip}:{args.port}")
    print(f"Make sure motor_test_onboard.py is running on the Pi first.\n")
    print("Motor map:")
    for n, name in MOTOR_NAMES.items():
        print(f"  {n}  {name}")
    print(f"\nPWM range: {MIN_US}–{MAX_US} µs   neutral = {NEUTRAL_US}")
    print("Commands:  <motor> <pwm>  |  all [pwm] (a)  |  neutral (n)  |  status (s)  |  q\n")

    current = {ch: NEUTRAL_US for ch in range(1, NUM_CHANNELS + 1)}

    try:
        while True:
            try:
                line = input("motor> ").strip().lower()
            except EOFError:
                break

            if not line:
                continue

            # Quit
            if line in ("q", "quit", "exit"):
                break

            # All motors to same PWM
            parts = line.split()
            if parts and parts[0] in ("all", "a"):
                if len(parts) == 1:
                    us = 1600
                elif len(parts) == 2:
                    try:
                        us = int(parts[1])
                    except ValueError:
                        print("  Bad input — expected: all [pwm 1100-1900]")
                        continue
                else:
                    print("  Bad input — expected: all [pwm 1100-1900]")
                    continue

                us = clamp(us, MIN_US, MAX_US)
                if len(parts) == 2 and us != int(parts[1]):
                    print(f"  (PWM clamped to {us})")

                for ch in current:
                    current[ch] = us
                send_all_motors(sock, pi, us)
                print(f"  → All motors = {us} µs")
                print_state(current)
                continue

            # Neutral all
            if line in ("n", "neutral"):
                for ch in current:
                    current[ch] = NEUTRAL_US
                send_all_motors(sock, pi, NEUTRAL_US)
                print("  → All motors → 1500 µs")
                print_state(current)
                continue

            # Status
            if line in ("s", "status"):
                print_state(current)
                continue

            # Motor + PWM
            if len(parts) == 2:
                try:
                    ch = int(parts[0])
                    us = int(parts[1])
                except ValueError:
                    print("  Bad input — expected: <motor 1-8> <pwm 1100-1900>")
                    continue

                if not (1 <= ch <= NUM_CHANNELS):
                    print(f"  Motor must be 1-{NUM_CHANNELS}")
                    continue

                us = clamp(us, MIN_US, MAX_US)
                if us != int(parts[1]):
                    print(f"  (PWM clamped to {us})")

                current[ch] = us
                send(sock, pi, {"motor": ch, "pwm": us})
                print(f"  → Motor {ch} ({MOTOR_NAMES[ch]}) = {us} µs")
                print_state(current)
            else:
                print("  Usage:  <motor> <pwm>   e.g.  1 1600")

    except KeyboardInterrupt:
        pass

    print("\nExiting — sending neutral all to Pi.")
    send_all_motors(sock, pi, NEUTRAL_US)
    sock.close()


if __name__ == "__main__":
    main()
