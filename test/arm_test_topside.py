#!/usr/bin/env python3
"""
Arm joint test — TOPSIDE  (runs on your control laptop)
========================================================
Lets you manually set PWM for individual arm joint outputs by typing
commands.  Sends UDP packets to arm_test_onboard.py running on the Pi.

Usage:
    python test/arm_test_topside.py
    python test/arm_test_topside.py --ip 192.168.2.249

Commands at the prompt:
    1 1600      set AUX1 (J1) to 1600 µs
    1 1500      set AUX1 (J1) back to center
    3 800       set AUX3 (J3) to 800 µs
    center      all joints → 1500 µs  (also: c)
    status      print current PWM state  (also: s)
    q           center all then exit

Joint map (AUX1-8 → Pix6 AUX outputs, RC channels 9-16):
    1  J1      (AUX1 → RC ch 9  → SERVO9)
    2  J2      (AUX2 → RC ch 10 → SERVO10)
    3  J3      (AUX3 → RC ch 11 → SERVO11)
    4  J4      (AUX4 → RC ch 12 → SERVO12)
    5  J5      (AUX5 → RC ch 13 → SERVO13)
    6  J6      (AUX6 → RC ch 14 → SERVO14)
    7  Claw    (AUX7 → RC ch 15 → SERVO15)
    8  AUX8    (AUX8 → RC ch 16 → SERVO16)

Mission Planner parameters required (set once, write params):
    SERVO9_FUNCTION  = 1   SERVO10_FUNCTION = 1   SERVO11_FUNCTION = 1
    SERVO12_FUNCTION = 1   SERVO13_FUNCTION = 1   SERVO14_FUNCTION = 1
    SERVO15_FUNCTION = 1   SERVO16_FUNCTION = 1
    BRD_SAFETYENABLE = 0
"""

import argparse
import json
import socket
import sys

# ── Config ────────────────────────────────────────────────────────────────────
DEFAULT_PI_IP = "192.168.2.249"
TEST_PORT     = 5011          # Must match arm_test_onboard.py
CENTER_US     = 1500
MIN_US        = 500
MAX_US        = 2500
AUX_CHANNELS  = 8
# ─────────────────────────────────────────────────────────────────────────────

JOINT_NAMES = {
    1: "J1",
    2: "J2",
    3: "J3",
    4: "J4",
    5: "J5",
    6: "J6",
    7: "Claw",
    8: "AUX8",
}


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def send(sock, addr, payload):
    sock.sendto(json.dumps(payload).encode(), addr)


def print_state(current):
    cols = "  ".join(
        f"AUX{aux}({JOINT_NAMES[aux]})={current[aux]}"
        for aux in sorted(current)
    )
    print(f"  State: {cols}")


def main():
    parser = argparse.ArgumentParser(description="ROV arm joint PWM test — topside")
    parser.add_argument("--ip",   default=DEFAULT_PI_IP,
                        help=f"Pi IP address (default {DEFAULT_PI_IP})")
    parser.add_argument("--port", type=int, default=TEST_PORT,
                        help=f"UDP port (default {TEST_PORT})")
    args = parser.parse_args()

    pi   = (args.ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"Arm joint test — sending to {args.ip}:{args.port}")
    print("Make sure arm_test_onboard.py is running on the Pi first.\n")
    print("Joint map:")
    for aux, name in JOINT_NAMES.items():
        rc_ch = aux + 8
        print(f"  {aux}  {name:<6}  AUX{aux} → RC ch {rc_ch} → SERVO{rc_ch}")
    print(f"\nPWM range: {MIN_US}–{MAX_US} µs   center = {CENTER_US}")
    print("Commands:  <joint 1-8> <pwm>  |  center (c)  |  status (s)  |  q\n")

    current = {aux: CENTER_US for aux in range(1, AUX_CHANNELS + 1)}

    try:
        while True:
            try:
                line = input("arm> ").strip().lower()
            except EOFError:
                break

            if not line:
                continue

            if line in ("q", "quit", "exit"):
                break

            if line in ("c", "center"):
                for aux in current:
                    current[aux] = CENTER_US
                send(sock, pi, {"center_all": True})
                print("  → All joints → 1500 µs")
                print_state(current)
                continue

            if line in ("s", "status"):
                print_state(current)
                continue

            parts = line.split()
            if len(parts) == 2:
                try:
                    aux = int(parts[0])
                    us  = int(parts[1])
                except ValueError:
                    print(f"  Bad input — expected: <joint 1-{AUX_CHANNELS}> <pwm {MIN_US}-{MAX_US}>")
                    continue

                if not (1 <= aux <= AUX_CHANNELS):
                    print(f"  Joint must be 1-{AUX_CHANNELS}")
                    continue

                clamped = clamp(us, MIN_US, MAX_US)
                if clamped != us:
                    print(f"  (PWM clamped to {clamped})")
                us = clamped

                current[aux] = us
                send(sock, pi, {"joint": aux, "pwm": us})
                name   = JOINT_NAMES[aux]
                rc_ch  = aux + 8
                print(f"  → AUX{aux} ({name}) = {us} µs  [RC ch {rc_ch}]")
                print_state(current)
            else:
                print(f"  Usage:  <joint 1-{AUX_CHANNELS}> <pwm>   e.g.  1 1600")

    except KeyboardInterrupt:
        pass

    print("\nExiting — centering all joints on Pi.")
    send(sock, pi, {"center_all": True})
    sock.close()


if __name__ == "__main__":
    main()
