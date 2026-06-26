#!/usr/bin/env python3
"""
Arm joint test — TOPSIDE  (runs on your control laptop)
========================================================
Lets you manually set PWM for individual arm joint outputs by typing
commands.  Sends UDP packets to arm_test_onboard.py running on the Pi.

Usage:
    python test/arm_test_topside.py
    python test/arm_test_topside.py --ip 192.168.69.100

Commands at the prompt:
    1 1600      set J1 to 1600 µs  (AUX5 / M13)
    1 1400      set J1 back to neutral
    3 1500      set J3 to stop     (AUX3 / M11)
    center      all joints → neutral PWM  (also: c)
    status      print current PWM state  (also: s)
    q           center all then exit

Joint map (type joint number 1-4 — matches rov_ui / new_ar.py):
    1  J1         (AUX5 → RC ch 13 → M13)  500–2350 µs, neutral 1400
    2  J2         (AUX1 → RC ch  9 → M9)   950–2200 µs, neutral 1600
    3  J3         (AUX3 → RC ch 11 → M11)  1300–1700 µs, stop 1500 (continuous)
    4  Claw       (AUX7 → RC ch 15 → M15)  1325–1525 µs, stop 1425 (continuous)

Mission Planner parameters required (set once, write params):
    SERVO9_FUNCTION  = 59   (RCPassThru9  → J2 / AUX1 / M9)
    SERVO11_FUNCTION = 61   (RCPassThru11 → J3 / AUX3 / M11)
    SERVO13_FUNCTION = 63   (RCPassThru13 → J1 / AUX5 / M13)
    SERVO15_FUNCTION = 65   (RCPassThru15 → Claw / AUX7 / M15)
    BRD_SAFETYENABLE = 0
"""

import argparse
import json
import socket
import sys

# ── Config ────────────────────────────────────────────────────────────────────
DEFAULT_PI_IP = "192.168.69.100"
TEST_PORT     = 5011          # Must match arm_test_onboard.py
NUM_JOINTS    = 4
# ─────────────────────────────────────────────────────────────────────────────

JOINT_NAMES = {
    1: "J1",
    2: "J2",
    3: "J3",
    4: "Claw",
}

JOINT_TO_AUX = {1: 5, 2: 1, 3: 3, 4: 7}

JOINT_LIMITS = {
    1: (500, 2350, 1400),
    2: (950, 2200, 1600),
    3: (1300, 1700, 1500),
    4: (1325, 1525, 1425),
}


def joint_center_us(joint: int) -> int:
    return JOINT_LIMITS[joint][2]


def joint_pwm_range(joint: int) -> tuple[int, int]:
    lo, hi, _ = JOINT_LIMITS[joint]
    return lo, hi


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def send(sock, addr, payload):
    sock.sendto(json.dumps(payload).encode(), addr)


def print_state(current):
    cols = "  ".join(
        f"{JOINT_NAMES[j]}(AUX{JOINT_TO_AUX[j]})={current[j]}"
        for j in sorted(current)
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
    for joint, name in JOINT_NAMES.items():
        aux = JOINT_TO_AUX[joint]
        rc_ch = aux + 8
        lo, hi = joint_pwm_range(joint)
        ctr = joint_center_us(joint)
        print(f"  {joint}  {name:<6}  AUX{aux} → RC ch {rc_ch} → SERVO{rc_ch}  ({lo}–{hi}, neutral {ctr})")
    print("\nCommands:  <joint 1-4> <pwm>  |  center (c)  |  status (s)  |  q\n")

    current = {j: joint_center_us(j) for j in range(1, NUM_JOINTS + 1)}

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
                for joint in current:
                    current[joint] = joint_center_us(joint)
                send(sock, pi, {"center_all": True})
                print("  → All joints centered")
                print_state(current)
                continue

            if line in ("s", "status"):
                print_state(current)
                continue

            parts = line.split()
            if len(parts) == 2:
                try:
                    joint = int(parts[0])
                    us    = int(parts[1])
                except ValueError:
                    print(f"  Bad input — expected: <joint 1-{NUM_JOINTS}> <pwm>")
                    continue

                if not (1 <= joint <= NUM_JOINTS):
                    print(f"  Joint must be 1-{NUM_JOINTS}")
                    continue

                lo, hi = joint_pwm_range(joint)
                clamped = clamp(us, lo, hi)
                if clamped != us:
                    print(f"  (PWM clamped to {clamped})")
                us = clamped

                current[joint] = us
                send(sock, pi, {"joint": joint, "pwm": us})
                name  = JOINT_NAMES[joint]
                aux   = JOINT_TO_AUX[joint]
                rc_ch = aux + 8
                print(f"  → {name} (AUX{aux}) = {us} µs  [RC ch {rc_ch}]")
                print_state(current)
            else:
                print(f"  Usage:  <joint 1-{NUM_JOINTS}> <pwm>   e.g.  1 1600")

    except KeyboardInterrupt:
        pass

    print("\nExiting — centering all joints on Pi.")
    send(sock, pi, {"center_all": True})
    sock.close()


if __name__ == "__main__":
    main()
