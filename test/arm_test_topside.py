#!/usr/bin/env python3
"""
Arm joint test — TOPSIDE  (runs on your control laptop)
========================================================
Lets you manually set PWM for individual arm joint outputs by typing
commands.  Sends UDP packets to arm_test_onboard.py running on the Pi.

Usage:
    python test/arm_test_topside.py
    python test/arm_test_topside.py --ip 192.168.69.100
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "onboard"))
from arm_joints import (
    JOINT_CONTINUOUS,
    JOINT_NAMES,
    JOINT_TO_AUX,
    JOINT_TO_MOTOR,
    NUM_JOINTS,
    clamp_joint_pwm,
    default_joint_pwm,
    joint_center_us,
    joint_pwm_range,
    joint_to_rc_ch,
)

DEFAULT_PI_IP = "192.168.69.100"
TEST_PORT = 5011


def joint_center_label(joint: int) -> str:
    return "stop" if joint in JOINT_CONTINUOUS else "neutral"


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
    parser.add_argument("--ip", default=DEFAULT_PI_IP, help=f"Pi IP (default {DEFAULT_PI_IP})")
    parser.add_argument("--port", type=int, default=TEST_PORT, help=f"UDP port (default {TEST_PORT})")
    args = parser.parse_args()

    pi = (args.ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"Arm joint test — sending to {args.ip}:{args.port}")
    print("Make sure arm_test_onboard.py is running on the Pi first.\n")
    print("Joint map:")
    for joint, name in JOINT_NAMES.items():
        aux = JOINT_TO_AUX[joint]
        motor = JOINT_TO_MOTOR[joint]
        rc_ch = joint_to_rc_ch(joint)
        lo, hi = joint_pwm_range(joint)
        ctr = joint_center_us(joint)
        cont = " continuous" if joint in JOINT_CONTINUOUS else ""
        label = joint_center_label(joint)
        extra = " (open→close)" if joint == 4 else ""
        print(
            f"  {joint}  {name:<4} M{motor:<2}  AUX{aux} → ch {rc_ch}{cont}  "
            f"({lo}–{hi}, {label} {ctr}){extra}"
        )
    print("\nCommands:  <joint 1-4> <pwm>  |  center (c)  |  status (s)  |  q\n")

    current = default_joint_pwm()

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
                current = default_joint_pwm()
                send(sock, pi, {"center_all": True})
                print("  → All joints centered")
                print_state(current)
                continue
            if line in ("s", "status"):
                print_state(current)
                continue

            parts = line.split()
            if len(parts) != 2:
                print(f"  Usage:  <joint 1-{NUM_JOINTS}> <pwm>   e.g.  1 1600")
                continue

            try:
                joint = int(parts[0])
                us = int(parts[1])
            except ValueError:
                print(f"  Bad input — expected: <joint 1-{NUM_JOINTS}> <pwm>")
                continue

            if not (1 <= joint <= NUM_JOINTS):
                print(f"  Joint must be 1-{NUM_JOINTS}")
                continue

            lo, hi = joint_pwm_range(joint)
            us = clamp_joint_pwm(joint, us)
            if us != int(parts[1]):
                print(f"  (PWM clamped to {us})")

            current[joint] = us
            send(sock, pi, {"joint": joint, "pwm": us})
            name = JOINT_NAMES[joint]
            aux = JOINT_TO_AUX[joint]
            rc_ch = joint_to_rc_ch(joint)
            print(f"  → {name} (AUX{aux}) = {us} µs  [RC ch {rc_ch}]")
            print_state(current)

    except KeyboardInterrupt:
        pass

    print("\nExiting — centering all joints on Pi.")
    send(sock, pi, {"center_all": True})
    sock.close()


if __name__ == "__main__":
    main()
