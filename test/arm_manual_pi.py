#!/usr/bin/env python3
"""
Manual arm control on the Pi — no web UI required.

Talks to new_ar.py (UDP 5009) on localhost.

Prerequisites:
  - MAVProxy running (tcpin 5762 + 5763)
  - python3 onboard/supervisor.py start arm

Usage:
    python3 test/arm_manual_pi.py              # interactive
    python3 test/arm_manual_pi.py --on       # arm unlock + manual ON
    python3 test/arm_manual_pi.py --j1 1600  # one-shot J1 move
    python3 test/arm_manual_pi.py --claw 1425

Interactive commands (joint numbers 1–4):
    1 1600   J1 (AUX4)     2 1600   J2 (AUX1)
    3 1600   J3 (AUX3)     4 1525   Claw (AUX7, 1325–1525)
    on       arm unlock + manual ON
    off      manual OFF + arm lock
    center   all joints neutral, claw stop
    q        quit (arm lock)
"""
from __future__ import annotations

import argparse
import json
import socket
import sys

ARM_PORT = 5009
CLAW_STOP = 1425
CLAW_MIN = 1325
CLAW_MAX = 1525
JOINT_CENTER = 1500

JOINT_NAMES = {1: "J1", 2: "J2", 3: "J3", 4: "Claw"}
JOINT_TO_AUX = {1: 4, 2: 1, 3: 3, 4: 7}


def send(port: int, pkt: dict) -> None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.sendto(json.dumps(pkt).encode("utf-8"), ("127.0.0.1", port))
    finally:
        s.close()
    print(f"  → {pkt}")


def arm_enable(on: bool) -> None:
    send(ARM_PORT, {"cmd": "arm_enable", "enabled": on})


def manual_enable(on: bool) -> None:
    send(ARM_PORT, {"cmd": "manual_pwm", "enabled": on})


def move_joint(joint: int, pwm: int) -> None:
    aux = JOINT_TO_AUX[joint]
    if joint == 4:
        pwm = max(CLAW_MIN, min(CLAW_MAX, pwm))
    send(ARM_PORT, {"cmd": "manual_pwm", "enabled": True, "aux": aux, "pwm": pwm})


def center_all() -> None:
    send(ARM_PORT, {"cmd": "manual_pwm", "center": True, "enabled": True})


def boot_sequence() -> None:
    print("Arm unlock, manual mode ON")
    arm_enable(True)
    manual_enable(True)


def shutdown() -> None:
    print("Manual OFF, arm lock")
    manual_enable(False)
    arm_enable(False)


def run_interactive() -> None:
    print("Manual arm — joints 1=J1 2=J2 3=J3 4=Claw | on | off | center | q")
    print(f"Claw range {CLAW_MIN}–{CLAW_MAX} µs (stop {CLAW_STOP})")
    print("Log: tail -f /tmp/rov_arm.log\n")
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
            if line == "on":
                boot_sequence()
                continue
            if line == "off":
                shutdown()
                continue
            if line in ("c", "center"):
                center_all()
                continue
            parts = line.split()
            if len(parts) != 2:
                print("  Usage: <1-4> <pwm>  |  on  |  off  |  center  |  q")
                continue
            try:
                joint = int(parts[0])
                pwm = int(parts[1])
            except ValueError:
                print("  Bad joint or PWM")
                continue
            if joint not in JOINT_TO_AUX:
                print("  Joint must be 1–4")
                continue
            boot_sequence()
            move_joint(joint, pwm)
            print(f"  {JOINT_NAMES[joint]} (AUX{JOINT_TO_AUX[joint]}) → {pwm} µs")
    finally:
        shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="Manual arm control on Pi (no UI)")
    parser.add_argument("--on", action="store_true", help="Arm unlock + manual ON, then exit")
    parser.add_argument("--off", action="store_true", help="Manual OFF + arm lock")
    parser.add_argument("--center", action="store_true", help="Center all joints")
    for j in (1, 2, 3):
        parser.add_argument(f"--j{j}", type=int, metavar="PWM", help=f"Move J{j}")
    parser.add_argument("--claw", type=int, metavar="PWM", help="Move claw (1325–1525)")
    args = parser.parse_args()

    if args.off:
        shutdown()
        return 0

    one_shot = args.on or args.center or any(
        getattr(args, f"j{j}") is not None for j in (1, 2, 3)
    ) or args.claw is not None

    if one_shot:
        boot_sequence()
        if args.center:
            center_all()
        for j in (1, 2, 3):
            pwm = getattr(args, f"j{j}")
            if pwm is not None:
                move_joint(j, pwm)
        if args.claw is not None:
            move_joint(4, args.claw)
        if not args.on:
            print("Done — check: tail -f /tmp/rov_arm.log")
        return 0

    run_interactive()
    return 0


if __name__ == "__main__":
    sys.exit(main())
