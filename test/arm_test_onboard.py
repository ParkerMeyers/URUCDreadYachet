#!/usr/bin/env python3
"""
Arm joint test — ONBOARD  (runs on the Raspberry Pi)
=====================================================
Listens for joint/PWM commands from the topside test script over UDP,
then forwards them to the Pix6 via MAVLink RC_CHANNELS_OVERRIDE.

Uses the same arm_joints + MAVLink path as onboard/new_ar.py.

Start MAVProxy first (arm test uses tcp:127.0.0.1:5763), then run:
    python3 test/arm_test_onboard.py
"""
from __future__ import annotations

import json
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "onboard"))
from arm_joints import (
    JOINT_NAMES,
    JOINT_TO_AUX,
    JOINT_TO_MOTOR,
    NUM_JOINTS,
    RC_IGNORE,
    build_rc_override,
    default_joint_pwm,
    joint_center_us,
    joint_to_rc_ch,
)
from mavlink_rc import MAVLINK_ONBOARD_ARM, connect_mavlink, send_rc_channels_override

from pymavlink import mavutil

LISTEN_PORT = 5011
MAVLINK_URL = MAVLINK_ONBOARD_ARM
OVERRIDE_HZ = 20
DIAG_INTERVAL = 2.0


def main():
    print(f"[arm-onboard] Connecting to MAVProxy at {MAVLINK_URL} ...")
    master = connect_mavlink(MAVLINK_URL)

    print("[arm-onboard] Waiting for heartbeat from Pix6 ...")
    hb = master.wait_heartbeat(timeout=15)
    if hb:
        print(f"[arm-onboard] Heartbeat OK  "
              f"(system={master.target_system}  component={master.target_component})")
    else:
        print("[arm-onboard] *** NO HEARTBEAT in 15 s ***")
        print("[arm-onboard]     Check: MAVProxy running?  Pix6 USB plugged in?")

    for msg_id, interval_us in ((36, 500_000), (65, 500_000)):
        try:
            master.mav.command_long_send(
                master.target_system or 1,
                master.target_component or 1,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, interval_us, 0, 0, 0, 0, 0,
            )
        except Exception:
            pass

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", LISTEN_PORT))
    sock.setblocking(False)

    print(f"[arm-onboard] Listening for commands on UDP port {LISTEN_PORT}")
    print("[arm-onboard] Joint numbers 1-4 (J1, J2, J3, Claw)")
    print("[arm-onboard] Press Ctrl+C to stop.")

    pwm = default_joint_pwm()
    last_send = 0.0
    last_heartbeat = 0.0
    last_diag = 0.0
    fc_rc: dict[int, int] = {}
    fc_srv: dict[int, int] = {}

    def send_override():
        send_rc_channels_override(master, build_rc_override(pwm), ignore=RC_IGNORE)

    def send_heartbeat():
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )

    def poll_mavlink():
        while True:
            msg = master.recv_match(type=["RC_CHANNELS", "SERVO_OUTPUT_RAW"], blocking=False)
            if msg is None:
                break
            if msg.get_type() == "RC_CHANNELS":
                for joint in range(1, NUM_JOINTS + 1):
                    rc_ch = joint_to_rc_ch(joint)
                    fc_rc[rc_ch] = getattr(msg, f"chan{rc_ch}_raw", 0)
            else:
                for joint in range(1, NUM_JOINTS + 1):
                    rc_ch = joint_to_rc_ch(joint)
                    fc_srv[rc_ch] = getattr(msg, f"servo{rc_ch}_raw", 0)

    def print_diag():
        print()
        if fc_rc:
            rc_str = "  ".join(
                f"{JOINT_NAMES[j]}(AUX{JOINT_TO_AUX[j]},ch{joint_to_rc_ch(j)})="
                f"{fc_rc.get(joint_to_rc_ch(j), '?')}"
                for j in range(1, NUM_JOINTS + 1)
            )
            print(f"[DIAG] RC_CHANNELS (FC RC input):     {rc_str}")
        else:
            print("[DIAG] RC_CHANNELS: no data yet")

        if fc_srv:
            srv_str = "  ".join(
                f"{JOINT_NAMES[j]}(AUX{JOINT_TO_AUX[j]},ch{joint_to_rc_ch(j)})="
                f"{fc_srv.get(joint_to_rc_ch(j), '?')}"
                for j in range(1, NUM_JOINTS + 1)
            )
            print(f"[DIAG] SERVO_OUTPUT_RAW (FC PWM out): {srv_str}")
        else:
            print("[DIAG] SERVO_OUTPUT_RAW: no data yet")

        sending = "  ".join(
            f"{JOINT_NAMES[j]}(AUX{JOINT_TO_AUX[j]})={pwm[j]}"
            for j in sorted(pwm)
        )
        print(f"[DIAG] We are sending:                {sending}")
        print()

    send_override()

    try:
        while True:
            now = time.time()
            poll_mavlink()

            try:
                while True:
                    data, _ = sock.recvfrom(4096)
                    try:
                        cmd = json.loads(data.decode())
                    except Exception:
                        continue

                    if cmd.get("center_all"):
                        pwm = default_joint_pwm()
                        print("[arm-onboard] ALL CENTER")
                        continue

                    joint = int(cmd.get("joint", 0))
                    us = int(cmd.get("pwm", 1500))
                    if 1 <= joint <= NUM_JOINTS:
                        pwm[joint] = us
                        name = JOINT_NAMES[joint]
                        motor = JOINT_TO_MOTOR[joint]
                        aux = JOINT_TO_AUX[joint]
                        rc_ch = joint_to_rc_ch(joint)
                        print(f"[arm-onboard] {name}/M{motor} (AUX{aux}) → ch{rc_ch}  {us} µs")
                    else:
                        print(f"[arm-onboard] Ignored joint {joint} — valid range 1-{NUM_JOINTS}")

            except BlockingIOError:
                pass

            if now - last_heartbeat >= 1.0:
                last_heartbeat = now
                send_heartbeat()

            if now - last_send >= 1.0 / OVERRIDE_HZ:
                last_send = now
                send_override()

            if now - last_diag >= DIAG_INTERVAL:
                last_diag = now
                print_diag()

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[arm-onboard] Stopping — centering all joints.")
        pwm = default_joint_pwm()
        send_override()
        time.sleep(0.2)
        sock.close()


if __name__ == "__main__":
    main()
