#!/usr/bin/env python3
"""
Arm joint test — ONBOARD  (runs on the Raspberry Pi)
=====================================================
Listens for joint/PWM commands from the topside test script over UDP,
then forwards them to the Pix6 via MAVLink RC_CHANNELS_OVERRIDE on
channels 9-16 (AUX outputs 1-8).

DIAGNOSTIC mode: reads back SERVO_OUTPUT_RAW (servo9-servo16) from
the FC every 2 s so you can see exactly where the pipeline is stalling.

Setup in Mission Planner first:
    SERVO9_FUNCTION  = 1  (RCPassThru)  ← AUX1
    SERVO10_FUNCTION = 1  (RCPassThru)  ← AUX2
    SERVO11_FUNCTION = 1  (RCPassThru)  ← AUX3
    SERVO12_FUNCTION = 1  (RCPassThru)  ← AUX4
    SERVO13_FUNCTION = 1  (RCPassThru)  ← AUX5
    SERVO14_FUNCTION = 1  (RCPassThru)  ← AUX6
    SERVO15_FUNCTION = 1  (RCPassThru)  ← AUX7
    SERVO16_FUNCTION = 1  (RCPassThru)  ← AUX8
    BRD_SAFETYENABLE = 0

Start MAVProxy first, then run this script:
    python3 test/arm_test_onboard.py
"""

import json
import socket
import time

from pymavlink import mavutil

# ── Config ────────────────────────────────────────────────────────────────────
LISTEN_PORT   = 5011
MAVLINK_URL   = "udp:127.0.0.1:14551"
CENTER_US     = 1500
MIN_US        = 500
MAX_US        = 2500
OVERRIDE_HZ   = 20
DIAG_INTERVAL = 2.0   # seconds between diagnostic prints

# AUX outputs are RC channels 9-16 in MAVLink RC_CHANNELS_OVERRIDE.
# Update this map to match your actual wiring (AUX port → joint name).
AUX_CHANNELS = 8      # number of AUX ports in use
AUX_RC_OFFSET = 9     # AUX1 = RC channel 9, AUX2 = channel 10, etc.

JOINT_NAMES = {
    1: "J1",    # AUX1 → RC ch 9
    2: "J2",    # AUX2 → RC ch 10
    3: "J3",    # AUX3 → RC ch 11
    4: "J4",    # AUX4 → RC ch 12
    5: "J5",    # AUX5 → RC ch 13
    6: "J6",    # AUX6 → RC ch 14
    7: "Claw",  # AUX7 → RC ch 15
    8: "AUX8",  # AUX8 → RC ch 16 (spare)
}
IGNORE = 65535
# ─────────────────────────────────────────────────────────────────────────────


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def main():
    print(f"[arm-onboard] Connecting to MAVProxy at {MAVLINK_URL} ...")
    master = mavutil.mavlink_connection(MAVLINK_URL)

    print("[arm-onboard] Waiting for heartbeat from Pix6 ...")
    hb = master.wait_heartbeat(timeout=15)
    if hb:
        print(f"[arm-onboard] Heartbeat OK  "
              f"(system={master.target_system}  component={master.target_component})")
    else:
        print("[arm-onboard] *** NO HEARTBEAT in 15 s ***")
        print("[arm-onboard]     Check: MAVProxy running?  Pix6 USB plugged in?")

    # Request SERVO_OUTPUT_RAW at ~2 Hz so we can read AUX output values
    for msg_id, interval_us in [
        (36,  500_000),   # SERVO_OUTPUT_RAW  @ 2 Hz
        (65,  500_000),   # RC_CHANNELS        @ 2 Hz
    ]:
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
    print("[arm-onboard] All AUX channels → CENTER (1500 µs)")
    print("[arm-onboard] ── DIAGNOSTIC OUTPUT every 2 s ────────────────────────────")
    print("[arm-onboard]   RC_CHANNELS ch9-16 = what FC thinks AUX RC inputs are")
    print("[arm-onboard]   SERVO_OUTPUT ch9-16 = what FC is actually sending to AUX pins")
    print("[arm-onboard]   If RC changes but SERVO_OUTPUT doesn't →")
    print("[arm-onboard]     SERVOx_FUNCTION not set to 1 (RCPassThru) in Mission Planner")
    print("[arm-onboard]     or BRD_SAFETYENABLE=1 (safety switch blocking output)")
    print("[arm-onboard]   If RC never changes →")
    print("[arm-onboard]     RC_CHANNELS_OVERRIDE not reaching FC — check MAVProxy")
    print("[arm-onboard] ────────────────────────────────────────────────────────────")
    print("[arm-onboard] Press Ctrl+C to stop.")

    # pwm[1..8] = current target for AUX1..AUX8
    pwm = {aux: CENTER_US for aux in range(1, AUX_CHANNELS + 1)}

    last_send      = 0.0
    last_heartbeat = 0.0
    last_diag      = 0.0

    fc_rc  = {}   # rc channel → pwm from RC_CHANNELS
    fc_srv = {}   # rc channel → pwm from SERVO_OUTPUT_RAW

    def send_override():
        rc = [IGNORE] * 18
        for aux, us in pwm.items():
            rc_ch = aux + AUX_RC_OFFSET - 1          # AUX1→index 8, AUX2→index 9 …
            rc[rc_ch] = int(clamp(us, MIN_US, MAX_US))
        ts = master.target_system or 1
        tc = master.target_component or 1
        master.mav.rc_channels_override_send(ts, tc, *rc)

    def send_heartbeat():
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )

    def poll_mavlink():
        while True:
            msg = master.recv_match(
                type=["RC_CHANNELS", "SERVO_OUTPUT_RAW"],
                blocking=False,
            )
            if msg is None:
                break
            t = msg.get_type()
            if t == "RC_CHANNELS":
                for rc_ch in range(AUX_RC_OFFSET, AUX_RC_OFFSET + AUX_CHANNELS):
                    v = getattr(msg, f"chan{rc_ch}_raw", 0)
                    fc_rc[rc_ch] = v
            elif t == "SERVO_OUTPUT_RAW":
                # servo9..servo16 carry AUX1..AUX8 outputs
                for rc_ch in range(AUX_RC_OFFSET, AUX_RC_OFFSET + AUX_CHANNELS):
                    v = getattr(msg, f"servo{rc_ch}_raw", 0)
                    fc_srv[rc_ch] = v

    def print_diag():
        print()
        if fc_rc:
            rc_str = "  ".join(
                f"AUX{rc_ch - AUX_RC_OFFSET + 1}(ch{rc_ch})={fc_rc.get(rc_ch, '?')}"
                for rc_ch in range(AUX_RC_OFFSET, AUX_RC_OFFSET + AUX_CHANNELS)
            )
            print(f"[DIAG] RC_CHANNELS (FC RC input):     {rc_str}")
        else:
            print("[DIAG] RC_CHANNELS: no data yet")

        if fc_srv:
            srv_str = "  ".join(
                f"AUX{rc_ch - AUX_RC_OFFSET + 1}(ch{rc_ch})={fc_srv.get(rc_ch, '?')}"
                for rc_ch in range(AUX_RC_OFFSET, AUX_RC_OFFSET + AUX_CHANNELS)
            )
            print(f"[DIAG] SERVO_OUTPUT_RAW (FC PWM out): {srv_str}")
        else:
            print("[DIAG] SERVO_OUTPUT_RAW: no data yet")

        sending = "  ".join(
            f"AUX{aux}({JOINT_NAMES[aux]})={pwm[aux]}"
            for aux in sorted(pwm)
        )
        print(f"[DIAG] We are sending:                {sending}")

        non_center = [aux for aux, us in pwm.items() if us != CENTER_US]
        if non_center and fc_rc:
            rc_ch_list = [aux + AUX_RC_OFFSET - 1 for aux in non_center]
            rc_matches = all(
                abs(fc_rc.get(rc_ch, CENTER_US) - pwm[aux]) < 20
                for aux, rc_ch in zip(non_center, rc_ch_list)
            )
            srv_matches = all(
                abs(fc_srv.get(rc_ch, CENTER_US) - pwm[aux]) < 20
                for aux, rc_ch in zip(non_center, rc_ch_list)
            ) if fc_srv else False

            if not rc_matches:
                print("[DIAG] *** RC_CHANNELS not matching sent values —")
                print("[DIAG]     RC_CHANNELS_OVERRIDE not reaching FC. Is MAVProxy running?")
            elif rc_matches and not srv_matches:
                print("[DIAG] *** RC input changed but SERVO output did NOT —")
                print("[DIAG]     Most likely fixes:")
                print("[DIAG]       1. Set SERVO9-16_FUNCTION = 1 (RCPassThru) in Mission Planner")
                print("[DIAG]       2. Set BRD_SAFETYENABLE = 0 in Mission Planner")
            elif rc_matches and srv_matches:
                print("[DIAG] RC input AND servo output both match — FC side is OK.")
                print("[DIAG]     If servo still doesn't move: check wiring to AUX pins.")
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
                        for aux in pwm:
                            pwm[aux] = CENTER_US
                        print("[arm-onboard] ALL CENTER (1500 µs)")
                        continue

                    aux = int(cmd.get("joint", 0))
                    us  = int(cmd.get("pwm", CENTER_US))
                    if 1 <= aux <= AUX_CHANNELS:
                        pwm[aux] = us
                        name = JOINT_NAMES.get(aux, "?")
                        rc_ch = aux + AUX_RC_OFFSET - 1
                        print(f"[arm-onboard] AUX{aux} ({name}) → ch{rc_ch}  {us} µs")
                    else:
                        print(f"[arm-onboard] Ignored AUX {aux} — valid range 1-{AUX_CHANNELS}")

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
        print("\n[arm-onboard] Stopping — centering all AUX channels.")
        for aux in pwm:
            pwm[aux] = CENTER_US
        send_override()
        time.sleep(0.2)
        sock.close()


if __name__ == "__main__":
    main()
