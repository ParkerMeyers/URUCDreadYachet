#!/usr/bin/env python3
"""
Motor test — ONBOARD  (runs on the Raspberry Pi)
=================================================
Listens for motor/PWM commands from the topside test script over UDP,
then forwards them to the Pix6 via MAVLink RC_CHANNELS_OVERRIDE through
MAVProxy.

DIAGNOSTIC mode: also reads back RC_CHANNELS and SERVO_OUTPUT_RAW from
the FC every 2 s so you can see exactly where the pipeline is stalling.

Start MAVProxy first, then run this script:
    python3 test/motor_test_onboard.py
"""

import json
import socket
import time

from pymavlink import mavutil

# ── Config ────────────────────────────────────────────────────────────────────
LISTEN_PORT   = 5010
MAVLINK_URL   = "udp:127.0.0.1:14551"
NEUTRAL_US    = 1500
MIN_US        = 1100
MAX_US        = 1900
OVERRIDE_HZ   = 20
NUM_CHANNELS  = 8
IGNORE        = 65535
DIAG_INTERVAL = 2.0   # seconds between diagnostic prints
# ─────────────────────────────────────────────────────────────────────────────

MOTOR_NAMES = {
    1: "front_left_h",
    2: "back_left_h",
    3: "front_left_v",
    4: "front_right_v",
    5: "front_right_h",
    6: "back_right_h",
    7: "back_right_v",
    8: "back_left_v",
}


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def fmt_rc(vals):
    return "  ".join(f"ch{i+1}={v}" for i, v in enumerate(vals) if v != IGNORE)


def main():
    print(f"[onboard] Connecting to MAVProxy at {MAVLINK_URL} ...")
    master = mavutil.mavlink_connection(MAVLINK_URL)

    print("[onboard] Waiting for heartbeat from Pix6 ...")
    hb = master.wait_heartbeat(timeout=15)
    if hb:
        print(f"[onboard] Heartbeat OK  "
              f"(system={master.target_system}  component={master.target_component})")
    else:
        print("[onboard] *** NO HEARTBEAT in 15 s ***")
        print("[onboard]     Check: MAVProxy running?  Pix6 USB plugged in?")

    # Request SERVO_OUTPUT_RAW and RC_CHANNELS at ~2 Hz so we can read them back
    for msg_id, interval_us in [
        (36,  500_000),   # SERVO_OUTPUT_RAW  @ 2 Hz
        (65,  500_000),   # RC_CHANNELS        @ 2 Hz
        (193, 500_000),   # HEARTBEAT          @ 2 Hz (for arm state)
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

    print(f"[onboard] Listening for test commands on UDP port {LISTEN_PORT}")
    print("[onboard] All channels → NEUTRAL (1500 µs)")
    print("[onboard] ── DIAGNOSTIC OUTPUT every 2 s ──────────────────────────────")
    print("[onboard]   RC_CHANNELS  = what the FC thinks the RC inputs are")
    print("[onboard]   SERVO_OUTPUT = what the FC is actually sending to PWM pins")
    print("[onboard]   If RC_CHANNELS changes but SERVO_OUTPUT doesn't →")
    print("[onboard]     BRD_SAFETYENABLE=1 (safety switch blocking output)")
    print("[onboard]     or SERVO1-8_FUNCTION not set to RCPassThru (51-58)")
    print("[onboard]   If RC_CHANNELS never changes → RC_CHANNELS_OVERRIDE not")
    print("[onboard]     reaching FC — check MAVProxy is running")
    print("[onboard] ────────────────────────────────────────────────────────────")
    print("[onboard] Press Ctrl+C to stop.")

    pwm            = {ch: NEUTRAL_US for ch in range(1, NUM_CHANNELS + 1)}
    last_send      = 0.0
    last_heartbeat = 0.0
    last_diag      = 0.0

    # Cached FC feedback
    fc_rc   = {}   # channel → pwm from RC_CHANNELS
    fc_srv  = {}   # channel → pwm from SERVO_OUTPUT_RAW
    fc_armed = None

    def send_override():
        rc = [IGNORE] * 18
        for ch, us in pwm.items():
            rc[ch - 1] = int(clamp(us, MIN_US, MAX_US))
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
        """Drain MAVLink messages; update fc_rc, fc_srv, fc_armed."""
        nonlocal fc_armed
        while True:
            msg = master.recv_match(
                type=["RC_CHANNELS", "SERVO_OUTPUT_RAW", "HEARTBEAT"],
                blocking=False,
            )
            if msg is None:
                break
            t = msg.get_type()
            if t == "RC_CHANNELS":
                for i in range(1, 9):
                    v = getattr(msg, f"chan{i}_raw", 0)
                    fc_rc[i] = v
            elif t == "SERVO_OUTPUT_RAW":
                for i in range(1, 9):
                    v = getattr(msg, f"servo{i}_raw", 0)
                    fc_srv[i] = v
            elif t == "HEARTBEAT":
                # MAV_MODE_FLAG_SAFETY_ARMED = 128
                fc_armed = bool(msg.base_mode & 128)

    def print_diag():
        armed_str = ("ARMED" if fc_armed else "DISARMED") if fc_armed is not None else "unknown"
        print(f"\n[DIAG] FC armed: {armed_str}")

        if fc_rc:
            rc_str = "  ".join(f"ch{i}={fc_rc.get(i,'?')}" for i in range(1, 9))
            print(f"[DIAG] RC_CHANNELS (FC RC input):  {rc_str}")
        else:
            print("[DIAG] RC_CHANNELS: no data yet (FC not sending it back)")

        if fc_srv:
            srv_str = "  ".join(f"ch{i}={fc_srv.get(i,'?')}" for i in range(1, 9))
            print(f"[DIAG] SERVO_OUTPUT_RAW (FC PWM out): {srv_str}")
        else:
            print("[DIAG] SERVO_OUTPUT_RAW: no data yet")

        # Show what we are currently sending
        sending = "  ".join(f"ch{ch}={pwm[ch]}" for ch in sorted(pwm))
        print(f"[DIAG] We are sending:                {sending}")

        # Give a quick diagnosis hint
        non_neutral = [ch for ch, us in pwm.items() if us != NEUTRAL_US]
        if non_neutral and fc_rc:
            rc_matches = all(
                abs(fc_rc.get(ch, NEUTRAL_US) - pwm[ch]) < 20
                for ch in non_neutral
            )
            srv_matches = all(
                abs(fc_srv.get(ch, NEUTRAL_US) - pwm[ch]) < 20
                for ch in non_neutral
            ) if fc_srv else False

            if not rc_matches:
                print("[DIAG] *** RC_CHANNELS not matching sent values —")
                print("[DIAG]     RC_CHANNELS_OVERRIDE may not be reaching the FC.")
                print("[DIAG]     Is MAVProxy running?  Check: pgrep mavproxy")
            elif rc_matches and not srv_matches:
                print("[DIAG] *** RC input changed but SERVO output did NOT —")
                print("[DIAG]     Most likely fixes:")
                print("[DIAG]       1. Set SERVO1-8_FUNCTION = 51-58 (RCPassThru) in QGC")
                print("[DIAG]       2. Set BRD_SAFETYENABLE = 0 in QGC (no safety switch)")
                print("[DIAG]       3. If using Motor1-8 functions: arm the FC first")
            elif rc_matches and srv_matches:
                print("[DIAG] RC input AND servo output both match — FC side is OK.")
                print("[DIAG]     If motors still don't move: check ESC wiring to Pix6 outputs.")
        print()

    send_override()

    try:
        while True:
            now = time.time()

            poll_mavlink()

            # Drain incoming UDP from topside
            try:
                while True:
                    data, _ = sock.recvfrom(4096)
                    try:
                        cmd = json.loads(data.decode())
                    except Exception:
                        continue

                    if cmd.get("neutral_all"):
                        for ch in pwm:
                            pwm[ch] = NEUTRAL_US
                        print("[onboard] ALL NEUTRAL")
                        continue

                    ch = int(cmd.get("motor", 0))
                    us = int(cmd.get("pwm", NEUTRAL_US))
                    if 1 <= ch <= NUM_CHANNELS:
                        pwm[ch] = us
                        name = MOTOR_NAMES.get(ch, "?")
                        print(f"[onboard] Motor {ch} ({name}) → {us} µs")
                    else:
                        print(f"[onboard] Ignored motor {ch} — valid range 1-{NUM_CHANNELS}")

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
        print("\n[onboard] Stopping — sending neutral to all channels.")
        for ch in pwm:
            pwm[ch] = NEUTRAL_US
        send_override()
        time.sleep(0.2)
        sock.close()


if __name__ == "__main__":
    main()
