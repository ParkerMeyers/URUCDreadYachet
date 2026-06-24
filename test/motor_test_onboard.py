#!/usr/bin/env python3
"""
Motor test — ONBOARD  (runs on the Raspberry Pi)
=================================================
Listens for motor/PWM commands from the topside test script over UDP,
then forwards them to the Pix6 via MAVLink RC_CHANNELS_OVERRIDE through
MAVProxy.  All 8 channels stay at 1500 µs until you change them.

Start MAVProxy first, then run this script:
    python3 test/motor_test_onboard.py
"""

import json
import socket
import time

from pymavlink import mavutil

# ── Config ────────────────────────────────────────────────────────────────────
LISTEN_PORT  = 5010                  # UDP port — must match motor_test_topside.py
MAVLINK_URL  = "udp:127.0.0.1:14551" # MAVProxy output (same as stabilization.py)
NEUTRAL_US   = 1500
MIN_US       = 1100
MAX_US       = 1900
OVERRIDE_HZ  = 20                    # How often to re-send RC_CHANNELS_OVERRIDE
NUM_CHANNELS = 8                     # Pix6 outputs 1-8
IGNORE       = 65535                 # MAVLink: leave this RC channel unchanged
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


def main():
    print(f"[onboard] Connecting to MAVProxy at {MAVLINK_URL} ...")
    master = mavutil.mavlink_connection(MAVLINK_URL)

    print("[onboard] Waiting for heartbeat from Pix6 ...")
    hb = master.wait_heartbeat(timeout=15)
    if hb:
        print(f"[onboard] Heartbeat OK  (system={master.target_system})")
    else:
        print("[onboard] WARNING: no heartbeat in 15 s — continuing anyway.")
        print("[onboard] Check that MAVProxy is running and Pix6 is connected.")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", LISTEN_PORT))
    sock.setblocking(False)
    print(f"[onboard] Listening for test commands on UDP port {LISTEN_PORT}")
    print("[onboard] All channels → NEUTRAL (1500 µs)  — waiting for commands.")
    print("[onboard] Press Ctrl+C to stop.")

    pwm = {ch: NEUTRAL_US for ch in range(1, NUM_CHANNELS + 1)}
    last_send      = 0.0
    last_heartbeat = 0.0

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

    send_override()

    try:
        while True:
            now = time.time()

            # Drain incoming UDP commands from topside
            try:
                while True:
                    data, addr = sock.recvfrom(4096)
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
                        print(f"[onboard] Ignored unknown motor {ch} "
                              f"(valid range 1-{NUM_CHANNELS})")

            except BlockingIOError:
                pass

            # Heartbeat every second
            if now - last_heartbeat >= 1.0:
                last_heartbeat = now
                send_heartbeat()

            # Re-send RC override at OVERRIDE_HZ
            if now - last_send >= 1.0 / OVERRIDE_HZ:
                last_send = now
                send_override()

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
