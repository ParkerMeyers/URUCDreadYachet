#!/usr/bin/env python3
"""
Boot-time arm neutral hold — runs on the Pi before new_ar.py starts.

Continuous-rotation J6 spins if the Pix6 outputs anything other than 1500 µs
on AUX3 while the Pi is booting (USB link up, no companion override yet).
This script connects directly to the Pix6 serial port and streams
RC_CHANNELS_OVERRIDE with all arm AUX channels at neutral until new_ar.py
binds UDP :5006 and takes over.

Install on the Pi (once):
    sudo cp onboard/rov-arm-boot.service /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable rov-arm-boot.service

The ROV UI stops this automatically before launching MAVProxy.
"""
from __future__ import annotations

import argparse
import os
import socket
import sys
import time

from pymavlink import mavutil

from mavlink_rc import RC_IGNORE, send_rc_channels_override, wait_for_heartbeat

MOSFET_GPIO = 17
ARM_UDP_PORT = 5006
OVERRIDE_HZ = 20

# AUX1=ch9 … AUX8=ch16 — same mapping as new_ar.py
J6_RC_CH = 11
CLAW_RC_CH = 15
CENTER_US = 1500
CLAW_CENTER_US = 1515
ARM_RC_CHS = (9, 10, 11, 12, 13, 14, 15)


def _neutral_rc() -> list[int]:
    rc = [RC_IGNORE] * 18
    for ch in ARM_RC_CHS:
        if ch == J6_RC_CH:
            rc[ch - 1] = CENTER_US
        elif ch == CLAW_RC_CH:
            rc[ch - 1] = CLAW_CENTER_US
        else:
            rc[ch - 1] = CENTER_US
    rc[15] = CENTER_US  # AUX8 spare
    return rc


def _set_mosfet_on() -> None:
    try:
        import lgpio

        h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(h, MOSFET_GPIO, 1)
        print(f"[boot-guard] MOSFET ON (GPIO{MOSFET_GPIO})", flush=True)
        # Leave chip open for process lifetime — GPIO stays driven.
        return
    except Exception as exc:
        print(f"[boot-guard] MOSFET GPIO unavailable ({exc})", flush=True)


def _arm_controller_active() -> bool:
    """True when new_ar.py is listening on the arm UDP port."""
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(("127.0.0.1", ARM_UDP_PORT))
        probe.close()
        return False
    except OSError:
        return True


def _connect_serial(device: str, baud: int, timeout: float):
    url = device if device.startswith("/dev/") else f"{device}:{baud}"
    print(f"[boot-guard] Connecting to Pix6 at {url} ...", flush=True)
    deadline = time.time() + timeout
    last_err: Exception | None = None
    while time.time() < deadline:
        try:
            master = mavutil.mavlink_connection(url, baud=baud, source_system=255)
            try:
                master.mav.set_protocol(mavutil.mavlink.MAVLINK_V2)
            except Exception:
                pass
            if wait_for_heartbeat(master, timeout=6.0):
                print("[boot-guard] Pix6 heartbeat OK — holding AUX neutral", flush=True)
            else:
                print("[boot-guard] No heartbeat yet — sending neutral anyway", flush=True)
            return master
        except Exception as exc:
            last_err = exc
            time.sleep(1.0)
    raise OSError(f"Could not open {url}: {last_err}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Hold arm AUX at neutral until new_ar.py starts")
    parser.add_argument(
        "--serial",
        default=os.environ.get("ROV_MAV_SERIAL", "/dev/ttyACM0"),
        help="Pix6 serial device (default: /dev/ttyACM0)",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--connect-timeout", type=float, default=90.0)
    args = parser.parse_args()

    _set_mosfet_on()
    neutral = _neutral_rc()
    master = None
    release_after = 0
    interval = 1.0 / OVERRIDE_HZ

    print("[boot-guard] Waiting for Pix6 USB ...", flush=True)
    while master is None:
        if _arm_controller_active():
            print("[boot-guard] new_ar.py already active — exiting", flush=True)
            return 0
        try:
            master = _connect_serial(args.serial, args.baud, timeout=15.0)
        except OSError as exc:
            print(f"[boot-guard] {exc} — retrying", flush=True)
            time.sleep(2.0)

    try:
        while True:
            loop_start = time.time()
            if _arm_controller_active():
                release_after += 1
                if release_after >= 6:
                    print("[boot-guard] new_ar.py took over — releasing serial", flush=True)
                    break
            else:
                release_after = 0

            try:
                send_rc_channels_override(master, neutral, ignore=RC_IGNORE)
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
            except OSError as exc:
                print(f"[boot-guard] MAVLink send failed ({exc}) — reconnecting", flush=True)
                try:
                    master.close()
                except Exception:
                    pass
                master = None
                while master is None:
                    time.sleep(1.0)
                    try:
                        master = _connect_serial(args.serial, args.baud, timeout=15.0)
                    except OSError:
                        pass

            elapsed = time.time() - loop_start
            if elapsed < interval:
                time.sleep(interval - elapsed)
    except KeyboardInterrupt:
        print("\n[boot-guard] Stopped", flush=True)
    finally:
        if master is not None:
            try:
                send_rc_channels_override(master, neutral, ignore=RC_IGNORE)
            except Exception:
                pass
            try:
                master.close()
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
