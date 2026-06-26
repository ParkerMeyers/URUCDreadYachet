#!/usr/bin/env python3
"""
MOSFET UDP service — ONBOARD (Raspberry Pi)
============================================
Dedicated process for GPIO17 servo power rail control (same as
test/mosfet_test_onboard.py). Runs independently of new_ar.py so MOSFET
works even while the arm stack is starting or retrying BNO055 I2C.

    python3 onboard/mosfet_service.py
    python3 onboard/mosfet_service.py --port 5007 --gpio 17
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
from pathlib import Path

# Allow import when run as script from repo root or onboard/
sys.path.insert(0, str(Path(__file__).resolve().parent))
from mosfet_gpio import (
    MOSFET_GPIO,
    handle_mosfet_cmd,
    init_mosfet_gpio,
    mosfet_is_on,
    release_mosfet_gpio,
)

DEFAULT_PORT = 5007
ARM_CONTROL_PORT = 5006

# Legacy: topside used to send manual AUX / preset JSON to the MOSFET port (5007).
_ARM_FORWARD_CMDS = frozenset({
    "manual_pwm", "preset_motion", "preset_step", "claw_hold",
    "arm_imu_cal", "arm_claw_stop", "arm_imu_zero", "arm_enable", "arm_telemetry",
})


def _maybe_forward_arm_cmd(data: bytes, cmd: dict) -> bool:
    """Forward non-MOSFET JSON to new_ar.py (UDP 5009)."""
    if cmd.get("cmd") not in _ARM_FORWARD_CMDS:
        return False
    try:
        fwd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            fwd.sendto(data, ("127.0.0.1", ARM_CONTROL_PORT))
        finally:
            fwd.close()
        print(
            f"[mosfet-svc] Forwarded {cmd.get('cmd')} → arm control UDP {ARM_CONTROL_PORT}",
            flush=True,
        )
        return True
    except OSError as e:
        print(f"[mosfet-svc] Arm forward failed: {e}", flush=True)
        return False


def main() -> None:
    parser = argparse.ArgumentParser(description="ROV MOSFET UDP service — onboard")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help=f"UDP listen port (default {DEFAULT_PORT})")
    parser.add_argument("--gpio", type=int, default=MOSFET_GPIO,
                        help=f"MOSFET GPIO pin (default {MOSFET_GPIO})")
    args = parser.parse_args()

    if not init_mosfet_gpio(args.gpio):
        print("[mosfet-svc] FATAL: GPIO init failed", flush=True)
        sys.exit(1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))

    print(f"[mosfet-svc] Listening on UDP {args.port} (GPIO{args.gpio})", flush=True)
    print('[mosfet-svc] JSON: {"cmd": "mosfet", "state": true|false}', flush=True)

    try:
        while True:
            data, addr = sock.recvfrom(512)
            try:
                cmd = json.loads(data.decode())
            except Exception:
                print(f"[mosfet-svc] Bad JSON from {addr[0]}:{addr[1]}", flush=True)
                continue
            if handle_mosfet_cmd(cmd, args.gpio):
                print(
                    f"[mosfet-svc] Command from {addr[0]}:{addr[1]} "
                    f"→ {'ON' if mosfet_is_on() else 'OFF'}",
                    flush=True,
                )
            elif _maybe_forward_arm_cmd(data, cmd):
                pass
            else:
                print(f"[mosfet-svc] Ignored: {cmd}", flush=True)
    except KeyboardInterrupt:
        print("\n[mosfet-svc] Stopping.", flush=True)
    finally:
        sock.close()
        release_mosfet_gpio(args.gpio)
        print("[mosfet-svc] GPIO released — MOSFET OFF", flush=True)


if __name__ == "__main__":
    main()
