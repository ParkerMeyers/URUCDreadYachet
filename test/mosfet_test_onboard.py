#!/usr/bin/env python3
"""
MOSFET test — ONBOARD  (runs on the Raspberry Pi)
==================================================
Listens for on/off commands from the topside test script over UDP,
then drives GPIO17 (servo power rail MOSFET) via lgpio.

Run this instead of new_ar.py when you only want to exercise the
MOSFET switch without the full arm stack.

    python3 test/mosfet_test_onboard.py
    python3 test/mosfet_test_onboard.py --gpio 17 --port 5012
"""

import argparse
import json
import socket
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "onboard"))
from mosfet_gpio import (
    MOSFET_GPIO,
    handle_mosfet_cmd,
    init_mosfet_gpio,
    release_mosfet_gpio,
)

LISTEN_PORT = 5012


def main():
    parser = argparse.ArgumentParser(description="ROV MOSFET test — onboard")
    parser.add_argument("--port", type=int, default=LISTEN_PORT,
                        help=f"UDP listen port (default {LISTEN_PORT})")
    parser.add_argument("--gpio", type=int, default=MOSFET_GPIO,
                        help=f"MOSFET GPIO pin (default {MOSFET_GPIO})")
    args = parser.parse_args()

    if not init_mosfet_gpio(args.gpio):
        sys.exit(1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))

    print(f"[mosfet-onboard] Listening on UDP port {args.port}")
    print("[mosfet-onboard] JSON: {\"cmd\": \"mosfet\", \"state\": true|false}")
    print("[mosfet-onboard] Press Ctrl+C to stop (MOSFET turns OFF on exit).")

    try:
        while True:
            data, addr = sock.recvfrom(512)
            try:
                cmd = json.loads(data.decode())
            except Exception:
                print(f"[mosfet-onboard] Bad JSON from {addr[0]}:{addr[1]}", flush=True)
                continue
            if not handle_mosfet_cmd(cmd, args.gpio):
                print(f"[mosfet-onboard] Ignored command: {cmd}", flush=True)
    except KeyboardInterrupt:
        print("\n[mosfet-onboard] Stopping.", flush=True)
    finally:
        sock.close()
        release_mosfet_gpio(args.gpio)
        print("[mosfet-onboard] GPIO released — MOSFET OFF", flush=True)


if __name__ == "__main__":
    main()
