#!/usr/bin/env python3
"""Forward arm-controller serial (motor-native µs + sensors) to the Pi over UDP."""

import socket
import serial
import time
import argparse
from pathlib import Path
from serial.tools import list_ports

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from onboard.arm_joints import (
    ARM_CONTROLLER_FIELDS,
    format_arm_controller_csv,
    looks_like_arm_controller_line,
    parse_arm_controller_csv,
)

# ── Argument parsing (allows web UI to pass config at launch) ────────────────
_parser = argparse.ArgumentParser(description="ROV Arm Sender")
_parser.add_argument("--ip",   type=str, default="192.168.69.100",
                     help="Pi IP address (default 192.168.69.100)")
_parser.add_argument("--port", type=str, default="auto",
                     help="Serial port, or 'auto' to scan (Windows: COM*, Linux: ttyACM*)")
_parser.add_argument("--udp-port", type=int, default=5006,
                     help="UDP destination port on Pi (default 5006)")
_parser.add_argument("--scan-timeout", type=float, default=2.0,
                     help="Seconds to listen on each port while scanning (default 2.0)")
_args = _parser.parse_args()
# ─────────────────────────────────────────────────────────────────────────────

BAUD = 115200
PI_IP = _args.ip
UDP_PORT = _args.udp_port
PRINT_EVERY = 0.1
PROBE_TIMEOUT_SEC = max(0.5, float(_args.scan_timeout))


def _port_description(port_name: str) -> str:
    for info in list_ports.comports():
        if info.device == port_name:
            bits = [info.description or ""]
            if info.manufacturer:
                bits.append(info.manufacturer)
            if info.vid is not None:
                bits.append(f"VID:PID={info.vid:04X}:{info.pid:04X}")
            return " | ".join(x for x in bits if x)
    return ""


def _probe_port(port_name: str) -> bool:
    """Open port briefly and look for arm-controller CSV lines."""
    try:
        ser = serial.Serial(port_name, BAUD, timeout=0.15)
    except (serial.SerialException, PermissionError, OSError) as e:
        print(f"  skip {port_name}: {e}")
        return False
    try:
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        deadline = time.time() + PROBE_TIMEOUT_SEC
        while time.time() < deadline:
            raw = ser.readline().decode(errors="ignore").strip()
            if looks_like_arm_controller_line(raw):
                return True
        print(f"  skip {port_name}: no arm controller data in {PROBE_TIMEOUT_SEC:.1f}s")
        return False
    finally:
        ser.close()


def resolve_serial_port(requested: str) -> str:
    """Find the arm controller on a serial port (preferred first, then scan all)."""
    available = [p.device for p in list_ports.comports()]
    req = (requested or "auto").strip()

    if req.lower() in ("auto", ""):
        candidates = list(available)
    else:
        candidates = [req] + [p for p in available if p != req]

    if not candidates:
        raise SystemExit(
            "No serial ports found. Plug in the arm controller USB and try again."
        )

    print(f"Scanning {len(candidates)} serial port(s) for arm controller...")
    for port in candidates:
        desc = _port_description(port)
        label = f"{port} ({desc})" if desc else port
        print(f"  trying {label} ...")
        if _probe_port(port):
            print(f"Arm controller found on {port}")
            return port

    listed = ", ".join(candidates)
    raise SystemExit(
        f"No arm controller found. Tried: {listed}\n"
        "Check USB cable, power, and that no other program has the port open."
    )


SERIAL_PORT = resolve_serial_port(_args.port)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
except Exception as e:
    print(f"ERROR: Could not open serial port {SERIAL_PORT}: {e}")
    raise SystemExit(1)

print(f"Reading serial: {SERIAL_PORT} @ {BAUD}")
print(f"Sending UDP to: {PI_IP}:{UDP_PORT}")
print(
    "Serial format: "
    "PWM_J1,PWM_J2,PWM_J3,PWM_CLAW,ENCODER1,ENCODER2,IMU_STATUS,IMU_ANGLE,GRIP_ONOFF"
)
print(f"Example: 1400,1600,1500,1425,1,1,OK,-12.35,0  ({ARM_CONTROLLER_FIELDS} fields)")

last_print = 0

while True:
    raw = ser.readline().decode(errors="ignore").strip()

    if not raw:
        continue

    line = raw
    if line.startswith("PWM:"):
        line = line[4:]

    parsed = parse_arm_controller_csv(line.split(","))
    if parsed is None:
        print(f"BAD LINE (need {ARM_CONTROLLER_FIELDS} fields or legacy 7-field PWM): {raw}")
        continue

    send_line = format_arm_controller_csv(parsed)
    sock.sendto(send_line.encode(), (PI_IP, UDP_PORT))

    now = time.time()
    if now - last_print >= PRINT_EVERY:
        last_print = now
        print(f"RAW:  {raw}")
        print(f"SENT: {send_line}")
