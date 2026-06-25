#!/usr/bin/env python3

import socket
import serial
import time
import argparse
from serial.tools import list_ports

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

J6_TARGET_MIN_DEG = -90.0
J6_TARGET_MAX_DEG = 90.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def clamp_pwm(x):
    return max(500, min(2500, int(round(float(x)))))


def clamp_angle(x):
    return clamp(float(x), J6_TARGET_MIN_DEG, J6_TARGET_MAX_DEG)


def _looks_like_arm_line(raw: str) -> bool:
    """True when a serial line matches arm controller PWM CSV output."""
    line = raw.strip()
    if not line:
        return False
    if line.startswith("PWM:"):
        line = line[4:]
    parts = line.split(",")
    if len(parts) < 8:
        return False
    try:
        for part in parts[:8]:
            float(part.strip())
        return True
    except ValueError:
        return False


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
    """Open port briefly and look for arm-controller PWM lines."""
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
            if _looks_like_arm_line(raw):
                return True
        print(f"  skip {port_name}: no arm PWM data in {PROBE_TIMEOUT_SEC:.1f}s")
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
print("Output format: PWM1,PWM2,PWM3,PWM4,PWM5,J6_PWM,PWM7,J6_TARGET_ANGLE")
print("Example: 1500,1500,1500,1500,1500,1500,1500,-12.35")

last_print = 0

while True:
    raw = ser.readline().decode(errors="ignore").strip()

    if not raw:
        continue

    line = raw

    if line.startswith("PWM:"):
        line = line[4:]

    parts = line.split(",")

    if len(parts) < 8:
        print(f"BAD SHORT LINE, need 8 values: {raw}")
        continue

    try:
        pwms = [clamp_pwm(x) for x in parts[:7]]

        # 8th serial value from controller becomes J6 target angle
        j6_target_angle = clamp_angle(parts[7])
        j6_target_angle_text = f"{j6_target_angle:.2f}"

    except ValueError:
        print(f"BAD NUMBER LINE: {raw}")
        continue

    send_line = ",".join(str(x) for x in pwms) + f",{j6_target_angle_text}"
    sock.sendto(send_line.encode(), (PI_IP, UDP_PORT))

    now = time.time()
    if now - last_print >= PRINT_EVERY:
        last_print = now
        print(f"RAW:  {raw}")
        print(f"SENT: {send_line}")
