#!/usr/bin/env python3

import os
import socket
import sys
import serial
import time

SERIAL_PORT = "/dev/ttyACM0"
BAUD = 115200

PI_IP = sys.argv[1] if len(sys.argv) >= 2 else os.getenv("ROV_HOST", "10.42.0.181")
UDP_PORT = 5006

PRINT_EVERY = 0.1

J6_TARGET_MIN_DEG = -90.0
J6_TARGET_MAX_DEG = 90.0

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def clamp_pwm(x):
    return max(500, min(2500, int(round(float(x)))))

def clamp_angle(x):
    return clamp(float(x), J6_TARGET_MIN_DEG, J6_TARGET_MAX_DEG)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)

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
