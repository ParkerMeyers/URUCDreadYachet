#!/usr/bin/env python3
"""Topside arm controller serial -> UDP bridge."""

import os
import socket
import sys
import time

SERIAL_PORT = os.getenv("ROV_ARM_SERIAL", "/dev/ttyACM0")
BAUD = int(os.getenv("ROV_ARM_BAUD", "115200"))
UDP_PORT = 5006
PRINT_EVERY = 0.1
RETRY_SEC = 2.0

J6_TARGET_MIN_DEG = -90.0
J6_TARGET_MAX_DEG = 90.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def clamp_pwm(x):
    return max(500, min(2500, int(round(float(x)))))


def clamp_angle(x):
    return clamp(float(x), J6_TARGET_MIN_DEG, J6_TARGET_MAX_DEG)


def open_serial():
    import serial

    return serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)


def main():
    if len(sys.argv) >= 2:
        pi_ip = sys.argv[1]
    else:
        pi_ip = os.getenv("ROV_HOST", "10.42.0.181")
        print(f"No IP given. Using default Pi IP: {pi_ip}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    last_print = 0.0
    ser = None

    print(f"Arm sender target: {pi_ip}:{UDP_PORT}")
    print(f"Serial port: {SERIAL_PORT} @ {BAUD}")
    print("Waiting for serial device...")

    while ser is None:
        try:
            ser = open_serial()
            print(f"Serial connected: {SERIAL_PORT}")
        except Exception as exc:
            print(f"Serial not ready ({exc}). Retrying in {RETRY_SEC:.0f}s...")
            time.sleep(RETRY_SEC)

    print("Output format: PWM1,PWM2,PWM3,PWM4,PWM5,J6_PWM,PWM7,J6_TARGET_ANGLE")

    try:
        while True:
            raw = ser.readline().decode(errors="ignore").strip()
            if not raw:
                continue

            line = raw[4:] if raw.startswith("PWM:") else raw
            parts = line.split(",")

            if len(parts) < 8:
                print(f"BAD SHORT LINE, need 8 values: {raw}")
                continue

            try:
                pwms = [clamp_pwm(x) for x in parts[:7]]
                j6_target_angle = clamp_angle(parts[7])
            except ValueError:
                print(f"BAD NUMBER LINE: {raw}")
                continue

            send_line = ",".join(str(x) for x in pwms) + f",{j6_target_angle:.2f}"
            sock.sendto(send_line.encode(), (pi_ip, UDP_PORT))

            now = time.time()
            if now - last_print >= PRINT_EVERY:
                last_print = now
                print(f"RAW:  {raw}")
                print(f"SENT: {send_line}")

    except KeyboardInterrupt:
        print("\nArm sender stopped.")
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
