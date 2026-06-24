#!/usr/bin/env python3
"""
PWM oscillate test — ONBOARD (runs on the Raspberry Pi)
========================================================
Smoothly oscillates PWM on GPIO pin 12 between 1400 µs and 1600 µs
using lgpio (pre-installed on Pi OS Bookworm, supports Pi 4 and Pi 5).

Usage:
    python3 test/pwm_oscillate_test.py
"""

import math
import time
import lgpio

PIN       = 12
LOW_US    = 1400
HIGH_US   = 1600
PERIOD_S  = 2.0   # time for one full oscillation cycle
UPDATE_HZ = 50    # updates per second

h = lgpio.gpiochip_open(0)

print(f"Oscillating PWM on GPIO {PIN}: {LOW_US}–{HIGH_US} µs  "
      f"(period={PERIOD_S}s, {UPDATE_HZ} Hz)  Ctrl-C to stop")

try:
    t0 = time.time()
    while True:
        elapsed  = time.time() - t0
        sine     = math.sin(2 * math.pi * elapsed / PERIOD_S)
        mid      = (HIGH_US + LOW_US) / 2
        amp      = (HIGH_US - LOW_US) / 2
        pulse_us = int(mid + amp * sine)

        # tx_servo(handle, gpio, pulse_width_us, freq_hz=50)
        lgpio.tx_servo(h, PIN, pulse_us)
        time.sleep(1.0 / UPDATE_HZ)

except KeyboardInterrupt:
    print("\nStopping — neutral then off")
    lgpio.tx_servo(h, PIN, 1500)
    time.sleep(0.5)
    lgpio.tx_servo(h, PIN, 0)   # 0 disables the servo pulse
    lgpio.gpiochip_close(h)
