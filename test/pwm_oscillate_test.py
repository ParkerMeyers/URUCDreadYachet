#!/usr/bin/env python3
"""
PWM oscillate test — ONBOARD (runs on the Raspberry Pi)
========================================================
Smoothly oscillates PWM on GPIO pin 12 between 1400 µs and 1600 µs
using pigpio for hardware-accurate pulse widths.

Usage:
    sudo pigpiod          # start the pigpio daemon if not already running
    python3 test/pwm_oscillate_test.py
"""

import math
import time
import pigpio

PIN        = 12
LOW_US     = 1400
HIGH_US    = 1600
PERIOD_S   = 2.0    # time for one full oscillation cycle
UPDATE_HZ  = 50     # updates per second (matches typical ESC/servo rate)

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Cannot connect to pigpio daemon — did you run 'sudo pigpiod'?")

print(f"Oscillating PWM on GPIO {PIN}: {LOW_US}–{HIGH_US} µs  "
      f"(period={PERIOD_S}s, {UPDATE_HZ} Hz)  Ctrl-C to stop")

try:
    t0 = time.time()
    while True:
        elapsed = time.time() - t0
        # sine wave: -1..+1, map to LOW_US..HIGH_US
        sine = math.sin(2 * math.pi * elapsed / PERIOD_S)
        mid  = (HIGH_US + LOW_US) / 2
        amp  = (HIGH_US - LOW_US) / 2
        pulse_us = int(mid + amp * sine)

        pi.set_servo_pulsewidth(PIN, pulse_us)
        time.sleep(1.0 / UPDATE_HZ)

except KeyboardInterrupt:
    print("\nStopping — setting neutral (1500 µs) then disabling PWM")
    pi.set_servo_pulsewidth(PIN, 1500)
    time.sleep(0.5)
    pi.set_servo_pulsewidth(PIN, 0)   # turn off PWM
    pi.stop()
