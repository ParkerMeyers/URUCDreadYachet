#!/usr/bin/env python3
"""
PWM oscillate test — ONBOARD (runs on the Raspberry Pi)
========================================================
Smoothly oscillates PWM on GPIO pin 12 between 1400 µs and 1600 µs
using RPi.GPIO software PWM (pre-installed on Raspberry Pi OS).

Usage:
    python3 test/pwm_oscillate_test.py
"""

import math
import time
import RPi.GPIO as GPIO

PIN        = 12
LOW_US     = 1400
HIGH_US    = 1600
PERIOD_S   = 2.0    # time for one full oscillation cycle
UPDATE_HZ  = 50     # updates per second (matches typical ESC/servo rate)
PWM_FREQ   = 50     # Hz — standard servo/ESC frequency (20 ms period)

def us_to_duty(pulse_us: float) -> float:
    """Convert pulse width in µs to duty cycle % for a 50 Hz PWM signal."""
    return pulse_us / 20_000.0 * 100.0

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.OUT)
pwm = GPIO.PWM(PIN, PWM_FREQ)
pwm.start(us_to_duty(1500))   # start at neutral

print(f"Oscillating PWM on GPIO {PIN}: {LOW_US}–{HIGH_US} µs  "
      f"(period={PERIOD_S}s, {UPDATE_HZ} Hz)  Ctrl-C to stop")

try:
    t0 = time.time()
    while True:
        elapsed = time.time() - t0
        sine     = math.sin(2 * math.pi * elapsed / PERIOD_S)
        mid      = (HIGH_US + LOW_US) / 2
        amp      = (HIGH_US - LOW_US) / 2
        pulse_us = mid + amp * sine

        pwm.ChangeDutyCycle(us_to_duty(pulse_us))
        time.sleep(1.0 / UPDATE_HZ)

except KeyboardInterrupt:
    print("\nStopping — setting neutral then cleaning up")
    pwm.ChangeDutyCycle(us_to_duty(1500))
    time.sleep(0.5)
    pwm.stop()
    GPIO.cleanup()
