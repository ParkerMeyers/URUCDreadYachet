"""Small shared helpers."""
from __future__ import annotations

from topside.constants import ARM_PWM_MAX, ARM_PWM_MIN, THR_PWM_MAX, THR_PWM_MIN


def clamp_arm_pwm(value) -> int:
    return int(max(ARM_PWM_MIN, min(ARM_PWM_MAX, int(round(float(value))))))


def clamp_thr_pwm(value) -> int:
    return int(max(THR_PWM_MIN, min(THR_PWM_MAX, int(round(float(value))))))
