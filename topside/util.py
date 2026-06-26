"""Small shared helpers."""
from __future__ import annotations

from topside.constants import (
    ARM_PWM_MAX,
    ARM_PWM_MIN,
    CLAW_PWM_MAX,
    CLAW_PWM_MIN,
    CLAW_STOP_US_DEFAULT,
    THR_PWM_MAX,
    THR_PWM_MIN,
)


def clamp_arm_pwm(value) -> int:
    return int(max(ARM_PWM_MIN, min(ARM_PWM_MAX, int(round(float(value))))))


def clamp_claw_pwm(value) -> int:
    return int(max(CLAW_PWM_MIN, min(CLAW_PWM_MAX, int(round(float(value))))))


def clamp_arm_pwm_list(pwms: list) -> list[int]:
    """Clamp 7 joint PWM values; index 6 (claw) uses claw limits."""
    out = [clamp_arm_pwm(v) for v in pwms[:7]]
    while len(out) < 7:
        out.append(1500)
    out[6] = clamp_claw_pwm(out[6])
    return out


def clamp_thr_pwm(value) -> int:
    return int(max(THR_PWM_MIN, min(THR_PWM_MAX, int(round(float(value))))))
