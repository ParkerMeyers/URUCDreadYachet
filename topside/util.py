"""Small shared helpers."""
from __future__ import annotations

from topside.constants import (
    AUX_TO_CSV,
    ARM_CSV_INDICES,
    ARM_PWM_MAX,
    ARM_PWM_MIN,
    CLAW_PWM_MAX,
    CLAW_PWM_MIN,
    CLAW_STOP_US_DEFAULT,
    JOINT_PWM_SPECS,
    THR_PWM_MAX,
    THR_PWM_MIN,
)


def clamp_arm_pwm(value) -> int:
    return int(max(ARM_PWM_MIN, min(ARM_PWM_MAX, int(round(float(value))))))


def clamp_claw_pwm(value) -> int:
    return int(max(CLAW_PWM_MIN, min(CLAW_PWM_MAX, int(round(float(value))))))


def clamp_joint_pwm_csv(csv_idx: int, value) -> int:
    spec = JOINT_PWM_SPECS.get(csv_idx)
    if spec is None:
        return clamp_arm_pwm(value)
    lo, hi = spec["min"], spec["max"]
    return int(max(lo, min(hi, int(round(float(value))))))


def clamp_joint_pwm_aux(aux: int, value) -> int:
    csv_idx = AUX_TO_CSV.get(int(aux))
    if csv_idx is None:
        return clamp_arm_pwm(value)
    return clamp_joint_pwm_csv(csv_idx, value)


def clamp_arm_pwm_list(pwms: list) -> list[int]:
    """Clamp 7 joint PWM values using per-joint limits at active CSV indices."""
    out = [clamp_arm_pwm(v) for v in pwms[:7]]
    while len(out) < 7:
        out.append(1500)
    for csv_idx in ARM_CSV_INDICES:
        out[csv_idx] = clamp_joint_pwm_csv(csv_idx, out[csv_idx])
    return out


def clamp_thr_pwm(value) -> int:
    return int(max(THR_PWM_MIN, min(THR_PWM_MAX, int(round(float(value))))))
