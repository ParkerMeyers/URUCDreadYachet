#!/usr/bin/env python3
"""Shared arm joint map and MAVLink RC override helpers (J1–J3 + Claw)."""
from __future__ import annotations

RC_IGNORE = 65535
CENTER_US = 1500
SPARE_RC_CH = 16
REMOVED_RC_CHS = (10, 12, 14)  # AUX2, AUX4, AUX6 — no hardware

NUM_JOINTS = 4

JOINT_NAMES = {1: "J1", 2: "J2", 3: "J3", 4: "Claw"}
JOINT_TO_AUX = {1: 5, 2: 1, 3: 3, 4: 7}
AUX_TO_JOINT = {v: k for k, v in JOINT_TO_AUX.items()}
JOINT_TO_MOTOR = {1: 13, 2: 9, 3: 11, 4: 15}
JOINT_CONTINUOUS = {3, 4}

CLAW_MIN_US = 1325
CLAW_MAX_US = 1525
CLAW_STOP_US_DEFAULT = 1425

JOINT_LIMITS = {
    1: (500, 2350, 1400),   # M13 — positional, neutral 1400
    2: (950, 2200, 1600),   # M9  — positional, neutral 1600
    3: (1300, 1700, 1500),  # M11 — continuous, stop 1500
    4: (CLAW_MIN_US, CLAW_MAX_US, CLAW_STOP_US_DEFAULT),  # M15 — open/close/stop
}

# arm_sender CSV index → joint number (legacy 7-field wire format)
CSV_IDX_TO_JOINT = {0: 1, 4: 2, 5: 3, 6: 4}
JOINT_TO_CSV_IDX = {1: 0, 2: 4, 3: 5, 4: 6}
ARM_CSV_INDICES = list(CSV_IDX_TO_JOINT.keys())


def joint_center_us(joint: int, *, claw_stop: int | None = None) -> int:
    if joint == 4 and claw_stop is not None:
        return int(claw_stop)
    return JOINT_LIMITS[joint][2]


def joint_pwm_range(joint: int) -> tuple[int, int]:
    lo, hi, _ = JOINT_LIMITS[joint]
    return lo, hi


def joint_to_rc_ch(joint: int) -> int:
    return JOINT_TO_AUX[joint] + 8


def clamp_joint_pwm(joint: int, pwm, *, claw_stop: int | None = None) -> int:
    lo, hi, _ = JOINT_LIMITS[joint]
    return int(max(lo, min(hi, round(float(pwm)))))


def default_joint_pwm(*, claw_stop: int = CLAW_STOP_US_DEFAULT) -> dict[int, int]:
    return {
        j: joint_center_us(j, claw_stop=claw_stop if j == 4 else None)
        for j in range(1, NUM_JOINTS + 1)
    }


def csv_list_to_joint_pwm(values: list, *, claw_stop: int = CLAW_STOP_US_DEFAULT) -> dict[int, int]:
    """Convert arm_sender 7-field CSV to joint PWM dict."""
    pwm = default_joint_pwm(claw_stop=claw_stop)
    for csv_idx, joint in CSV_IDX_TO_JOINT.items():
        if csv_idx < len(values):
            pwm[joint] = clamp_joint_pwm(joint, values[csv_idx], claw_stop=claw_stop)
    return pwm


def joint_pwm_to_csv_list(joint_pwm: dict[int, int]) -> list[int]:
    """7-field CSV for telemetry / presets (removed slots held at 1500)."""
    csv = [CENTER_US] * 7
    for joint, us in joint_pwm.items():
        csv[JOINT_TO_CSV_IDX[joint]] = int(us)
    return csv


def build_rc_override(
    joint_pwm: dict[int, int],
    *,
    ignore: int = RC_IGNORE,
) -> list[int]:
    """Build 18-channel RC_CHANNELS_OVERRIDE from joints 1–4."""
    rc = [ignore] * 18
    rc[SPARE_RC_CH - 1] = CENTER_US
    for joint in range(1, NUM_JOINTS + 1):
        us = joint_pwm.get(joint, joint_center_us(joint))
        rc_ch = joint_to_rc_ch(joint)
        lo, hi = joint_pwm_range(joint)
        rc[rc_ch - 1] = clamp_joint_pwm(joint, us)
    for rc_ch in REMOVED_RC_CHS:
        rc[rc_ch - 1] = CENTER_US
    return rc
