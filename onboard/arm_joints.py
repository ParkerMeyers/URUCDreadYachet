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

# Arm USB controller serial / UDP (motor-native µs + sensors):
#   J1, J2, J3, Claw, ENC1, ENC2, IMU_STATUS, IMU_ANGLE, GRIP_ONOFF
ARM_CONTROLLER_FIELDS = 9
IMU_ANGLE_MIN_DEG = -90.0
IMU_ANGLE_MAX_DEG = 90.0


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


def _parse_status_int(value) -> int | None:
    try:
        return int(round(float(str(value).strip())))
    except (TypeError, ValueError):
        return None


def _parse_imu_status(value) -> str | None:
    text = str(value).strip()
    return text if text else None


def _parse_imu_angle(value) -> float | None:
    try:
        angle = float(str(value).strip())
    except (TypeError, ValueError):
        return None
    return max(IMU_ANGLE_MIN_DEG, min(IMU_ANGLE_MAX_DEG, angle))


def _parse_grip_onoff(value) -> bool | None:
    status = _parse_status_int(value)
    if status is None:
        return None
    return bool(status)


def _sensor_fields(
    *,
    encoder1_status: int | None = None,
    encoder2_status: int | None = None,
    imu_status: str | None = None,
    imu_angle_deg: float | None = None,
    grip_on: bool | None = None,
) -> dict:
    return {
        "encoder1_status": encoder1_status,
        "encoder2_status": encoder2_status,
        "imu_status": imu_status,
        "imu_angle_deg": imu_angle_deg,
        "grip_on": grip_on,
    }


def parse_arm_controller_csv(
    parts: list,
    *,
    claw_stop: int = CLAW_STOP_US_DEFAULT,
) -> dict | None:
    """
    Parse arm controller CSV.

    New format (9 fields):
        PWM_J1, PWM_J2, PWM_J3, PWM_CLAW,
        ENCODER1_STATUS, ENCODER2_STATUS, IMU_STATUS, IMU_ANGLE, GRIP_ONOFF

    Legacy format (7 fields): padded J1..Claw at CSV indices 0,4,5,6.
    """
    fields = [str(p).strip() for p in parts]
    while fields and not fields[-1]:
        fields.pop()
    if not fields:
        return None

    if len(fields) >= ARM_CONTROLLER_FIELDS:
        try:
            joint_pwm = {
                1: clamp_joint_pwm(1, fields[0], claw_stop=claw_stop),
                2: clamp_joint_pwm(2, fields[1], claw_stop=claw_stop),
                3: clamp_joint_pwm(3, fields[2], claw_stop=claw_stop),
                4: clamp_joint_pwm(4, fields[3], claw_stop=claw_stop),
            }
        except (TypeError, ValueError):
            return None
        sensors = _sensor_fields(
            encoder1_status=_parse_status_int(fields[4]),
            encoder2_status=_parse_status_int(fields[5]),
            imu_status=_parse_imu_status(fields[6]),
            imu_angle_deg=_parse_imu_angle(fields[7]),
            grip_on=_parse_grip_onoff(fields[8]),
        )
        return {
            "joint_pwm": joint_pwm,
            "pwm_csv": joint_pwm_to_csv_list(joint_pwm),
            **sensors,
        }

    if len(fields) >= 7:
        try:
            vals = [float(x) for x in fields[:7]]
        except ValueError:
            return None
        joint_pwm = csv_list_to_joint_pwm(vals, claw_stop=claw_stop)
        sensors = _sensor_fields()
        if len(fields) >= 8:
            sensors["imu_angle_deg"] = _parse_imu_angle(fields[7])
        return {
            "joint_pwm": joint_pwm,
            "pwm_csv": joint_pwm_to_csv_list(joint_pwm),
            **sensors,
        }

    return None


def format_arm_controller_csv(parsed: dict) -> str:
    """9-field wire CSV for UDP (motor PWM + sensor snapshot)."""
    joint_pwm = parsed["joint_pwm"]
    enc1 = parsed.get("encoder1_status")
    enc2 = parsed.get("encoder2_status")
    imu_status = parsed.get("imu_status") or ""
    angle = parsed.get("imu_angle_deg")
    grip = parsed.get("grip_on")
    angle_s = f"{float(angle):.2f}" if angle is not None else "0.00"
    grip_s = "1" if grip else "0"
    return (
        f"{joint_pwm[1]},{joint_pwm[2]},{joint_pwm[3]},{joint_pwm[4]},"
        f"{0 if enc1 is None else int(enc1)},{0 if enc2 is None else int(enc2)},"
        f"{imu_status},{angle_s},{grip_s}"
    )


def looks_like_arm_controller_line(raw: str) -> bool:
    """True when a serial line matches arm controller CSV output."""
    line = raw.strip()
    if not line:
        return False
    if line.startswith("PWM:"):
        line = line[4:]
    return parse_arm_controller_csv(line.split(",")) is not None


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
