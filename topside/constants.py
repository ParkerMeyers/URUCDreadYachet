"""ROV hardware labels, PWM limits, and MAVLink port constants."""
from __future__ import annotations

import platform

from onboard.ports import (
    MAVPROXY_ONBOARD_ARM,
    MAVPROXY_ONBOARD_STAB,
    MAVPROXY_TCP_ARM,
    MAVPROXY_TCP_STAB,
)

IS_WINDOWS = platform.system() == "Windows"

# ── Arm (Pix6 AUX) — 4 DOF: J1, J2, J3, Claw ───────────────────────────────
# arm_sender CSV indices for active joints: 0=J1, 4=J2, 5=J3, 6=Claw
ARM_JOINT_NAMES = ["J1", "J2", "J3", "Claw"]
ARM_CSV_INDICES = [0, 4, 5, 6]
ARM_CSV_TO_NAME = {0: "J1", 4: "J2", 5: "J3", 6: "Claw"}
ARM_PWM_MIN = 500
ARM_PWM_MAX = 2500
CLAW_PWM_MIN = 1325
CLAW_PWM_MAX = 1525
CLAW_STOP_US_DEFAULT = 1425
ARM_DEFAULT_PWM = [1500, 1500, 1500, 1500, 1500, 1500, CLAW_STOP_US_DEFAULT]

ARM_PRESET_JOINT_ORDER = (5, 4, 0, 6)  # J3→J2→J1→Claw (CSV indices)
ARM_PRESET_DELAY_MIN_SEC = 0.45
ARM_PRESET_DELAY_MAX_SEC = 4.0

MANUAL_AUX_LABELS = ["J2", "—", "J3", "J1", "—", "—", "Claw"]
MANUAL_AUX_DEFAULTS = [1500, 1500, 1500, 1500, 1500, 1500, CLAW_STOP_US_DEFAULT]
JOINT_TO_AUX = {1: 4, 2: 1, 3: 3, 4: 7}
AUX_TO_JOINT = {v: k for k, v in JOINT_TO_AUX.items()}
ARM_MANUAL_JOINTS = [
    {"joint": 1, "name": "J1", "aux": 4},
    {"joint": 2, "name": "J2", "aux": 1},
    {"joint": 3, "name": "J3", "aux": 3},
    {"joint": 4, "name": "Claw", "aux": 7},
]

# ── Thrusters (Pix6 RC1–8) ───────────────────────────────────────────────────
MANUAL_THR_LABELS = ["FR_H", "BR_V", "BR_H", "BL_V", "FR_V", "FL_H", "FL_V", "BL_H"]
MANUAL_THR_NAMES = {
    1: "front_right_h", 2: "back_right_v", 3: "back_right_h", 4: "back_left_v",
    5: "front_right_v", 6: "front_left_h", 7: "front_left_v", 8: "back_left_h",
}
MOTOR_NAME_ALIASES = {
    "m1": 1, "motor1": 1, "frh": 1, "front_right_h": 1,
    "m2": 2, "motor2": 2, "brv": 2, "back_right_v": 2,
    "m3": 3, "motor3": 3, "brh": 3, "back_right_h": 3,
    "m4": 4, "motor4": 4, "blv": 4, "back_left_v": 4,
    "m5": 5, "motor5": 5, "frv": 5, "front_right_v": 5,
    "m6": 6, "motor6": 6, "flh": 6, "front_left_h": 6,
    "m7": 7, "motor7": 7, "flv": 7, "front_left_v": 7,
    "m8": 8, "motor8": 8, "blh": 8, "back_left_h": 8,
}
THR_PWM_MIN = 1100
THR_PWM_MAX = 1900
NEUTRAL_THR_PWM = 1500

DEFAULT_ARM_PRESETS = {
    "stow": {"label": "Stow", "pwm": [1500, 1500, 1500, 1500, 1500, 1500, CLAW_STOP_US_DEFAULT]},
    "sample": {"label": "Sample", "pwm": [1600, 1500, 1500, 1500, 1450, 1500, CLAW_PWM_MAX]},
    "deploy_claw": {"label": "Claw", "pwm": [1500, 1500, 1500, 1500, 1500, 1500, CLAW_PWM_MAX]},
}

MAVPROXY_TCP_PORT = MAVPROXY_TCP_STAB
MAVPROXY_ARM_TCP_PORT = MAVPROXY_TCP_ARM
MAVPROXY_ONBOARD_OUT = MAVPROXY_ONBOARD_STAB
MAVPROXY_ARM_ONBOARD_OUT = MAVPROXY_ONBOARD_ARM

CTRL_SEND_HZ = 50
MAX_LOG_LINES = 200

TELEMETRY_CSV_FIELDS = [
    "time", "state", "depth_m", "hold_depth_m", "yaw_deg", "roll_deg", "pitch_deg",
    "pressure_hpa", "pressure_temperature_c", "stabilize",
    "depth_hold_active", "yaw_hold_active", "gain_percent",
    "control_timeout", "mavlink_link_dead",
]
