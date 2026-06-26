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

# ── Arm (Pix6 AUX) ───────────────────────────────────────────────────────────
ARM_JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6", "Claw"]
ARM_PWM_MIN = 500
ARM_PWM_MAX = 2500
ARM_DEFAULT_PWM = [1500, 1500, 1500, 1500, 1500, 1500, 1515]

ARM_PRESET_JOINT_ORDER = (5, 4, 3, 2, 1, 0, 6)
ARM_PRESET_DELAY_MIN_SEC = 0.45
ARM_PRESET_DELAY_MAX_SEC = 4.0

MANUAL_AUX_LABELS = ["J5", "J2", "J6", "J1", "J3", "J4", "Claw"]
MANUAL_AUX_DEFAULTS = [1500, 1500, 1500, 1500, 1500, 1500, 1515]
JOINT_TO_AUX = {1: 4, 2: 2, 3: 5, 4: 6, 5: 1, 6: 3, 7: 7}
AUX_TO_JOINT = {v: k for k, v in JOINT_TO_AUX.items()}

# ── Thrusters (Pix6 RC1–8) ───────────────────────────────────────────────────
MANUAL_THR_LABELS = ["FL_H", "BL_H", "FL_V", "FR_V", "FR_H", "BR_H", "BR_V", "BL_V"]
MANUAL_THR_NAMES = {
    1: "front_left_h", 2: "back_left_h", 3: "front_left_v", 4: "front_right_v",
    5: "front_right_h", 6: "back_right_h", 7: "back_right_v", 8: "back_left_v",
}
MOTOR_NAME_ALIASES = {
    "m1": 1, "motor1": 1, "flh": 1, "front_left_h": 1,
    "m2": 2, "motor2": 2, "blh": 2, "back_left_h": 2,
    "m3": 3, "motor3": 3, "flv": 3, "front_left_v": 3,
    "m4": 4, "motor4": 4, "frv": 4, "front_right_v": 4,
    "m5": 5, "motor5": 5, "frh": 5, "front_right_h": 5,
    "m6": 6, "motor6": 6, "brh": 6, "back_right_h": 6,
    "m7": 7, "motor7": 7, "brv": 7, "back_right_v": 7,
    "m8": 8, "motor8": 8, "blv": 8, "back_left_v": 8,
}
THR_PWM_MIN = 1100
THR_PWM_MAX = 1900
NEUTRAL_THR_PWM = 1500

DEFAULT_ARM_PRESETS = {
    "stow": {"label": "Stow", "pwm": [1500, 1500, 1500, 1500, 1500, 1500, 1515], "j6_angle": 0.0},
    "sample": {"label": "Sample", "pwm": [1600, 1400, 1550, 1500, 1450, 1500, 1515], "j6_angle": 15.0},
    "deploy_claw": {"label": "Claw", "pwm": [1500, 1500, 1500, 1500, 1500, 1500, 2000], "j6_angle": 0.0},
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
