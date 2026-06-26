"""Global UI/runtime state."""
from __future__ import annotations

import threading
import time
from pathlib import Path

from topside.config import ROV_ROOT, config
from topside.constants import CLAW_STOP_US_DEFAULT, MANUAL_AUX_DEFAULTS, MAX_LOG_LINES, NEUTRAL_THR_PWM

LOGS_DIR = ROV_ROOT / "logs"
VIDEO_DIR = LOGS_DIR / "videos"

_state_lock = threading.Lock()
_telemetry_record_lock = threading.Lock()
_telemetry_rate_counter = {"count": 0, "window_start": time.time()}


def manual_aux_defaults() -> list[int]:
    claw_stop = int(config.get("arm_claw_stop_us", CLAW_STOP_US_DEFAULT))
    defaults = list(MANUAL_AUX_DEFAULTS)
    defaults[6] = claw_stop
    return defaults


def manual_thr_defaults() -> list[int]:
    return [NEUTRAL_THR_PWM] * 8


STATE: dict = {
    "thrust_running": False,
    "arm_running": False,
    "onboard_stab": False,
    "onboard_arm": False,
    "onboard_cam": False,
    "onboard_mavproxy": False,
    "pixhawk_serial_ok": False,
    "mavproxy_detail": "",
    "ssh_connected": False,
    "ssh_error": "",
    "mode": "disarmed",
    "last_telemetry_time": 0.0,
    "last_arm_telemetry_time": 0.0,
    "telemetry_packets": 0,
    "telemetry_rate_hz": 0.0,
    "last_ctrl_time": 0.0,
    "ctrl_stabilize": False,
    "ctrl_depth_hold": False,
    "ctrl_yaw_hold": False,
    "telemetry_listener_ok": False,
    "telemetry_recording": False,
    "telemetry_record_file": "",
    "video_recording": False,
    "video_record_session": "",
    "video_record_mode": "",
    "video_record_files": [],
    "onboard_starting": False,
    "onboard_progress": [],
    "arm_last_pwm": None,
    "preset_running": False,
    "preset_active_name": "",
    "manual_pwm_enabled": False,
    "manual_aux_pwm": list(manual_aux_defaults()),
    "manual_thr_pwm": list(manual_thr_defaults()),
    "telemetry": {
        "rx_state": "NO_TELEMETRY",
        "gain_percent": 100,
        "cmd_forward": 0.0, "cmd_lateral": 0.0, "cmd_yaw": 0.0, "cmd_vertical": 0.0,
        "stabilize": False,
        "depth_hold_request": False, "depth_hold_active": False,
        "yaw_hold_request": False, "yaw_hold_active": False,
        "depth_m": None, "hold_depth_m": None,
        "yaw_deg": None, "hold_yaw_deg": None,
        "roll_deg": None, "pitch_deg": None,
        "h_group": 0.0, "v_group": 0.0,
        "pressure_hpa": None, "temperature_c": None,
        "control_timeout": False, "attitude_stale": False, "depth_stale": False,
        "mavlink_link_dead": False, "mavlink_last_rx_age_sec": None,
        "attitude_age_sec": None,
        "depth_recapture_pending": False, "yaw_recapture_pending": False,
        "arm_hold_neutral": None, "arm_rx_count": None, "arm_enabled": None,
        "arm_mavlink_ok": None, "arm_joint_us": None,
        "arm_manual_mode": None, "arm_preset_motion": None,
    },
    "logs": {
        "thrust": [], "arm": [], "onboard_stab": [], "onboard_arm": [],
        "onboard_cam": [], "colmap": [], "crabs": [],
    },
}
