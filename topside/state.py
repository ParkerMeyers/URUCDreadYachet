"""Global UI/runtime state."""
from __future__ import annotations

import threading
import time
from pathlib import Path

from topside.config import ROV_ROOT, config
from topside.constants import MAX_LOG_LINES, NEUTRAL_THR_PWM

LOGS_DIR = ROV_ROOT / "logs"
VIDEO_DIR = LOGS_DIR / "videos"

_state_lock = threading.Lock()
_telemetry_record_lock = threading.Lock()
_telemetry_rate_counter = {"count": 0, "window_start": time.time()}


def manual_aux_defaults() -> list[int]:
    claw_stop = int(config.get("arm_claw_stop_us", 1515))
    return [1500, 1500, 1500, 1500, 1500, 1500, claw_stop]


def manual_thr_defaults() -> list[int]:
    return [NEUTRAL_THR_PWM] * 8


STATE: dict = {
    "thrust_running": False,
    "arm_running": False,
    "onboard_stab": False,
    "onboard_arm": False,
    "onboard_cam": False,
    "onboard_mavproxy": False,
    "onboard_mosfet": False,
    "ssh_connected": False,
    "ssh_error": "",
    "mode": "disarmed",
    "mosfet_on": False,
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
    "claw_hold": bool(config.get("claw_hold", False)),
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
        "arm_imu_ok": False, "arm_bno_ready": False, "arm_imu_stale": False,
        "arm_imu_angle_deg": None, "arm_j6_target_deg": None, "arm_j6_pwm_out": None,
        "arm_claw_hold_request": False, "arm_claw_hold_active": False,
        "arm_j6_manual": True,
    },
    "logs": {
        "thrust": [], "arm": [], "onboard_stab": [], "onboard_arm": [],
        "onboard_cam": [], "colmap": [], "crabs": [],
    },
}
