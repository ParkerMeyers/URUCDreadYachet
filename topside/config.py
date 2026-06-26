"""Configuration load/save and normalization."""
from __future__ import annotations

import json
from pathlib import Path

from topside.constants import (
    DEFAULT_ARM_PRESETS,
    IS_WINDOWS,
    MAVPROXY_ARM_ONBOARD_OUT,
    MAVPROXY_ARM_TCP_PORT,
    MAVPROXY_ONBOARD_OUT,
    MAVPROXY_TCP_PORT,
)

ROV_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROV_ROOT / "rov_config.json"

DEFAULT_CONFIG = {
    "pi_ip": "192.168.69.100",
    "pi_user": "uruc",
    "pi_password": "yahboom",
    "pi_ssh_port": 22,
    "pi_rov_path": "/home/uruc/URUCDreadYachet",
    "serial_port": "auto" if IS_WINDOWS else "/dev/ttyACM0",
    "forward_camera_url": "http://192.168.69.100:8161",
    "arm_camera_url": "http://192.168.69.100:8160",
    "camera0_device": "/dev/video0",
    "camera1_device": "/dev/video2",
    "thrust_udp_port": 5005,
    "telemetry_port": 5006,
    "arm_udp_port": 5006,
    "arm_control_port": 5009,
    "arm_telemetry_port": 5006,
    "arm_claw_stop_us": 1425,
    "colmap_command": "python3 colmap_run.py",
    "crabs_command": "python3 crabs.py",
    "mavproxy_bin": "/home/uruc/mav_env/bin/mavproxy.py",
    "mavproxy_serial": "/dev/ttyACM0",
    "mavproxy_baud": "115200",
    "mavproxy_out1": "udp:192.168.69.2:14550",
    "topside_ip": "",
    "mavproxy_out2": MAVPROXY_ONBOARD_OUT,
    "mavproxy_out3": MAVPROXY_ARM_ONBOARD_OUT,
    "arm_presets": {
        k: {"label": v["label"], "pwm": list(v["pwm"])}
        for k, v in DEFAULT_ARM_PRESETS.items()
    },
}

config: dict = DEFAULT_CONFIG.copy()


def normalize_camera_config() -> None:
    c1 = str(config.get("camera1_url", "")).strip()
    c2 = str(config.get("camera2_url", "")).strip()
    if c1 and not str(config.get("arm_camera_url", "")).strip():
        config["arm_camera_url"] = c1
    if c2 and not str(config.get("forward_camera_url", "")).strip():
        config["forward_camera_url"] = c2
    if not str(config.get("forward_camera_url", "")).strip():
        config["forward_camera_url"] = DEFAULT_CONFIG["forward_camera_url"]
    if not str(config.get("arm_camera_url", "")).strip():
        config["arm_camera_url"] = DEFAULT_CONFIG["arm_camera_url"]
    if not str(config.get("camera0_device", "")).strip():
        config["camera0_device"] = DEFAULT_CONFIG["camera0_device"]
    if not str(config.get("camera1_device", "")).strip():
        config["camera1_device"] = DEFAULT_CONFIG["camera1_device"]


def slug_preset_name(name: str) -> str:
    slug = "".join(c if c.isalnum() or c in "-_" else "_" for c in str(name).strip().lower())
    slug = slug.strip("_")
    return slug or "preset"


def normalize_preset_entry(raw) -> dict | None:
    from topside.util import clamp_arm_pwm_list

    if isinstance(raw, str):
        parts = raw.replace("PWM:", "").split(",")
        if len(parts) < 7:
            return None
        try:
            pwms = clamp_arm_pwm_list(parts)
            return {"label": "Preset", "pwm": pwms}
        except (TypeError, ValueError):
            return None
    if not isinstance(raw, dict):
        return None
    pwm_in = raw.get("pwm", [])
    if not isinstance(pwm_in, (list, tuple)) or len(pwm_in) < 7:
        return None
    try:
        pwms = clamp_arm_pwm_list(list(pwm_in))
    except (TypeError, ValueError):
        return None
    label = str(raw.get("label") or raw.get("name") or "Preset").strip() or "Preset"
    return {"label": label, "pwm": pwms}


def normalize_arm_presets() -> None:
    raw = config.get("arm_presets")
    cleaned = {}
    if isinstance(raw, dict):
        for name, entry in raw.items():
            slug = slug_preset_name(name)
            norm = normalize_preset_entry(entry)
            if norm:
                if not norm["label"] or norm["label"] == "Preset":
                    norm["label"] = slug.replace("_", " ").title()
                cleaned[slug] = norm
    if not cleaned:
        cleaned = {
            k: {"label": v["label"], "pwm": list(v["pwm"])}
            for k, v in DEFAULT_ARM_PRESETS.items()
        }
    config["arm_presets"] = cleaned


def normalize_onboard_config() -> None:
    normalize_camera_config()
    normalize_arm_presets()
    try:
        config["arm_control_port"] = int(config.get("arm_control_port", config.get("arm_udp_port", 5006)))
    except (TypeError, ValueError):
        config["arm_control_port"] = int(config.get("arm_udp_port", 5006))
    try:
        config["arm_udp_port"] = int(config.get("arm_udp_port", 5006))
    except (TypeError, ValueError):
        config["arm_udp_port"] = 5006
    if config["arm_control_port"] == config["arm_udp_port"]:
        config["arm_control_port"] = 5009
    out2 = str(config.get("mavproxy_out2", "")).strip()
    if "tcpin" not in out2.lower() or str(MAVPROXY_TCP_PORT) not in out2:
        config["mavproxy_out2"] = MAVPROXY_ONBOARD_OUT
    out3 = str(config.get("mavproxy_out3", "")).strip()
    if "tcpin" not in out3.lower() or str(MAVPROXY_ARM_TCP_PORT) not in out3:
        config["mavproxy_out3"] = MAVPROXY_ARM_ONBOARD_OUT


def load_config_file() -> None:
    if not CONFIG_PATH.is_file():
        return
    try:
        data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        for k, v in data.items():
            if k in config:
                config[k] = v
        normalize_onboard_config()
    except Exception as e:
        print(f"[WARN] Could not load {CONFIG_PATH}: {e}")


def save_config_file() -> None:
    try:
        CONFIG_PATH.write_text(json.dumps(config, indent=2, sort_keys=True), encoding="utf-8")
    except Exception as e:
        print(f"[WARN] Could not save {CONFIG_PATH}: {e}")


normalize_onboard_config()
load_config_file()
