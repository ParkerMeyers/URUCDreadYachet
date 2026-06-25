#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
"""
ROV Web Control UI — DreadYachet ROV Main Control System
=========================================================

A fully web-based dashboard for the DreadYachet ROV.
Gamepad control is handled directly in the browser (replaces thrust_sender.py pygame UI).

Architecture:
  Browser Gamepad API → WebSocket → Flask → UDP 5005 → Pi stabilization.py
  Pi stabilization.py → UDP 5006 → Flask → WebSocket → Browser telemetry

Features:
  - Full Gamepad API control (same keybinds/axes/logic as thrust_sender.py)
  - Keybinds/Controls reference screen
  - SSH to Pi → launch/monitor stabilization.py + new_ar.py
  - Local process launch for arm_sender.py
  - Live MJPEG camera feed proxying from Pi
  - Real-time telemetry via WebSocket (JSON from stabilization.py directly)
  - Direction HUD overlay on camera feeds
  - MOSFET / servo power toggle (via UDP to Pi)
  - Drive mode selection (Disarmed / Armed / Stabilize)
  - COLMAP and Crabs sequence SSH commands

Dependencies:
    pip install flask flask-socketio paramiko requests

Usage:
    python rov_ui.py
    Then open http://localhost:8080
"""

import os
import sys
import re
import shlex
import json
import time
import socket
import logging
import platform
import argparse
import threading
import subprocess
import webbrowser
from pathlib import Path

IS_WINDOWS = platform.system() == "Windows"
PYTHON = sys.executable
ROV_ROOT = Path(__file__).parent

log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)

try:
    from flask import Flask, Response, request, jsonify, render_template, stream_with_context
    from flask_socketio import SocketIO
except ImportError:
    print("Missing Flask dependencies. Run:\n  pip install flask flask-socketio")
    sys.exit(1)

try:
    import paramiko
    HAVE_PARAMIKO = True
except ImportError:
    HAVE_PARAMIKO = False

try:
    import requests as _requests
    HAVE_REQUESTS = True
except ImportError:
    HAVE_REQUESTS = False

# ─────────────────────────────────────────────────────────────────────────────
# FLASK + SOCKETIO SETUP
# ─────────────────────────────────────────────────────────────────────────────

app = Flask(__name__)
app.config["SECRET_KEY"] = "dreadyachet-rov-2025"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# UDP socket for sending control packets to Pi (replaces thrust_sender.py)
_pi_ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Control keepalive — Pi only sends telemetry after it receives control UDP packets
_ctrl_lock = threading.Lock()
_last_browser_ctrl: dict | None = None
_last_browser_ctrl_time = 0.0
_ctrl_keepalive_seq = 0

# ─────────────────────────────────────────────────────────────────────────────
# DEFAULT CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

ARM_JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6", "Claw"]
ARM_PWM_MIN = 500
ARM_PWM_MAX = 2500

DEFAULT_ARM_PRESETS = {
    "stow": {
        "label": "Stow",
        "pwm": [1500, 1500, 1500, 1500, 1500, 1500, 1500],
        "j6_angle": 0.0,
    },
    "sample": {
        "label": "Sample",
        "pwm": [1600, 1400, 1550, 1500, 1450, 1500, 1500],
        "j6_angle": 15.0,
    },
    "deploy_claw": {
        "label": "Claw",
        "pwm": [1500, 1500, 1500, 1500, 1500, 1500, 2000],
        "j6_angle": 0.0,
    },
}

DEFAULT_CONFIG = {
    "pi_ip":               "192.168.69.100",
    "pi_user":             "uruc",
    "pi_password":         "yahboom",
    "pi_ssh_port":         22,
    "pi_rov_path":         "/home/uruc/URUCDreadYachet",
    "serial_port":         "COM3" if IS_WINDOWS else "/dev/ttyACM0",
    "forward_camera_url":  "http://192.168.69.100:8161",
    "arm_camera_url":      "http://192.168.69.100:8160",
    "camera0_device":      "/dev/video0",   # Pi cam0 → port 8160 (arm USB)
    "camera1_device":      "/dev/video2",   # Pi cam1 → port 8161 (forward USB)
    "thrust_udp_port":     5005,
    "telemetry_port":      5006,
    "arm_udp_port":        5006,
    "mosfet_control_port": 5007,
    "colmap_command":      "python3 colmap_run.py",
    "crabs_command":       "python3 crabs.py",
    "battery_warn_v":      12.0,
    "battery_crit_v":      11.0,
    "mavproxy_bin":        "/home/uruc/mav_env/bin/mavproxy.py",
    "mavproxy_serial":     "/dev/ttyACM0",
    "mavproxy_baud":       "115200",
    "mavproxy_out1":       "udp:192.168.69.2:14550",
    "mavproxy_out2":       "tcpin:127.0.0.1:5762",  # onboard: stab + arm (TCP)
    "arm_presets": {
        k: {
            "label": v["label"],
            "pwm": list(v["pwm"]),
            "j6_angle": float(v["j6_angle"]),
        }
        for k, v in DEFAULT_ARM_PRESETS.items()
    },
}

# Must match onboard/mavlink_rc.py MAVLINK_ONBOARD tcp port.
MAVPROXY_TCP_PORT = 5762
MAVPROXY_ONBOARD_OUT = f"tcpin:127.0.0.1:{MAVPROXY_TCP_PORT}"


def normalize_camera_config():
    """Keep forward/arm URLs and Pi USB wiring consistent for this ROV."""
    global config
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
    # Fixed wiring — onboard restart must not swap streams on the Pi ports.
    config["camera0_device"] = DEFAULT_CONFIG["camera0_device"]
    config["camera1_device"] = DEFAULT_CONFIG["camera1_device"]


def normalize_onboard_config():
    """Force the onboard MAVProxy output to TCP — scripts connect to tcp:127.0.0.1:5762."""
    global config
    normalize_camera_config()
    normalize_arm_presets()
    out2 = str(config.get("mavproxy_out2", "")).strip()
    if "tcpin" not in out2.lower() or str(MAVPROXY_TCP_PORT) not in out2:
        config["mavproxy_out2"] = MAVPROXY_ONBOARD_OUT


def _slug_preset_name(name: str) -> str:
    slug = "".join(c if c.isalnum() or c in "-_" else "_" for c in str(name).strip().lower())
    slug = slug.strip("_")
    return slug or "preset"


def _clamp_arm_pwm(value) -> int:
    return int(max(ARM_PWM_MIN, min(ARM_PWM_MAX, int(round(float(value))))))


def _preset_to_csv(preset: dict) -> str:
    pwms = [_clamp_arm_pwm(v) for v in preset["pwm"][:7]]
    while len(pwms) < 7:
        pwms.append(1500)
    angle = float(preset.get("j6_angle", 0.0))
    return ",".join(str(x) for x in pwms) + f",{angle:.2f}"


def _normalize_preset_entry(raw) -> dict | None:
    if isinstance(raw, str):
        parts = raw.replace("PWM:", "").split(",")
        if len(parts) < 7:
            return None
        try:
            pwms = [_clamp_arm_pwm(x) for x in parts[:7]]
            angle = float(parts[7]) if len(parts) >= 8 else 0.0
            return {"label": "Preset", "pwm": pwms, "j6_angle": angle}
        except (TypeError, ValueError):
            return None
    if not isinstance(raw, dict):
        return None
    pwm_in = raw.get("pwm", [])
    if not isinstance(pwm_in, (list, tuple)) or len(pwm_in) < 7:
        return None
    try:
        pwms = [_clamp_arm_pwm(x) for x in pwm_in[:7]]
        angle = float(raw.get("j6_angle", 0.0))
    except (TypeError, ValueError):
        return None
    label = str(raw.get("label") or raw.get("name") or "Preset").strip() or "Preset"
    return {"label": label, "pwm": pwms, "j6_angle": angle}


def normalize_arm_presets():
    """Ensure arm_presets in config is a valid name → preset dict map."""
    global config
    raw = config.get("arm_presets")
    cleaned = {}
    if isinstance(raw, dict):
        for name, entry in raw.items():
            slug = _slug_preset_name(name)
            norm = _normalize_preset_entry(entry)
            if norm:
                if not norm["label"] or norm["label"] == "Preset":
                    norm["label"] = slug.replace("_", " ").title()
                cleaned[slug] = norm
    if not cleaned:
        cleaned = {
            k: {
                "label": v["label"],
                "pwm": list(v["pwm"]),
                "j6_angle": float(v["j6_angle"]),
            }
            for k, v in DEFAULT_ARM_PRESETS.items()
        }
    config["arm_presets"] = cleaned


def _parse_arm_sent_line(line: str) -> dict | None:
    text = line.strip()
    if not text or text.startswith("BAD "):
        return None
    if "SENT:" in text:
        text = text.split("SENT:", 1)[1].strip()
    elif text.startswith("RAW:"):
        return None
    if text.startswith("PWM:"):
        text = text[4:]
    parts = text.split(",")
    if len(parts) < 7:
        return None
    try:
        pwms = [_clamp_arm_pwm(x) for x in parts[:7]]
        angle = float(parts[7]) if len(parts) >= 8 else 0.0
        return {"pwm": pwms, "j6_angle": angle}
    except (TypeError, ValueError):
        return None


def _send_arm_csv(csv_line: str):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(
            csv_line.encode("utf-8"),
            (config["pi_ip"], int(config["arm_udp_port"])),
        )
    finally:
        sock.close()


config = DEFAULT_CONFIG.copy()
normalize_onboard_config()

CONFIG_PATH = ROV_ROOT / "rov_config.json"

TELEMETRY_CSV_FIELDS = [
    "time", "state", "depth_m", "hold_depth_m", "yaw_deg", "roll_deg", "pitch_deg",
    "battery_voltage_v", "battery_current_a", "battery_remaining_pct",
    "pressure_hpa", "pressure_temperature_c", "stabilize",
    "depth_hold_active", "yaw_hold_active", "gain_percent",
    "control_timeout", "mavlink_link_dead",
]


def load_config_file():
    """Load persisted config from disk if present."""
    global config
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


def save_config_file():
    """Persist current config to disk."""
    try:
        CONFIG_PATH.write_text(
            json.dumps(config, indent=2, sort_keys=True),
            encoding="utf-8",
        )
    except Exception as e:
        print(f"[WARN] Could not save {CONFIG_PATH}: {e}")


load_config_file()


# ─────────────────────────────────────────────────────────────────────────────
# GLOBAL STATE
# ─────────────────────────────────────────────────────────────────────────────

STATE = {
    "thrust_running":      False,
    "arm_running":         False,
    "onboard_stab":        False,
    "onboard_arm":         False,
    "onboard_cam":         False,
    "onboard_mavproxy":    False,
    "ssh_connected":       False,
    "ssh_error":           "",
    "mode":                "disarmed",
    "mosfet_on":           False,
    "last_telemetry_time": 0.0,
    "telemetry_packets":   0,
    "telemetry_rate_hz":   0.0,
    "last_ctrl_time":      0.0,
    "telemetry_listener_ok": False,
    "telemetry_recording":   False,
    "telemetry_record_file": "",
    "onboard_starting":      False,
    "onboard_progress":      [],
    "arm_last_pwm":          None,
    "telemetry": {
        "rx_state":                "NO_TELEMETRY",
        "gain_percent":            100,
        "cmd_forward":             0.0,
        "cmd_lateral":             0.0,
        "cmd_yaw":                 0.0,
        "cmd_vertical":            0.0,
        "stabilize":               False,
        "depth_hold_request":      False,
        "depth_hold_active":       False,
        "yaw_hold_request":        False,
        "yaw_hold_active":         False,
        "depth_m":                 None,
        "hold_depth_m":            None,
        "yaw_deg":                 None,
        "hold_yaw_deg":            None,
        "roll_deg":                None,
        "pitch_deg":               None,
        "h_group":                 0.0,
        "v_group":                 0.0,
        "pressure_hpa":            None,
        "temperature_c":           None,
        "battery_voltage_v":       None,
        "battery_current_a":       None,
        "battery_remaining_pct":   None,
        "battery_consumed_mah":    None,
        "control_timeout":         False,
        "attitude_stale":          False,
        "depth_stale":             False,
        "mavlink_link_dead":       False,
        "mavlink_last_rx_age_sec": None,
        "attitude_age_sec":        None,
        "depth_recapture_pending": False,
        "yaw_recapture_pending":   False,
    },
    "logs": {
        "thrust":       [],
        "arm":          [],
        "onboard_stab": [],
        "onboard_arm":  [],
        "onboard_cam":  [],
        "colmap":       [],
        "crabs":        [],
    },
}

_state_lock = threading.Lock()
_telemetry_record_lock = threading.Lock()
_telemetry_rate_counter = {"count": 0, "window_start": time.time()}
MAX_LOG_LINES = 200
LOGS_DIR = ROV_ROOT / "logs"

# ─────────────────────────────────────────────────────────────────────────────
# SSH MANAGER
# ─────────────────────────────────────────────────────────────────────────────

class SSHManager:
    def __init__(self):
        self._client = None
        self._lock = threading.Lock()

    def connect(self, host, user, password, port=22, connect_timeout=20):
        if not HAVE_PARAMIKO:
            return False, "paramiko not installed. Run: pip install paramiko"
        try:
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect(
                host, port=port, username=user, password=password,
                timeout=connect_timeout,
                banner_timeout=max(connect_timeout, 15),
                auth_timeout=max(connect_timeout, 15),
            )
            # Keepalive every 5 s — catches a dead link quickly so the
            # monitor loop notices before exec() blocks on a stale channel.
            t = client.get_transport()
            if t:
                t.set_keepalive(5)
            with self._lock:
                if self._client:
                    try: self._client.close()
                    except: pass
                self._client = client
            return True, "Connected"
        except (TimeoutError, socket.timeout):
            return False, (
                f"Timed out connecting to {host}:{port} — "
                "is the Pi on and reachable? Check the IP address."
            )
        except paramiko.AuthenticationException:
            return False, f"Authentication failed for {user}@{host} — check username/password."
        except paramiko.SSHException as e:
            return False, f"SSH error: {e}"
        except OSError as e:
            # Covers ConnectionRefusedError, NetworkUnreachable, etc.
            return False, f"Network error reaching {host}:{port} — {e}"
        except Exception as e:
            return False, str(e)

    def disconnect(self):
        with self._lock:
            if self._client:
                try: self._client.close()
                except: pass
                self._client = None

    def is_connected(self):
        with self._lock:
            if self._client is None:
                return False
            try:
                t = self._client.get_transport()
                return t is not None and t.is_active()
            except:
                return False

    def _invalidate_client(self):
        if self._client:
            try:
                self._client.close()
            except Exception:
                pass
            self._client = None

    def exec(self, cmd, timeout=20):
        # Phase 1: acquire lock only long enough to open the exec channel.
        # Releasing the lock before stdout.read() lets is_connected() and
        # other threads proceed while we wait for the command output —
        # a dead connection won't freeze the entire SSH lock.
        with self._lock:
            if self._client is None:
                return "", "", "Not connected"
            try:
                transport = self._client.get_transport()
                if transport is None or not transport.is_active():
                    self._invalidate_client()
                    return "", "", "SSH session not active"
                _, stdout, stderr = self._client.exec_command(cmd, timeout=timeout)
            except Exception as e:
                self._invalidate_client()
                return "", "", str(e)
        # Phase 2: read output outside the lock (may block up to `timeout` s).
        try:
            out = stdout.read().decode("utf-8", errors="replace")
            err = stderr.read().decode("utf-8", errors="replace")
            return out.strip(), err.strip(), None
        except Exception as e:
            with self._lock:
                self._invalidate_client()
            return "", "", str(e)

    def _supervisor_cmd(self, *args, timeout=90):
        """Run onboard/supervisor.py on the Pi; return (parsed_json, error_msg)."""
        rov_path = shlex.quote(config["pi_rov_path"])
        cmd = (
            f"cd {rov_path} && python3 onboard/supervisor.py "
            + " ".join(shlex.quote(str(a)) for a in args)
        )
        out, err, error = self.exec(cmd, timeout=timeout)
        if error:
            return None, error
        blob = (out or "") + "\n" + (err or "")
        for line in reversed(blob.splitlines()):
            line = line.strip()
            if line.startswith("{"):
                try:
                    return json.loads(line), None
                except json.JSONDecodeError:
                    continue
        return None, (err or out or "supervisor returned no JSON").strip()[:240]

    def supervisor_start_and_wait(
        self, name: str, timeout_sec: float = 50.0, extra_args: str = ""
    ) -> tuple[bool, str]:
        """Start one onboard service and block until log-ready or timeout."""
        start_args = ["start", name]
        if extra_args.strip():
            start_args.extend(["--extra-args", extra_args.strip()])
        data, err = self._supervisor_cmd(*start_args, timeout=30)
        if err:
            return False, f"start failed: {err}"

        pid = (data or {}).get("pid", "?")
        data, err = self._supervisor_cmd(
            "wait", name, "--timeout", str(int(timeout_sec)),
            timeout=timeout_sec + 25,
        )
        if err:
            return False, f"wait failed: {err}"
        if data and data.get("ok"):
            return True, f"PID {data.get('pid', pid)} ready"

        tail = (data or {}).get("log_tail", "")
        detail = (data or {}).get("error", "not ready")
        if tail:
            last = tail.strip().splitlines()[-1][:120]
            return False, f"{detail} | Log: {last}"
        return False, detail

    def supervisor_stop_all(self):
        self._supervisor_cmd("stop", "all", timeout=20)

    def supervisor_status(self) -> dict:
        data, err = self._supervisor_cmd("status", timeout=15)
        if err or not data:
            return {}
        return data

    def is_onboard_running(self, script_name: str) -> bool:
        key = {
            "stabilization.py": "stab",
            "new_ar.py": "arm",
            "camera_stream.py": "cam",
        }.get(script_name, "")
        if not key:
            return False
        st = self.supervisor_status().get(key, {})
        return bool(st.get("alive"))

    def stop_onboard_process(self, script_name):
        key = {
            "stabilization.py": "stab",
            "new_ar.py": "arm",
            "camera_stream.py": "cam",
        }.get(script_name)
        if key:
            self._supervisor_cmd("stop", key, timeout=15)

    def start_onboard_process(self, script_rel, log_name, extra_args=""):
        """Legacy wrapper — prefer supervisor_start_and_wait()."""
        ok, msg = self.supervisor_start_and_wait(log_name, 50.0, extra_args)
        return ok, msg

    def get_onboard_log(self, log_name, lines=20):
        log_file = f"/tmp/rov_{log_name}.log"
        out, _, _ = self.exec(f"tail -n {lines} {log_file} 2>/dev/null || echo ''")
        return out

    def send_mosfet(self, state: bool):
        payload = json.dumps({"cmd": "mosfet", "state": state}).encode()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(1.0)
            s.sendto(payload, (config["pi_ip"], config["mosfet_control_port"]))
            s.close()
            return True, "sent"
        except Exception as e:
            return False, str(e)

    def _start_mavproxy_fresh(self):
        """Kill any existing MAVProxy and launch a fresh bridge."""
        self.exec("pkill -f mavproxy 2>/dev/null; pkill -f MAVProxy 2>/dev/null; sleep 0.5")
        normalize_onboard_config()
        bin_  = config["mavproxy_bin"]
        ser   = config["mavproxy_serial"]
        baud  = config["mavproxy_baud"]
        out1  = config["mavproxy_out1"]
        out2  = config["mavproxy_out2"]
        cmd = (
            f"setsid nohup {bin_} "
            f"--master={ser} "
            f"--baudrate {baud} "
            f"--non-interactive "
            f"--out={out1} "
            f"--out={out2} "
            f"< /dev/null > /tmp/rov_mavproxy.log 2>&1 & echo $!"
        )
        out, _, error = self.exec(cmd, timeout=10)
        if error:
            return False, error
        pid = out.strip()
        return True, f"MAVProxy started (PID {pid})"

    def ensure_mavproxy(self):
        """Start MAVProxy only if not already healthy — avoids killing a working bridge."""
        normalize_onboard_config()
        if (
            self.is_mavproxy_running()
            and self.is_mavproxy_fc_connected()
            and self.is_mavproxy_tcp_ready()
        ):
            return True, "MAVProxy already running — Pix6 online"
        return self._start_mavproxy_fresh()

    def start_mavproxy(self):
        """Legacy name — always forces a fresh MAVProxy."""
        return self._start_mavproxy_fresh()

    def is_mavproxy_tcp_ready(self):
        """True when MAVProxy is listening for onboard script TCP connections."""
        port = MAVPROXY_TCP_PORT
        out, _, _ = self.exec(
            f"(ss -tln 2>/dev/null || netstat -tln 2>/dev/null) | grep -q ':{port} ' "
            f"&& echo ok"
        )
        return "ok" in out

    def wait_mavproxy_tcp_ready(self, timeout_sec: float = 20.0) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self.is_mavproxy_tcp_ready():
                return True
            time.sleep(1.0)
        return False

    # Log substrings that mean the FC link is actively passing MAVLink traffic.
    _MAVPROXY_ALIVE_MARKERS = (
        "detected vehicle", "online system", "got command_ack",
        "vcc ", "ap:", "flight battery", "heartbeat", "fence present",
        "manual>", "received ", "saved ", "parameters",
    )

    def is_mavproxy_fc_connected(self):
        """True when MAVProxy log shows live FC traffic (not just one-time startup lines)."""
        if not self.is_mavproxy_running():
            return False
        if self.mavproxy_recent_no_link():
            return False

        out, _, _ = self.exec(
            "tail -n 30 /tmp/rov_mavproxy.log 2>/dev/null || true"
        )
        lines = [ln.strip() for ln in out.strip().splitlines() if ln.strip()]
        if not lines:
            return False

        recent = lines[-15:]
        recent_text = "\n".join(recent).lower()
        if "unloading module" in recent_text:
            return False

        trailing_no_link = 0
        for ln in reversed(recent):
            lower = ln.lower()
            if "no link" in lower or "link down" in lower:
                trailing_no_link += 1
            else:
                break
        if trailing_no_link >= 3:
            return False

        # Startup strings scroll out once stabilization requests message rates;
        # any recent FC traffic (COMMAND_ACK, Vcc, AP:, etc.) means link is up.
        return any(m in recent_text for m in self._MAVPROXY_ALIVE_MARKERS)

    def _device_exists(self, path: str) -> bool:
        out, _, _ = self.exec(f"test -e {shlex.quote(path)} && echo exists")
        return "exists" in out

    def mavproxy_serial_candidates(self):
        """Ordered serial ports to try: configured first, then other ttyACM/ttyUSB."""
        preferred = config.get("mavproxy_serial", "/dev/ttyACM0")
        alts = sorted(self.list_serial_candidates())
        ordered = []
        if self._device_exists(preferred):
            ordered.append(preferred)
        for dev in alts:
            if dev not in ordered:
                ordered.append(dev)
        return ordered

    def serial_port_exists(self):
        """True when the configured Pix6 serial device node is present."""
        ser = config.get("mavproxy_serial", "/dev/ttyACM0")
        return self._device_exists(ser)

    def any_serial_port_exists(self):
        return bool(self.mavproxy_serial_candidates())

    def list_serial_candidates(self):
        """Return ttyACM/ttyUSB device paths visible on the Pi."""
        out, _, _ = self.exec(
            "ls -1 /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true"
        )
        return [ln.strip() for ln in out.strip().splitlines() if ln.strip()]

    def wait_for_mavproxy_fc(self, on_wait=None) -> tuple[bool, str]:
        """Wait until MAVProxy reports the FC online, or fail fast on no link."""
        time.sleep(2.0)
        fc_deadline = time.time() + 45.0
        fc_wait_i = 0
        miss_running = 0
        ser = config.get("mavproxy_serial", "/dev/ttyACM0")

        while time.time() < fc_deadline:
            if self.is_mavproxy_running():
                miss_running = 0
            else:
                miss_running += 1
                if miss_running >= 3:
                    return False, "MAVProxy exited — check /tmp/rov_mavproxy.log on Pi"

            if self.is_mavproxy_fc_connected():
                return True, f"MAVProxy running — Pix6 online on {ser}"

            fc_wait_i += 1
            diag = self.mavproxy_diagnosis() if fc_wait_i >= 2 else ""
            wait_msg = f"Waiting for Pix6 on {ser}... ({fc_wait_i})"
            if diag:
                wait_msg += f" — {diag}"

            if fc_wait_i >= 4 and not self._device_exists(ser):
                return False, f"USB port {ser} disappeared — {self.mavproxy_diagnosis()}"
            if fc_wait_i >= 6 and self.mavproxy_recent_no_link():
                return False, f"No FC on {ser} — {self.mavproxy_diagnosis()}"

            if on_wait:
                on_wait(wait_msg)
            time.sleep(2.0)

        return False, (
            "MAVProxy running but Pix6 not detected — "
            + self.mavproxy_diagnosis()
        )

    def mavproxy_recent_no_link(self):
        """True when the last few MAVProxy log lines are all 'no link'."""
        out, _, _ = self.exec(
            "tail -n 5 /tmp/rov_mavproxy.log 2>/dev/null || true"
        )
        lines = [ln.strip() for ln in out.strip().splitlines() if ln.strip()]
        if len(lines) < 3:
            return False
        return all(
            "no link" in ln.lower() or "link down" in ln.lower()
            for ln in lines[-3:]
        )

    def mavproxy_diagnosis(self):
        """Human-readable summary when MAVProxy has no FC link."""
        ser = config.get("mavproxy_serial", "/dev/ttyACM0")
        parts = []
        if not self.serial_port_exists():
            parts.append(f"{ser} not found")
            alts = self.list_serial_candidates()
            if alts:
                parts.append(f"available: {', '.join(alts)}")
            else:
                parts.append("no /dev/ttyACM* or /dev/ttyUSB* devices")
        else:
            parts.append(f"{ser} exists but MAVProxy reports no link")
        log_tail = self.get_mavproxy_log(lines=3)
        if log_tail:
            parts.append(f"log: {log_tail.strip().splitlines()[-1][:80]}")
        return " — ".join(parts)

    def stop_mavproxy(self):
        self.exec("pkill -f mavproxy 2>/dev/null; pkill -f MAVProxy 2>/dev/null || true")

    def is_mavproxy_running(self):
        # pgrep -a shows full command line; -f matches against it.
        # Returns True if any mavproxy process is alive.
        out, _, error = self.exec("pgrep -f 'mavproxy'")
        if error:
            return False
        return bool(out.strip())

    def get_onboard_status(self):
        """Check onboard processes via the Pi-side supervisor."""
        st = self.supervisor_status()
        if not st:
            return {"mavproxy": False, "stab": False, "arm": False, "cam": False}
        return {
            "mavproxy": self.is_mavproxy_running() and self.is_mavproxy_fc_connected(),
            "stab":     bool(st.get("stab", {}).get("alive")),
            "arm":      bool(st.get("arm", {}).get("alive")),
            "cam":      bool(st.get("cam", {}).get("alive")),
        }

    def get_mavproxy_log(self, lines=10):
        out, _, _ = self.exec(f"tail -n {lines} /tmp/rov_mavproxy.log 2>/dev/null || echo ''")
        return out

    def run_colmap(self):
        rov_path = config["pi_rov_path"]
        cmd = config["colmap_command"]
        full_cmd = f"cd {rov_path} && nohup {cmd} > /tmp/rov_colmap.log 2>&1 &"
        _, _, error = self.exec(full_cmd)
        return error is None, error or "started"

    def run_crabs(self):
        rov_path = config["pi_rov_path"]
        cmd = config["crabs_command"]
        full_cmd = f"cd {rov_path} && nohup {cmd} > /tmp/rov_crabs.log 2>&1 &"
        _, _, error = self.exec(full_cmd)
        return error is None, error or "started"

    def sync_onboard_files(self):
        """Upload all onboard/*.py files from the local project to the Pi via SFTP.

        Opens a dedicated SFTP channel on the existing SSH transport without
        holding the exec lock, so normal SSH commands can continue concurrently.
        """
        with self._lock:
            if self._client is None:
                return False, "Not connected"
            try:
                sftp = self._client.open_sftp()
            except Exception as e:
                return False, f"SFTP channel failed: {e}"

        remote_onboard = f"{config['pi_rov_path']}/onboard"
        local_onboard  = ROV_ROOT / "onboard"

        uploaded, errors = [], []
        try:
            try:
                sftp.stat(remote_onboard)
            except FileNotFoundError:
                sftp.mkdir(remote_onboard)

            for local_file in sorted(local_onboard.glob("*.py")):
                remote_path = f"{remote_onboard}/{local_file.name}"
                try:
                    sftp.put(str(local_file), remote_path)
                    uploaded.append(local_file.name)
                except Exception as e:
                    errors.append(f"{local_file.name}: {e}")
        except Exception as e:
            errors.append(f"Sync error: {e}")
        finally:
            try:
                sftp.close()
            except Exception:
                pass

        if errors:
            return False, "Upload errors — " + "; ".join(errors)
        return True, f"Synced {len(uploaded)} file(s): {', '.join(uploaded)}"


ssh = SSHManager()

# ─────────────────────────────────────────────────────────────────────────────
# LOCAL PROCESS MANAGEMENT
# ─────────────────────────────────────────────────────────────────────────────

_procs: dict[str, subprocess.Popen] = {}
_procs_lock = threading.Lock()


def _topside_dir():
    return ROV_ROOT / "topside"


def start_local_process(name: str, cmd: list[str], cwd=None, env_extra=None):
    with _procs_lock:
        proc = _procs.get(name)
        if proc and proc.poll() is None:
            return True, "already running"

        env = os.environ.copy()
        env.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        if env_extra:
            env.update(env_extra)

        try:
            kwargs = dict(
                cwd=str(cwd or ROV_ROOT),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            if IS_WINDOWS:
                kwargs["creationflags"] = subprocess.CREATE_NO_WINDOW
            proc = subprocess.Popen(cmd, **kwargs)
            _procs[name] = proc
        except Exception as e:
            return False, str(e)

    threading.Thread(
        target=_drain_process_output, args=(name, proc), daemon=True
    ).start()
    return True, "started"


def stop_local_process(name: str):
    with _procs_lock:
        proc = _procs.pop(name, None)
    if proc and proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            proc.kill()


def is_local_running(name: str) -> bool:
    with _procs_lock:
        proc = _procs.get(name)
        return proc is not None and proc.poll() is None


def _drain_process_output(name: str, proc: subprocess.Popen):
    """Read stdout from a local process and emit logs."""
    for raw_line in proc.stdout:
        line = raw_line.rstrip()
        if not line:
            continue

        with _state_lock:
            log_list = STATE["logs"].get(name, [])
            log_list.append(line)
            if len(log_list) > MAX_LOG_LINES:
                del log_list[:-MAX_LOG_LINES]
            if name == "arm":
                parsed = _parse_arm_sent_line(line)
                if parsed:
                    STATE["arm_last_pwm"] = parsed

        socketio.emit("process_log", {"name": name, "line": line})

    with _state_lock:
        if name == "thrust":
            STATE["thrust_running"] = False
        elif name == "arm":
            STATE["arm_running"] = False

    emit_status()


# ─────────────────────────────────────────────────────────────────────────────
# TELEMETRY RECEIVER — parses JSON directly from stabilization.py
# ─────────────────────────────────────────────────────────────────────────────

def _forward_ctrl_to_pi(packet: dict):
    """Send a control JSON packet to stabilization.py on the Pi."""
    ip = config["pi_ip"]
    port = int(config["thrust_udp_port"])
    try:
        _pi_ctrl_sock.sendto(json.dumps(packet).encode("utf-8"), (ip, port))
    except Exception as e:
        print(f"[WARN] Control UDP send failed: {e}")


def _get_active_ctrl_packet() -> dict:
    """Pick the best control packet to send: live browser input or neutral keepalive."""
    global _ctrl_keepalive_seq
    with _ctrl_lock:
        recent = (time.time() - _last_browser_ctrl_time) < 1.0
        if recent and _last_browser_ctrl:
            return dict(_last_browser_ctrl)

    with _ctrl_lock:
        _ctrl_keepalive_seq += 1
        seq = _ctrl_keepalive_seq

    mode = STATE.get("mode", "disarmed")
    armed = mode in ("armed", "stabilize")
    return {
        "seq": seq,
        "time": time.time(),
        "forward": 0.0,
        "lateral": 0.0,
        "yaw": 0.0,
        "vertical": 0.0,
        "stabilize": armed and mode == "stabilize",
        "depth_hold": False,
        "yaw_hold": False,
        "gain_percent": STATE["telemetry"].get("gain_percent", 100),
        "telemetry_port": int(config["telemetry_port"]),
    }


def _make_keepalive_packet() -> dict:
    """Backward-compatible alias."""
    return _get_active_ctrl_packet()


def _start_control_keepalive():
    """Send control UDP at 20 Hz so Pi always has a telemetry return address."""
    def _loop():
        while True:
            time.sleep(0.05)
            _forward_ctrl_to_pi(_get_active_ctrl_packet())

    threading.Thread(target=_loop, daemon=True, name="ctrl-keepalive").start()


def _emit_onboard_progress(step: str, status: str, msg: str = ""):
    """Record onboard start progress and push to all connected browsers."""
    entry = {"step": step, "status": status, "msg": msg, "time": time.time()}
    with _state_lock:
        STATE["onboard_progress"].append(entry)
        if len(STATE["onboard_progress"]) > 50:
            STATE["onboard_progress"] = STATE["onboard_progress"][-50:]
        if step == "complete":
            STATE["onboard_starting"] = False

    socketio.emit("onboard_progress", entry)


def _wait_onboard_running(check_fn, label: str, timeout_sec: float = 12.0) -> tuple[bool, str]:
    """Poll until an onboard process is running or timeout.
    Polls every 2 s so we don't flood the SSH connection while
    the monitor loop is also making SSH calls every second.
    While SSH is disconnected the countdown is paused — disconnected time
    does not count against the budget, since the process may be running fine
    (it's nohup'd) and we just can't reach the Pi temporarily."""
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if not ssh.is_connected():
            time.sleep(2.0)
            deadline += 2.0  # don't count time we can't even poll against timeout
            continue
        if check_fn():
            return True, f"{label} running"
        time.sleep(2.0)
    return False, f"{label} did not start within {int(timeout_sec)}s — check onboard logs"


def _update_telemetry_from_json(pkt: dict):
    """Map stabilization.py JSON telemetry → UI state and emit to browser."""
    tel = STATE["telemetry"]
    tel["rx_state"]                = pkt.get("state", "OK")
    tel["gain_percent"]            = pkt.get("gain_percent", tel["gain_percent"])
    tel["stabilize"]               = bool(pkt.get("stabilize", False))
    tel["depth_hold_request"]      = bool(pkt.get("depth_hold_request", False))
    tel["depth_hold_active"]       = bool(pkt.get("depth_hold_active", False))
    tel["yaw_hold_request"]        = bool(pkt.get("yaw_hold_request", False))
    tel["yaw_hold_active"]         = bool(pkt.get("yaw_hold_active", False))
    tel["depth_m"]                 = pkt.get("depth_m")
    tel["hold_depth_m"]            = pkt.get("hold_depth_m")
    tel["yaw_deg"]                 = pkt.get("yaw_deg")
    tel["hold_yaw_deg"]            = pkt.get("hold_yaw_deg")
    tel["roll_deg"]                = pkt.get("roll_deg")
    tel["pitch_deg"]               = pkt.get("pitch_deg")
    tel["h_group"]                 = float(pkt.get("horizontal_group", 0.0))
    tel["v_group"]                 = float(pkt.get("vertical_group", 0.0))
    tel["pressure_hpa"]            = pkt.get("pressure_hpa")
    tel["temperature_c"]           = pkt.get("pressure_temperature_c")
    tel["battery_voltage_v"]       = pkt.get("battery_voltage_v")
    tel["battery_current_a"]       = pkt.get("battery_current_a")
    tel["battery_remaining_pct"]   = pkt.get("battery_remaining_pct")
    tel["battery_consumed_mah"]    = pkt.get("battery_consumed_mah")
    tel["control_timeout"]         = bool(pkt.get("control_timeout", False))
    tel["attitude_stale"]          = bool(pkt.get("attitude_stale", False))
    tel["depth_stale"]             = bool(pkt.get("depth_stale", False))
    tel["mavlink_link_dead"]       = bool(pkt.get("mavlink_link_dead", False))
    tel["mavlink_last_rx_age_sec"] = pkt.get("mavlink_last_rx_age_sec")
    tel["attitude_age_sec"]        = pkt.get("attitude_age_sec")
    tel["depth_recapture_pending"] = bool(pkt.get("depth_recapture_pending", False))
    tel["yaw_recapture_pending"]   = bool(pkt.get("yaw_recapture_pending", False))

    now = time.time()
    STATE["last_telemetry_time"] = now
    STATE["telemetry_packets"]   = STATE.get("telemetry_packets", 0) + 1

    _telemetry_rate_counter["count"] += 1
    elapsed = now - _telemetry_rate_counter["window_start"]
    if elapsed >= 1.0:
        STATE["telemetry_rate_hz"] = round(_telemetry_rate_counter["count"] / elapsed, 1)
        _telemetry_rate_counter["count"] = 0
        _telemetry_rate_counter["window_start"] = now

    _append_telemetry_record(pkt)
    payload = dict(tel)
    payload["link_health"] = _compute_link_health()
    socketio.emit("telemetry", payload)


def _append_telemetry_record(pkt: dict):
    """Append one telemetry row to the active CSV black-box file."""
    if not STATE.get("telemetry_recording"):
        return
    path = STATE.get("telemetry_record_file")
    if not path:
        return
    record = dict(pkt)
    record.setdefault("gain_percent", STATE["telemetry"].get("gain_percent"))
    row = []
    for key in TELEMETRY_CSV_FIELDS:
        val = record.get(key)
        if val is None and key == "pressure_temperature_c":
            val = record.get("pressure_temperature_c")
        if val is None:
            row.append("")
        elif isinstance(val, bool):
            row.append("1" if val else "0")
        else:
            row.append(str(val))
    line = ",".join(row) + "\n"
    try:
        with _telemetry_record_lock:
            with open(path, "a", encoding="utf-8") as f:
                f.write(line)
    except Exception as e:
        print(f"[WARN] Telemetry record write failed: {e}")


def _compute_link_health() -> dict:
    """Composite link health from telemetry + control path."""
    tel = STATE["telemetry"]
    tel_age = time.time() - STATE["last_telemetry_time"]
    ctrl_age = time.time() - STATE.get("last_ctrl_time", 0.0)

    level = "ok"
    detail_parts = []

    if tel_age > 2.0 or tel.get("rx_state") == "NO_TELEMETRY":
        level = "err"
        detail_parts.append(f"telem {tel_age:.1f}s")
    elif tel.get("control_timeout") or tel.get("rx_state") not in ("OK", None):
        level = "warn" if level == "ok" else level
        detail_parts.append(tel.get("rx_state", "fault"))

    if tel.get("mavlink_link_dead"):
        level = "err"
        detail_parts.append("mavlink dead")
    elif tel.get("attitude_stale") or tel.get("depth_stale"):
        level = "warn" if level == "ok" else level
        if tel.get("attitude_stale"):
            detail_parts.append("IMU stale")
        if tel.get("depth_stale"):
            detail_parts.append("depth stale")

    if ctrl_age > 1.5:
        level = "warn" if level == "ok" else level
        detail_parts.append(f"ctrl {ctrl_age:.1f}s")

    return {
        "level": level,
        "detail": " · ".join(detail_parts) if detail_parts else "OK",
        "telemetry_age_sec": round(tel_age, 2),
        "ctrl_age_sec": round(ctrl_age, 2),
        "telemetry_rate_hz": STATE.get("telemetry_rate_hz", 0.0),
    }


def _start_telemetry_listener():
    """Listen for JSON telemetry from stabilization.py on telemetry_port (UDP)."""
    def _listen():
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("0.0.0.0", int(config["telemetry_port"])))
            s.settimeout(1.0)
            STATE["telemetry_listener_ok"] = True
            emit_status()
        except Exception as e:
            STATE["telemetry_listener_ok"] = False
            emit_status()
            print(
                f"[ERROR] Telemetry listener bind failed on port {config['telemetry_port']}: {e}\n"
                f"        Stop thrust_sender.py or any other program using UDP {config['telemetry_port']}."
            )
            return

        print(f"[INFO] Telemetry listener active on UDP port {config['telemetry_port']}")
        while True:
            try:
                data, _ = s.recvfrom(8192)
                try:
                    pkt = json.loads(data.decode("utf-8"))
                    _update_telemetry_from_json(pkt)
                except Exception:
                    pass
            except socket.timeout:
                pass
            except Exception:
                pass

    threading.Thread(target=_listen, daemon=True).start()


# ─────────────────────────────────────────────────────────────────────────────
# BACKGROUND MONITOR
# ─────────────────────────────────────────────────────────────────────────────

_ssh_monitor_counter  = 0      # only query SSH process status every 5 s
_ssh_was_connected    = False  # True once user has successfully connected
_ssh_reconnect_active = False  # guard: only one auto-reconnect attempt at a time


def _trigger_ssh_reconnect():
    """Spawn a background thread that attempts one quick SSH reconnect (5 s
    timeout).  Only fires if the user previously had a working connection and
    no reconnect is already in flight."""
    global _ssh_reconnect_active
    if _ssh_reconnect_active:
        return
    if not config.get("pi_ip") or not config.get("pi_user"):
        return
    _ssh_reconnect_active = True

    def _do():
        global _ssh_reconnect_active
        try:
            ok, _ = ssh.connect(
                config["pi_ip"], config["pi_user"], config["pi_password"],
                port=int(config["pi_ssh_port"]),
                connect_timeout=5,
            )
            if ok:
                STATE["ssh_connected"] = True
                STATE["ssh_error"]     = ""
                emit_status()
        finally:
            _ssh_reconnect_active = False

    threading.Thread(target=_do, daemon=True, name="ssh-auto-reconnect").start()


def _monitor_loop():
    global _ssh_monitor_counter, _ssh_was_connected
    while True:
        time.sleep(1.0)
        STATE["thrust_running"] = is_local_running("thrust")
        STATE["arm_running"]    = is_local_running("arm")

        _ssh_monitor_counter += 1
        # Skip heavy SSH polling while the startup sequence is running —
        # it and _wait_onboard_running are already making SSH calls and
        # competing for the channel can drop the connection.
        # Between startup sequences, check every 5 s via a single batched exec.
        if not STATE["onboard_starting"] and _ssh_monitor_counter >= 5:
            _ssh_monitor_counter = 0
            if ssh.is_connected():
                _ssh_was_connected            = True
                STATE["ssh_connected"]         = True
                status = ssh.get_onboard_status()
                STATE["onboard_mavproxy"] = status["mavproxy"]
                STATE["onboard_stab"]     = status["stab"]
                STATE["onboard_arm"]      = status["arm"]
                STATE["onboard_cam"]      = status["cam"]
            else:
                STATE["ssh_connected"]    = False
                STATE["onboard_mavproxy"] = False
                STATE["onboard_stab"]     = False
                STATE["onboard_arm"]      = False
                STATE["onboard_cam"]      = False
                # Auto-reconnect only if the user had a working session before.
                if _ssh_was_connected:
                    _trigger_ssh_reconnect()

        tel_age = time.time() - STATE["last_telemetry_time"]
        if tel_age > 2.0:
            STATE["telemetry"]["rx_state"] = "NO_TELEMETRY"

        emit_status()


def emit_status():
    with _state_lock:
        progress = list(STATE["onboard_progress"])
    link = _compute_link_health()
    socketio.emit("status", {
        "thrust_running":        STATE["thrust_running"],
        "arm_running":           STATE["arm_running"],
        "onboard_stab":          STATE["onboard_stab"],
        "onboard_arm":           STATE["onboard_arm"],
        "onboard_cam":           STATE["onboard_cam"],
        "onboard_mavproxy":      STATE["onboard_mavproxy"],
        "ssh_connected":         STATE["ssh_connected"],
        "ssh_error":             STATE["ssh_error"],
        "mode":                  STATE["mode"],
        "mosfet_on":             STATE["mosfet_on"],
        "telemetry_listener_ok": STATE["telemetry_listener_ok"],
        "onboard_starting":      STATE["onboard_starting"],
        "onboard_progress":      progress,
        "telemetry_recording":   STATE["telemetry_recording"],
        "telemetry_record_file": STATE.get("telemetry_record_file", ""),
        "link_health":           link,
        "arm_last_pwm":          STATE.get("arm_last_pwm"),
    })


# ─────────────────────────────────────────────────────────────────────────────
# CAMERA PROXY
# ─────────────────────────────────────────────────────────────────────────────

# UI slot 1 = forward, slot 2 = arm
_CAMERA_UI_URL_KEY = {1: "forward_camera_url", 2: "arm_camera_url"}

@app.route("/camera/<int:cam_num>")
def camera_stream(cam_num):
    if cam_num not in (1, 2):
        return "", 404

    url_key = _CAMERA_UI_URL_KEY.get(cam_num, f"camera{cam_num}_url")
    cam_url = config.get(url_key, "")

    def _gen():
        if not HAVE_REQUESTS or not cam_url:
            return
        while True:
            try:
                r = _requests.get(cam_url, stream=True, timeout=(5, 30))
                r.raise_for_status()
                for chunk in r.iter_content(chunk_size=8192):
                    if chunk:
                        yield chunk
            except Exception:
                time.sleep(3)
                continue

    content_type = "multipart/x-mixed-replace; boundary=frame"
    if HAVE_REQUESTS and cam_url:
        try:
            head_r = _requests.head(cam_url, timeout=2)
            content_type = head_r.headers.get("Content-Type", content_type)
        except Exception:
            pass

    return Response(stream_with_context(_gen()), mimetype=content_type)


@app.route("/camera/<int:cam_num>/snapshot")
def camera_snapshot(cam_num):
    if cam_num not in (1, 2):
        return "", 404
    if not HAVE_REQUESTS:
        return jsonify({"ok": False, "msg": "requests not installed"}), 503

    url_key = _CAMERA_UI_URL_KEY.get(cam_num, f"camera{cam_num}_url")
    base = str(config.get(url_key, "")).rstrip("/")
    if not base:
        return "", 404

    snap_url = f"{base}/snapshot"
    try:
        r = _requests.get(snap_url, timeout=5)
        r.raise_for_status()
        ctype = r.headers.get("Content-Type", "image/jpeg")
        return Response(r.content, mimetype=ctype)
    except Exception as e:
        return jsonify({"ok": False, "msg": str(e)}), 502


# ─────────────────────────────────────────────────────────────────────────────
# FLASK API ROUTES
# ─────────────────────────────────────────────────────────────────────────────

@app.route("/favicon.ico")
def favicon():
    return Response(status=204)


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/config", methods=["GET", "POST"])
def api_config():
    global config
    if request.method == "POST":
        data = request.get_json(force=True) or {}
        if "camera1_url" in data:
            data["arm_camera_url"] = data["camera1_url"]
        if "camera2_url" in data:
            data["forward_camera_url"] = data["camera2_url"]
        for k, v in data.items():
            if k in config:
                config[k] = v
        normalize_onboard_config()
        save_config_file()
        return jsonify({"ok": True, "config": config})
    normalize_onboard_config()
    return jsonify(config)


@app.route("/api/ssh/connect", methods=["POST"])
def api_ssh_connect():
    data = request.get_json(force=True) or {}
    for k in ("pi_ip", "pi_user", "pi_password", "pi_ssh_port", "pi_rov_path"):
        if k in data:
            config[k] = data[k]

    ok, msg = ssh.connect(
        config["pi_ip"], config["pi_user"], config["pi_password"],
        port=int(config["pi_ssh_port"])
    )
    STATE["ssh_connected"] = ok
    STATE["ssh_error"] = "" if ok else msg
    emit_status()
    return jsonify({"ok": ok, "msg": msg})


@app.route("/api/ssh/disconnect", methods=["POST"])
def api_ssh_disconnect():
    global _ssh_was_connected
    _ssh_was_connected = False   # don't auto-reconnect after deliberate disconnect
    ssh.disconnect()
    STATE["ssh_connected"] = False
    STATE["ssh_error"] = ""
    emit_status()
    return jsonify({"ok": True})


@app.route("/api/onboard/start", methods=["POST"])
def api_start_onboard():
    if not ssh.is_connected():
        return jsonify({"ok": False, "msg": "SSH not connected"})

    if STATE["onboard_starting"]:
        return jsonify({
            "ok": True,
            "in_progress": True,
            "msg": "Onboard start already in progress — see progress log below",
        })

    STATE["onboard_starting"] = True
    STATE["onboard_progress"] = []
    emit_status()

    def _do_start():
        try:
            # Step 0: push local onboard/*.py to the Pi so edits take effect immediately.
            _emit_onboard_progress("sync", "starting", "Uploading onboard scripts to Pi...")
            ok_sync, msg_sync = ssh.sync_onboard_files()
            _emit_onboard_progress("sync", "done" if ok_sync else "error", msg_sync)

            # Step 1: MAVProxy (try configured port, then auto-detect ttyACM/ttyUSB)
            normalize_onboard_config()
            preferred_serial = config.get("mavproxy_serial", "/dev/ttyACM0")
            serial_candidates = ssh.mavproxy_serial_candidates()
            if not serial_candidates:
                diag = ssh.mavproxy_diagnosis()
                _emit_onboard_progress("mavproxy", "error", diag)
                _emit_onboard_progress(
                    "complete", "error",
                    f"✕ Pix6 USB missing — {diag}",
                )
                emit_status()
                return

            ok_m = False
            msg_m = ""
            for attempt, ser in enumerate(serial_candidates):
                config["mavproxy_serial"] = ser
                auto_note = ""
                if ser != preferred_serial:
                    auto_note = f" (auto-selected; {preferred_serial} not found)"

                if attempt == 0:
                    _emit_onboard_progress(
                        "mavproxy", "starting",
                        f"Launching MAVProxy on {ser}{auto_note}...",
                    )
                    ok_m, msg_m = ssh.ensure_mavproxy()
                    fresh_mav = ok_m and "already running" not in msg_m
                    if ok_m and fresh_mav:
                        ok_m, msg_m = _wait_onboard_running(
                            ssh.is_mavproxy_running, "MAVProxy", timeout_sec=30.0
                        )
                else:
                    _emit_onboard_progress(
                        "mavproxy", "wait",
                        f"No FC on previous port — trying {ser}...",
                    )
                    ok_m, msg_m = ssh._start_mavproxy_fresh()
                    if ok_m:
                        ok_m, msg_m = _wait_onboard_running(
                            ssh.is_mavproxy_running, "MAVProxy", timeout_sec=30.0
                        )

                if not ok_m:
                    break

                fc_ok, fc_msg = ssh.wait_for_mavproxy_fc(
                    on_wait=lambda w: _emit_onboard_progress("mavproxy", "wait", w),
                )
                ok_m = fc_ok
                msg_m = fc_msg
                if ok_m:
                    if ser != preferred_serial:
                        msg_m += f" — saved {ser} for this session (update launch config to keep)"
                    break

                if attempt + 1 < len(serial_candidates):
                    ssh.exec(
                        "pkill -f mavproxy 2>/dev/null; "
                        "pkill -f MAVProxy 2>/dev/null; sleep 0.5"
                    )

            if not ok_m:
                mav_log = ssh.get_mavproxy_log(lines=5)
                if mav_log:
                    last_line = mav_log.strip().splitlines()[-1][:150]
                    msg_m = f"{msg_m} | Log: {last_line}"
                STATE["onboard_mavproxy"] = ok_m
                _emit_onboard_progress("mavproxy", "error", msg_m)
                _emit_onboard_progress(
                    "complete", "error",
                    "✕ Failed: MAVProxy / Pix6 — fix serial link then retry",
                )
                emit_status()
                return

            if not ssh.wait_mavproxy_tcp_ready(timeout_sec=20.0):
                ok_m = False
                msg_m = (
                    f"MAVProxy TCP :{MAVPROXY_TCP_PORT} not listening — "
                    f"onboard link must be {MAVPROXY_ONBOARD_OUT}"
                )
            STATE["onboard_mavproxy"] = ok_m
            _emit_onboard_progress(
                "mavproxy", "done" if ok_m else "error", msg_m
            )
            emit_status()
            if not ok_m:
                _emit_onboard_progress(
                    "complete", "error",
                    "✕ Failed: MAVProxy / Pix6 — fix serial link then retry",
                )
                emit_status()
                return

            # Step 2–4: stab, arm, cam in parallel (MAVProxy must be up first)
            cam0_dev = config.get("camera0_device", "/dev/video0")
            cam1_dev = config.get("camera1_device", "/dev/video2")
            cam_args = f"--cam0 {cam0_dev} --cam1 {cam1_dev}"

            parallel_specs = [
                ("stabilization", "stab", "onboard_stab", 75.0, ""),
                ("arm_ctrl", "arm", "onboard_arm", 75.0, ""),
                ("camera", "cam", "onboard_cam", 45.0, cam_args),
            ]
            parallel_labels = {
                "stabilization": "stabilization.py",
                "arm_ctrl": "new_ar.py (arm controller)",
                "camera": "camera_stream.py (MJPEG feeds)",
            }
            parallel_results: dict[str, tuple[bool, str]] = {}
            results_lock = threading.Lock()

            for step, _, _, _, _ in parallel_specs:
                _emit_onboard_progress(
                    step, "starting",
                    f"Launching {parallel_labels[step]}...",
                )

            def _launch_onboard_service(
                step: str,
                svc: str,
                state_key: str,
                timeout: float,
                extra_args: str,
            ):
                ok, msg = ssh.supervisor_start_and_wait(
                    svc, timeout_sec=timeout, extra_args=extra_args,
                )
                with results_lock:
                    parallel_results[step] = (ok, msg)
                with _state_lock:
                    STATE[state_key] = ok
                _emit_onboard_progress(step, "done" if ok else "error", msg)
                emit_status()

            threads = [
                threading.Thread(
                    target=_launch_onboard_service,
                    args=(step, svc, state_key, timeout, extra_args),
                    name=f"onboard-{svc}",
                    daemon=True,
                )
                for step, svc, state_key, timeout, extra_args in parallel_specs
            ]
            for t in threads:
                t.start()
            for t in threads:
                t.join()

            ok_s, msg_s = parallel_results.get("stabilization", (False, "missing result"))
            ok_a, msg_a = parallel_results.get("arm_ctrl", (False, "missing result"))
            ok_c, msg_c = parallel_results.get("camera", (False, "missing result"))

            core_ok = ok_m and ok_s
            if core_ok and ok_a and ok_c:
                summary = "✓ All onboard programs running (MAVProxy, stabilization, new_ar, cameras)"
            elif core_ok and ok_a:
                summary = "✓ ROV ready — cameras unavailable (check device paths / opencv install)"
            elif core_ok:
                parts_warn = []
                if not ok_a:
                    parts_warn.append("arm controller")
                if not ok_c:
                    parts_warn.append("cameras")
                summary = (
                    "✓ Thruster control ready. Optional component(s) failed: "
                    + ", ".join(parts_warn) + " — see Logs"
                )
            else:
                parts = []
                if not ok_m:
                    parts.append("MAVProxy")
                if not ok_s:
                    parts.append("stabilization")
                if not ok_a:
                    parts.append("new_ar")
                if not ok_c:
                    parts.append("cameras")
                summary = "✕ Failed: " + ", ".join(parts) + " — open Logs for details"

            _emit_onboard_progress(
                "complete",
                "done" if core_ok else "error",
                summary,
            )
            emit_status()
        except Exception as e:
            _emit_onboard_progress("complete", "error", f"Onboard start error: {e}")
            STATE["onboard_starting"] = False
            emit_status()

    socketio.start_background_task(_do_start)
    return jsonify({"ok": True, "msg": "Starting onboard programs..."})


@app.route("/api/onboard/stop", methods=["POST"])
def api_stop_onboard():
    ssh.supervisor_stop_all()
    ssh.stop_mavproxy()
    STATE["onboard_stab"]     = False
    STATE["onboard_arm"]      = False
    STATE["onboard_cam"]      = False
    STATE["onboard_mavproxy"] = False
    STATE["onboard_starting"] = False
    emit_status()
    return jsonify({"ok": True})


@app.route("/api/topside/start", methods=["POST"])
def api_start_topside():
    """Start arm_sender.py on topside. Gamepad control is built into the web UI."""
    data = request.get_json(force=True) or {}
    for k in ("pi_ip", "serial_port"):
        if k in data:
            config[k] = data[k]

    results = {}

    if not is_local_running("arm"):
        cmd = [
            PYTHON, str(ROV_ROOT / "topside" / "arm_sender.py"),
            "--ip",   config["pi_ip"],
            "--port", config["serial_port"],
        ]
        ok, msg = start_local_process("arm", cmd, cwd=ROV_ROOT)
        results["arm_sender"] = {"ok": ok, "msg": msg}
        STATE["arm_running"] = ok

        if ok:
            # Re-check after 1 s — if arm_sender crashed immediately (e.g. COM3
            # denied) this will flip the dot to red before the user notices a
            # false "running" state.
            def _arm_confirm():
                time.sleep(1.0)
                STATE["arm_running"] = is_local_running("arm")
                emit_status()
            socketio.start_background_task(_arm_confirm)
    else:
        results["arm_sender"] = {"ok": True, "msg": "already running"}

    emit_status()
    return jsonify({"ok": True, "results": results})


@app.route("/api/topside/stop", methods=["POST"])
def api_stop_topside():
    stop_local_process("thrust")
    stop_local_process("arm")
    STATE["thrust_running"] = False
    STATE["arm_running"]    = False
    emit_status()
    return jsonify({"ok": True})


@app.route("/api/mosfet", methods=["POST"])
def api_mosfet():
    data = request.get_json(force=True) or {}
    state = bool(data.get("state", False))
    ok, msg = ssh.send_mosfet(state)
    STATE["mosfet_on"] = state
    emit_status()
    return jsonify({"ok": ok, "msg": msg, "mosfet_on": state})


@app.route("/api/mode", methods=["POST"])
def api_mode():
    data = request.get_json(force=True) or {}
    mode = data.get("mode", "disarmed")
    if mode not in ("disarmed", "armed", "stabilize"):
        return jsonify({"ok": False, "msg": "invalid mode"})
    STATE["mode"] = mode
    emit_status()
    return jsonify({"ok": True, "mode": mode})


@app.route("/api/colmap", methods=["POST"])
def api_colmap():
    if not ssh.is_connected():
        return jsonify({"ok": False, "msg": "SSH not connected"})
    ok, msg = ssh.run_colmap()
    return jsonify({"ok": ok, "msg": msg})


@app.route("/api/crabs", methods=["POST"])
def api_crabs():
    if not ssh.is_connected():
        return jsonify({"ok": False, "msg": "SSH not connected"})
    ok, msg = ssh.run_crabs()
    return jsonify({"ok": ok, "msg": msg})


def _mission_script_status(name: str, cmd_fragment: str) -> dict:
    """Check if a mission script process is running and return log tail."""
    if not ssh.is_connected():
        return {"running": False, "log_tail": "", "last_line": ""}
    out, _, _ = ssh.exec(
        f"pgrep -f '{cmd_fragment}' >/dev/null 2>&1 && echo running || echo stopped"
    )
    running = "running" in (out or "")
    log_tail = ssh.get_onboard_log(name, lines=12)
    lines = [ln for ln in (log_tail or "").splitlines() if ln.strip()]
    return {
        "running": running,
        "log_tail": log_tail or "",
        "last_line": lines[-1] if lines else "",
    }


@app.route("/api/mission_status")
def api_mission_status():
    colmap_cmd = Path(config.get("colmap_command", "colmap_run.py")).name
    crabs_cmd = Path(config.get("crabs_command", "crabs.py")).name
    return jsonify({
        "colmap": _mission_script_status("colmap", colmap_cmd),
        "crabs":  _mission_script_status("crabs", crabs_cmd),
    })


@app.route("/api/arm_presets", methods=["GET"])
def api_arm_presets_list():
    normalize_arm_presets()
    with _state_lock:
        current = STATE.get("arm_last_pwm")
    return jsonify({
        "ok": True,
        "presets": config.get("arm_presets", {}),
        "current": current,
        "joint_names": ARM_JOINT_NAMES,
    })


@app.route("/api/arm_presets", methods=["POST"])
def api_arm_presets_save():
    data = request.get_json(force=True) or {}
    name = _slug_preset_name(data.get("name", ""))
    if not name:
        return jsonify({"ok": False, "msg": "Preset name required"}), 400

    pwm_in = data.get("pwm")
    if not isinstance(pwm_in, (list, tuple)) or len(pwm_in) < 7:
        return jsonify({"ok": False, "msg": "Need 7 PWM values (J1–Claw)"}), 400

    try:
        preset = {
            "label": str(data.get("label") or name.replace("_", " ").title()).strip(),
            "pwm": [_clamp_arm_pwm(x) for x in pwm_in[:7]],
            "j6_angle": float(data.get("j6_angle", 0.0)),
        }
    except (TypeError, ValueError):
        return jsonify({"ok": False, "msg": "Invalid PWM or J6 angle"}), 400

    normalize_arm_presets()
    config["arm_presets"][name] = preset
    save_config_file()
    return jsonify({"ok": True, "name": name, "preset": preset})


@app.route("/api/arm_presets/<name>", methods=["DELETE"])
def api_arm_presets_delete(name):
    slug = _slug_preset_name(name)
    normalize_arm_presets()
    if slug not in config.get("arm_presets", {}):
        return jsonify({"ok": False, "msg": f"Unknown preset: {name}"}), 404
    del config["arm_presets"][slug]
    save_config_file()
    return jsonify({"ok": True, "name": slug})


@app.route("/api/arm_preset/<name>", methods=["POST"])
def api_arm_preset(name):
    normalize_arm_presets()
    preset = config.get("arm_presets", {}).get(_slug_preset_name(name))
    if not preset:
        return jsonify({"ok": False, "msg": f"Unknown preset: {name}"}), 404
    try:
        csv_line = _preset_to_csv(preset)
        _send_arm_csv(csv_line)
        return jsonify({
            "ok": True,
            "msg": f"Sent arm preset '{name}'",
            "preset": name,
            "csv": csv_line,
        })
    except Exception as e:
        return jsonify({"ok": False, "msg": str(e)}), 500


@app.route("/api/telemetry_record", methods=["POST"])
def api_telemetry_record():
    data = request.get_json(force=True) or {}
    action = data.get("action", "toggle")

    if action == "start" or (action == "toggle" and not STATE["telemetry_recording"]):
        LOGS_DIR.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = LOGS_DIR / f"dive_{stamp}.csv"
        header = ",".join(TELEMETRY_CSV_FIELDS) + "\n"
        try:
            path.write_text(header, encoding="utf-8")
        except Exception as e:
            return jsonify({"ok": False, "msg": str(e)}), 500
        STATE["telemetry_recording"] = True
        STATE["telemetry_record_file"] = str(path)
        emit_status()
        return jsonify({"ok": True, "recording": True, "file": str(path)})

    STATE["telemetry_recording"] = False
    prev = STATE.get("telemetry_record_file", "")
    emit_status()
    return jsonify({"ok": True, "recording": False, "file": prev})


@app.route("/api/status")
def api_status():
    with _state_lock:
        progress = list(STATE["onboard_progress"])
    return jsonify({
        "thrust_running":        STATE["thrust_running"],
        "arm_running":           STATE["arm_running"],
        "onboard_stab":          STATE["onboard_stab"],
        "onboard_arm":           STATE["onboard_arm"],
        "onboard_cam":           STATE["onboard_cam"],
        "onboard_mavproxy":      STATE["onboard_mavproxy"],
        "ssh_connected":         STATE["ssh_connected"],
        "mode":                  STATE["mode"],
        "mosfet_on":             STATE["mosfet_on"],
        "telemetry_listener_ok": STATE["telemetry_listener_ok"],
        "onboard_starting":      STATE["onboard_starting"],
        "onboard_progress":      progress,
        "telemetry":             STATE["telemetry"],
        "telemetry_recording":   STATE["telemetry_recording"],
        "telemetry_record_file": STATE.get("telemetry_record_file", ""),
        "link_health":           _compute_link_health(),
        "arm_last_pwm":          STATE.get("arm_last_pwm"),
    })


@app.route("/api/ctrl", methods=["POST"])
def api_ctrl():
    """HTTP fallback for gamepad control (when WebSocket is unavailable)."""
    data = request.get_json(force=True) or {}
    _apply_browser_ctrl(data)
    return jsonify({"ok": True})


@app.route("/api/logs/<name>")
def api_logs(name):
    with _state_lock:
        lines = list(STATE["logs"].get(name, []))
    return jsonify({"lines": lines})


@app.route("/api/onboard/progress")
def api_onboard_progress():
    with _state_lock:
        return jsonify({
            "starting": STATE["onboard_starting"],
            "events": list(STATE["onboard_progress"]),
            "onboard_mavproxy": STATE["onboard_mavproxy"],
            "onboard_stab": STATE["onboard_stab"],
            "onboard_arm": STATE["onboard_arm"],
            "onboard_cam": STATE["onboard_cam"],
        })


@app.route("/api/onboard_log/<name>")
def api_onboard_log(name):
    allowed = {"stab", "arm", "cam", "colmap", "crabs"}
    if name not in allowed:
        return jsonify({"lines": []})
    out = ssh.get_onboard_log(name)
    return jsonify({"lines": out.splitlines() if out else []})


# ─────────────────────────────────────────────────────────────────────────────
# SOCKETIO EVENTS
# ─────────────────────────────────────────────────────────────────────────────

@socketio.on("connect")
def on_connect():
    emit_status()
    socketio.emit("telemetry", dict(STATE["telemetry"]))
    with _state_lock:
        for entry in STATE["onboard_progress"]:
            socketio.emit("onboard_progress", entry)


@socketio.on("request_status")
def on_request_status(_data=None):
    emit_status()


@socketio.on("ctrl_packet")
def on_ctrl_packet(data):
    """Receive gamepad control packet from browser, forward to Pi via UDP."""
    _apply_browser_ctrl(data)


def _apply_browser_ctrl(data: dict):
    global _last_browser_ctrl, _last_browser_ctrl_time
    with _ctrl_lock:
        _last_browser_ctrl = data
        _last_browser_ctrl_time = time.time()
    STATE["last_ctrl_time"] = time.time()
    _forward_ctrl_to_pi(data)


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="DreadYachet ROV Web Control UI")
    parser.add_argument("--port",       type=int, default=8080, help="Web server port (default 8080)")
    parser.add_argument("--host",       type=str, default="0.0.0.0", help="Bind address")
    parser.add_argument("--no-browser", action="store_true", help="Don't auto-open browser")
    args = parser.parse_args()

    if not HAVE_PARAMIKO:
        print("[WARN] paramiko not installed — SSH features disabled. Run: pip install paramiko")
    if not HAVE_REQUESTS:
        print("[WARN] requests not installed — camera proxy disabled. Run: pip install requests")

    # Start background threads
    stop_local_process("thrust")  # free UDP telemetry port if old thrust_sender was running
    _start_telemetry_listener()
    _start_control_keepalive()
    threading.Thread(target=_monitor_loop, daemon=True).start()

    url = f"http://localhost:{args.port}"
    print(f"\n{'='*55}")
    print(f"  DreadYachet ROV Control UI")
    print(f"  Open: {url}")
    print(f"  Telemetry listening on UDP port {config['telemetry_port']}")
    print(f"  Control packets → Pi UDP port {config['thrust_udp_port']}")
    print(f"{'='*55}\n")

    if not args.no_browser:
        threading.Timer(1.2, lambda: webbrowser.open(url)).start()

    socketio.run(app, host=args.host, port=args.port, debug=False, allow_unsafe_werkzeug=True)


if __name__ == "__main__":
    main()
