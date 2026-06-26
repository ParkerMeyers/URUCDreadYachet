#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
"""
ROV Web Control UI — DreadYachet ROV Main Control System
=========================================================

Modular topside dashboard. Shared code lives in ``topside/``; this file
holds Flask routes and segment wiring.

Robot segments (each debuggable independently):
  THRUST   — browser gamepad → UDP 5005 → onboard/stabilization.py
  ARM      — arm_sender.py → UDP 5006 → onboard/new_ar.py
  MOSFET   — UDP 5007 → onboard/mosfet_service.py
  CAMERA   — HTTP MJPEG ← onboard/camera_stream.py (:8160/:8161)
  SSH      — Pi process supervisor (onboard/supervisor.py)

Package layout:
  topside/config.py      — rov_config.json load/save
  topside/state.py       — runtime STATE dict
  topside/constants.py   — PWM limits, joint maps, ports
  topside/ssh_manager.py — SSH + MAVProxy + onboard start
  topside/segments/mosfet.py — MOSFET UDP commands
  onboard/ports.py       — shared UDP/TCP port map

Usage:
    python rov_ui.py
    Open http://localhost:8080
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

try:
    import serial as _serial  # pyserial — required by topside/arm_sender.py
    HAVE_PYSERIAL = True
except ImportError:
    HAVE_PYSERIAL = False

# ─────────────────────────────────────────────────────────────────────────────
# FLASK + SOCKETIO SETUP
# ─────────────────────────────────────────────────────────────────────────────

app = Flask(__name__)
app.config["SECRET_KEY"] = "dreadyachet-rov-2025"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# UDP socket for sending control packets to Pi (replaces thrust_sender.py)
_pi_ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Control sender — one fixed-rate UDP thread (Pi needs steady packets; duplicate
# senders + bursty Socket.IO handling caused jitter / CONTROL_TIMEOUT flapping).
_ctrl_lock = threading.Lock()
_last_browser_ctrl: dict | None = None
_last_browser_ctrl_time = 0.0
_ctrl_keepalive_seq = 0
CTRL_SEND_HZ = 50

# ─────────────────────────────────────────────────────────────────────────────
# SHARED MODULES (config, state, constants, SSH, MOSFET segment)
# ─────────────────────────────────────────────────────────────────────────────

from topside.config import (
    CONFIG_PATH,
    ROV_ROOT,
    config,
    load_config_file,
    normalize_arm_presets,
    normalize_onboard_config,
    save_config_file,
    slug_preset_name,
)
from topside.constants import *  # noqa: F403
from topside.state import (
    LOGS_DIR,
    STATE,
    VIDEO_DIR,
    _state_lock,
    _telemetry_rate_counter,
    _telemetry_record_lock,
    manual_aux_defaults,
    manual_thr_defaults,
)
from topside.util import clamp_arm_pwm, clamp_thr_pwm
from topside.ssh_manager import ssh
from topside.segments import mosfet

_manual_aux_defaults = manual_aux_defaults
_manual_thr_defaults = manual_thr_defaults
_clamp_arm_pwm = clamp_arm_pwm
_clamp_thr_pwm = clamp_thr_pwm
_slug_preset_name = slug_preset_name
_mosfet_enabled = mosfet.is_enabled
_send_mosfet_command = mosfet.send_command

# ─────────────────────────────────────────────────────────────────────────────
# ARM SEGMENT — presets, UDP control, manual PWM parsing
# ─────────────────────────────────────────────────────────────────────────────


_preset_lock = threading.Lock()


def _pwm_to_csv(pwms: list, j6_angle: float) -> str:
    vals = [_clamp_arm_pwm(v) for v in pwms[:7]]
    while len(vals) < 7:
        vals.append(1500)
    return ",".join(str(x) for x in vals) + f",{float(j6_angle):.2f}"


def _preset_to_csv(preset: dict) -> str:
    pwms = [_clamp_arm_pwm(v) for v in preset["pwm"][:7]]
    while len(pwms) < 7:
        pwms.append(1500)
    angle = float(preset.get("j6_angle", 0.0))
    return _pwm_to_csv(pwms, angle)


def _current_arm_pose() -> tuple[list[int], float]:
    """Last known J1..Claw PWM + J6 target angle from arm_sender telemetry."""
    raw = STATE.get("arm_last_pwm")
    if isinstance(raw, dict) and isinstance(raw.get("pwm"), list) and len(raw["pwm"]) >= 7:
        pwms = [_clamp_arm_pwm(x) for x in raw["pwm"][:7]]
        angle = float(raw.get("j6_angle", 0.0))
        return pwms, angle
    return list(ARM_DEFAULT_PWM), 0.0


def _joint_move_delay_sec(from_us: int, to_us: int) -> float:
    """Wait time proportional to PWM travel (full span ≈ max delay)."""
    delta = abs(_clamp_arm_pwm(to_us) - _clamp_arm_pwm(from_us))
    if delta < 12:
        return 0.0
    span = float(ARM_PWM_MAX - ARM_PWM_MIN)
    travel = delta / span
    delay = ARM_PRESET_DELAY_MIN_SEC + travel * (
        ARM_PRESET_DELAY_MAX_SEC - ARM_PRESET_DELAY_MIN_SEC
    )
    return round(min(ARM_PRESET_DELAY_MAX_SEC, max(ARM_PRESET_DELAY_MIN_SEC, delay)), 3)


def _emit_preset_progress(payload: dict) -> None:
    socketio.emit("preset_progress", payload)
    emit_status()


def _run_preset_sequence(preset_name: str, preset: dict) -> None:
    """Move joints one at a time J6→J1→Claw with distance-based pauses."""
    label = str(preset.get("label") or preset_name)
    target_pwms = [_clamp_arm_pwm(v) for v in preset["pwm"][:7]]
    while len(target_pwms) < 7:
        target_pwms.append(1500)
    target_angle = float(preset.get("j6_angle", 0.0))

    try:
        _send_pi_arm_control({"cmd": "preset_motion", "enabled": True})
        current_pwms, current_angle = _current_arm_pose()
        working = list(current_pwms)
        angle = current_angle
        total = len(ARM_PRESET_JOINT_ORDER)

        for step_i, joint_idx in enumerate(ARM_PRESET_JOINT_ORDER, start=1):
            if not _robot_armed():
                _emit_preset_progress({
                    "status": "error",
                    "preset": preset_name,
                    "label": label,
                    "msg": "Preset aborted — ROV disarmed",
                })
                return

            joint_name = ARM_JOINT_NAMES[joint_idx]
            from_us = working[joint_idx]
            to_us = target_pwms[joint_idx]
            working[joint_idx] = to_us
            if joint_idx == 5:
                angle = target_angle

            delay = _joint_move_delay_sec(from_us, to_us)
            _send_pi_arm_control({
                "cmd": "preset_step",
                "pwm": list(working),
                "j6_angle": angle,
            })

            with _state_lock:
                STATE["arm_last_pwm"] = {"pwm": list(working), "j6_angle": angle}

            _emit_preset_progress({
                "status": "running",
                "preset": preset_name,
                "label": label,
                "joint": joint_name,
                "step": step_i,
                "total": total,
                "delay_sec": delay,
                "from_us": from_us,
                "to_us": to_us,
            })

            if delay > 0:
                time.sleep(delay)

        _emit_preset_progress({
            "status": "done",
            "preset": preset_name,
            "label": label,
            "msg": f"Preset '{label}' complete",
        })
    except Exception as e:
        _emit_preset_progress({
            "status": "error",
            "preset": preset_name,
            "label": label,
            "msg": str(e),
        })
    finally:
        _send_pi_arm_control({"cmd": "preset_motion", "enabled": False})
        with _state_lock:
            STATE["preset_running"] = False
            STATE["preset_active_name"] = ""
        emit_status()


def _start_preset_sequence(preset_name: str, preset: dict) -> tuple[bool, str]:
    with _preset_lock:
        if STATE.get("preset_running"):
            return False, "Another preset sequence is already running"
        STATE["preset_running"] = True
        STATE["preset_active_name"] = preset_name
    threading.Thread(
        target=_run_preset_sequence,
        args=(preset_name, preset),
        daemon=True,
        name=f"preset-{preset_name}",
    ).start()
    return True, f"Moving to preset '{preset_name}' (J6→J1→Claw)"


def _robot_armed() -> bool:
    return STATE.get("mode", "disarmed") in ("armed", "stabilize")


def _sync_arm_enable():
    """Tell new_ar.py whether arm motion is allowed (disabled when DISARMED)."""
    if _robot_armed():
        _sync_arm_unlock()
    else:
        _send_pi_arm_control({"cmd": "arm_enable", "enabled": False})


def _arm_control_ports() -> list[int]:
    """UDP ports for JSON arm control (primary = arm CSV port, legacy 5009 optional)."""
    load_config_file()
    primary = int(config.get("arm_udp_port", 5006))
    try:
        legacy = int(config.get("arm_control_port", primary))
    except (TypeError, ValueError):
        legacy = primary
    ports = [primary]
    if legacy != primary:
        ports.append(legacy)
    if 5009 not in ports and primary != 5009:
        ports.append(5009)
    return ports


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


def _send_pi_arm_control(payload: dict) -> tuple[bool, str]:
    """Send JSON control command to new_ar.py on the arm UDP port(s)."""
    load_config_file()
    try:
        body = json.dumps(payload).encode("utf-8")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sent_ports = []
            for port in _arm_control_ports():
                sock.sendto(body, (config["pi_ip"], port))
                sent_ports.append(str(port))
        finally:
            sock.close()
        return True, f"sent → {config['pi_ip']}:{','.join(sent_ports)}"
    except Exception as e:
        return False, str(e)


def _send_pi_stab_control(payload: dict) -> tuple[bool, str]:
    """Send JSON control command to stabilization.py on the thrust UDP port."""
    load_config_file()
    try:
        body = json.dumps(payload).encode("utf-8")
        port = int(config["thrust_udp_port"])
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(body, (config["pi_ip"], port))
        finally:
            sock.close()
        return True, f"sent → {config['pi_ip']}:{port}"
    except Exception as e:
        return False, str(e)


def _sync_arm_power():
    """Push current MOSFET state to mosfet_service.py on the Pi."""
    if not _mosfet_enabled():
        return
    # Servo rail needs power for any arm motion — auto-enable when armed.
    if _robot_armed() and not STATE.get("mosfet_on"):
        STATE["mosfet_on"] = True
    _send_mosfet_command(bool(STATE.get("mosfet_on", False)))


def _apply_disarmed_arm_lockout():
    """Stop arm motion and overrides when the ROV is disarmed (MOSFET unchanged)."""
    if _robot_armed():
        _sync_arm_unlock()
        return
    STATE["manual_pwm_enabled"] = False
    STATE["preset_running"] = False
    STATE["preset_active_name"] = ""
    _send_pi_arm_control({"cmd": "arm_enable", "enabled": False})
    _send_pi_arm_control({"cmd": "manual_pwm", "enabled": False})
    _send_pi_stab_control({"cmd": "manual_pwm", "enabled": False})
    _send_pi_arm_control({"cmd": "preset_motion", "enabled": False})


def _sync_arm_unlock():
    """Allow arm motion on the Pi (clears stale preset lock)."""
    _send_pi_arm_control({"cmd": "arm_enable", "enabled": True})
    if not STATE.get("preset_running"):
        _send_pi_arm_control({"cmd": "preset_motion", "enabled": False})


def _send_arm_csv(csv_line: str):
    if not _robot_armed():
        return
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(
            csv_line.encode("utf-8"),
            (config["pi_ip"], int(config["arm_udp_port"])),
        )
    finally:
        sock.close()


def _parse_manual_pwm_line(line: str) -> tuple[str, int, int, str] | None:
    """Parse manual PWM line → (kind, index, pwm, label). kind is 'aux' or 'motor'."""
    line = (line or "").strip()
    if not line:
        return None
    parts = line.split()
    if len(parts) != 2:
        return None

    token = parts[0].strip().lower()
    try:
        pwm_raw = parts[1]
    except IndexError:
        return None

    # Thruster: M1–M8 or motor name alias
    if token.startswith("m") and len(token) > 1 and token[1:].isdigit():
        motor_i = int(token[1:])
        if not (1 <= motor_i <= 8):
            return None
        try:
            pwm = _clamp_thr_pwm(pwm_raw)
        except (TypeError, ValueError):
            return None
        label = f"M{motor_i} ({MANUAL_THR_LABELS[motor_i - 1]})"
        return "motor", motor_i, pwm, label

    if token in MOTOR_NAME_ALIASES:
        motor_i = MOTOR_NAME_ALIASES[token]
        try:
            pwm = _clamp_thr_pwm(pwm_raw)
        except (TypeError, ValueError):
            return None
        label = f"M{motor_i} ({MANUAL_THR_LABELS[motor_i - 1]})"
        return "motor", motor_i, pwm, label

    # Arm AUX / joint
    try:
        pwm = _clamp_arm_pwm(pwm_raw)
    except (TypeError, ValueError):
        return None

    if token in ("claw",):
        return "aux", 7, pwm, "Claw"
    if token in ("j6", "wrist"):
        return "aux", JOINT_TO_AUX[6], pwm, "J6"
    if token.startswith("j") and token[1:].isdigit():
        joint_i = int(token[1:])
        if joint_i not in JOINT_TO_AUX:
            return None
        aux = JOINT_TO_AUX[joint_i]
        label = f"J{joint_i}" if joint_i < 7 else "Claw"
        return "aux", aux, pwm, label
    try:
        aux = int(parts[0])
    except (TypeError, ValueError):
        return None
    if not (1 <= aux <= 7):
        return None
    label = f"AUX{aux} ({MANUAL_AUX_LABELS[aux - 1]})"
    return "aux", aux, pwm, label



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
        "stabilize": STATE.get("ctrl_stabilize", False) if armed else False,
        "depth_hold": STATE.get("ctrl_depth_hold", False) if armed else False,
        "yaw_hold": STATE.get("ctrl_yaw_hold", False) if armed else False,
        "gain_percent": STATE["telemetry"].get("gain_percent", 100),
        "telemetry_port": int(config["telemetry_port"]),
    }


def _make_keepalive_packet() -> dict:
    """Backward-compatible alias."""
    return _get_active_ctrl_packet()


def _start_control_sender():
    """Send control UDP at a fixed 50 Hz — browser only updates the cached packet."""
    interval = 1.0 / CTRL_SEND_HZ

    def _loop():
        next_t = time.time()
        while True:
            _forward_ctrl_to_pi(_get_active_ctrl_packet())
            next_t += interval
            delay = next_t - time.time()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.time()

    threading.Thread(target=_loop, daemon=True, name="ctrl-sender").start()


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


def _arm_telemetry_age_sec() -> float | None:
    last = STATE.get("last_arm_telemetry_time", 0.0)
    if last <= 0:
        return None
    return round(time.time() - last, 2)


def _reset_onboard_telemetry_state() -> None:
    """Clear stale telemetry after onboard stop/restart so UI waits for fresh data."""
    tel = STATE["telemetry"]
    tel["rx_state"] = "NO_TELEMETRY"
    tel["attitude_stale"] = False
    tel["depth_stale"] = False
    tel["mavlink_link_dead"] = False
    tel["control_timeout"] = False
    tel["arm_imu_ok"] = False
    tel["arm_bno_ready"] = False
    tel["arm_imu_stale"] = False
    tel["arm_imu_angle_deg"] = None
    tel["arm_j6_target_deg"] = None
    tel["arm_j6_pwm_out"] = None
    tel["arm_claw_hold_active"] = False
    tel["arm_hold_neutral"] = None
    tel["arm_rx_count"] = None
    tel["arm_enabled"] = None
    tel["arm_mavlink_ok"] = None
    tel["arm_joint_us"] = None
    tel["arm_manual_mode"] = None
    tel["arm_preset_motion"] = None
    STATE["last_telemetry_time"] = 0.0
    STATE["last_arm_telemetry_time"] = 0.0
    STATE["telemetry_packets"] = 0
    STATE["telemetry_rate_hz"] = 0.0


def _on_onboard_stack_ready() -> None:
    """Re-subscribe topside listeners after Pi processes restart."""
    _reset_onboard_telemetry_state()
    _subscribe_arm_telemetry()
    _sync_arm_claw_hold()
    _sync_arm_imu_cal()
    _sync_arm_claw_stop()
    _apply_disarmed_arm_lockout()
    if STATE.get("onboard_mosfet"):
        _sync_arm_power()
    socketio.emit("telemetry", _telemetry_emit_payload())
    emit_status()

    def _resync_arm_after_restart():
        for delay in (0.75, 2.0, 5.0):
            time.sleep(delay)
            if STATE.get("onboard_mosfet"):
                _sync_arm_power()
            if not STATE.get("onboard_arm"):
                continue
            _subscribe_arm_telemetry()
            _sync_arm_claw_hold()
            _sync_arm_imu_cal()
            _sync_arm_claw_stop()
            _sync_arm_enable()
            if STATE.get("onboard_mosfet"):
                _sync_arm_power()

    socketio.start_background_task(_resync_arm_after_restart)


def _telemetry_emit_payload() -> dict:
    payload = dict(STATE["telemetry"])
    payload["link_health"] = _compute_link_health()
    payload["arm_telemetry_age_sec"] = _arm_telemetry_age_sec()
    return payload


def _update_telemetry_from_json(pkt: dict):
    """Map stabilization.py JSON telemetry → UI state and emit to browser."""
    if pkt.get("type") == "arm":
        _update_arm_telemetry_from_json(pkt)
        return

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
    socketio.emit("telemetry", _telemetry_emit_payload())


def _update_arm_telemetry_from_json(pkt: dict):
    """Merge arm BNO055 gripper telemetry into UI state."""
    tel = STATE["telemetry"]
    tel["arm_bno_ready"]     = bool(pkt.get("arm_bno_ready", pkt.get("arm_imu_ok")))
    tel["arm_imu_ok"]        = bool(pkt.get("arm_imu_ok"))
    tel["arm_imu_stale"]     = bool(pkt.get("arm_imu_stale"))
    tel["arm_imu_angle_deg"] = pkt.get("arm_imu_angle_deg")
    tel["arm_j6_target_deg"] = pkt.get("arm_j6_target_deg")
    tel["arm_j6_pwm_out"]    = pkt.get("arm_j6_pwm_out")
    tel["arm_claw_hold_request"] = bool(pkt.get("arm_claw_hold_request", tel.get("arm_claw_hold_request", False)))
    tel["arm_claw_hold_active"]  = bool(pkt.get("arm_claw_hold_active", False))
    tel["arm_j6_manual"]         = bool(pkt.get("arm_j6_manual", True))
    if pkt.get("arm_enabled") is not None:
        tel["arm_enabled"] = bool(pkt.get("arm_enabled"))
    if pkt.get("arm_rx_count") is not None:
        tel["arm_rx_count"] = int(pkt.get("arm_rx_count"))
    if pkt.get("arm_hold_neutral") is not None:
        tel["arm_hold_neutral"] = bool(pkt.get("arm_hold_neutral"))
    if pkt.get("arm_mavlink_ok") is not None:
        tel["arm_mavlink_ok"] = bool(pkt.get("arm_mavlink_ok"))
    if isinstance(pkt.get("arm_joint_us"), list):
        tel["arm_joint_us"] = pkt["arm_joint_us"]
    if pkt.get("arm_manual_mode") is not None:
        tel["arm_manual_mode"] = bool(pkt.get("arm_manual_mode"))
    if pkt.get("arm_preset_motion") is not None:
        tel["arm_preset_motion"] = bool(pkt.get("arm_preset_motion"))
    if pkt.get("arm_imu_zero_offset") is not None:
        tel["arm_imu_zero_offset"] = float(pkt["arm_imu_zero_offset"])
    if pkt.get("arm_imu_sign") is not None:
        tel["arm_imu_sign"] = float(pkt["arm_imu_sign"])
    if pkt.get("arm_claw_stop_us") is not None:
        tel["arm_claw_stop_us"] = int(pkt["arm_claw_stop_us"])
    if pkt.get("arm_mosfet_on") is not None:
        tel["arm_mosfet_on"] = bool(pkt.get("arm_mosfet_on"))
    if pkt.get("arm_mosfet_gpio_ok") is not None:
        tel["arm_mosfet_gpio_ok"] = bool(pkt.get("arm_mosfet_gpio_ok"))
    if pkt.get("arm_imu_read_age_sec") is not None:
        tel["arm_imu_read_age_sec"] = float(pkt["arm_imu_read_age_sec"])
    STATE["last_arm_telemetry_time"] = time.time()
    socketio.emit("telemetry", _telemetry_emit_payload())


def _sync_arm_claw_hold():
    """Push claw-hold flag to new_ar.py (J6 IMU auto-level)."""
    _send_pi_arm_control({
        "cmd": "claw_hold",
        "enabled": bool(STATE.get("claw_hold", False)),
    })


def _sync_arm_claw_stop():
    """Push persisted claw stop PWM to new_ar.py."""
    _send_pi_arm_control({
        "cmd": "arm_claw_stop",
        "stop_us": int(config.get("arm_claw_stop_us", 1515)),
    })


def _sync_arm_imu_cal():
    """Push persisted arm IMU sign/zero to new_ar.py."""
    _send_pi_arm_control({
        "cmd": "arm_imu_cal",
        "sign": float(config.get("arm_imu_sign", -1.0)),
        "zero_offset_deg": float(config.get("arm_imu_zero_offset", -154.0)),
    })


def _subscribe_arm_telemetry():
    """Ask new_ar.py on the Pi to push arm IMU data to our UDP listener."""
    try:
        payload = json.dumps({
            "cmd": "arm_telemetry",
            "subscribe": True,
            "port": int(config.get("arm_telemetry_port", 5008)),
        }).encode("utf-8")
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            body = payload
            for port in _arm_control_ports():
                s.sendto(body, (config["pi_ip"], port))
        finally:
            s.close()
    except Exception as e:
        print(f"[WARN] Arm telemetry subscribe failed: {e}")


def _start_arm_telemetry_subscribe_loop():
    """Re-subscribe periodically so new_ar picks us up after restarts."""
    def _loop():
        while True:
            _subscribe_arm_telemetry()
            _sync_arm_claw_hold()
            _sync_arm_imu_cal()
            _sync_arm_claw_stop()
            _sync_arm_enable()
            _sync_arm_power()
            time.sleep(5.0)

    threading.Thread(target=_loop, daemon=True, name="arm-tel-sub").start()


def _start_arm_telemetry_listener():
    """Listen for JSON arm IMU telemetry from new_ar.py on arm_telemetry_port."""
    def _listen():
        port = int(config.get("arm_telemetry_port", 5008))
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("0.0.0.0", port))
            s.settimeout(1.0)
        except Exception as e:
            print(
                f"[ERROR] Arm telemetry listener bind failed on port {port}: {e}"
            )
            return

        print(f"[INFO] Arm telemetry listener active on UDP port {port}")
        _subscribe_arm_telemetry()
        _sync_arm_claw_hold()
        _sync_arm_imu_cal()
        _sync_arm_claw_stop()
        _sync_arm_enable()
        _sync_arm_power()
        while True:
            try:
                data, _ = s.recvfrom(4096)
                try:
                    pkt = json.loads(data.decode("utf-8"))
                    if pkt.get("type") == "arm":
                        _update_arm_telemetry_from_json(pkt)
                except Exception:
                    pass
            except socket.timeout:
                pass
            except Exception:
                pass

    threading.Thread(target=_listen, daemon=True, name="arm-tel-listen").start()


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
                STATE["onboard_mosfet"]   = status["mosfet"]
                STATE["onboard_stab"]     = status["stab"]
                STATE["onboard_arm"]      = status["arm"]
                STATE["onboard_cam"]      = status["cam"]
            else:
                STATE["ssh_connected"]    = False
                STATE["onboard_mavproxy"] = False
                STATE["onboard_mosfet"]   = False
                STATE["onboard_stab"]     = False
                STATE["onboard_arm"]      = False
                STATE["onboard_cam"]      = False
                # Auto-reconnect only if the user had a working session before.
                if _ssh_was_connected:
                    _trigger_ssh_reconnect()

        tel_age = time.time() - STATE["last_telemetry_time"]
        if tel_age > 2.0:
            STATE["telemetry"]["rx_state"] = "NO_TELEMETRY"

        arm_age = _arm_telemetry_age_sec()
        if arm_age is not None and arm_age > 3.0:
            _subscribe_arm_telemetry()

        emit_status()


def emit_status():
    with _state_lock:
        progress = list(STATE["onboard_progress"])
    link = _compute_link_health()
    socketio.emit("status", {
        "thrust_running":        STATE["thrust_running"],
        "arm_running":           STATE["arm_running"],
        "onboard_stab":          STATE["onboard_stab"],
        "onboard_mosfet":        STATE["onboard_mosfet"],
        "onboard_arm":           STATE["onboard_arm"],
        "onboard_cam":           STATE["onboard_cam"],
        "onboard_mavproxy":      STATE["onboard_mavproxy"],
        "ssh_connected":         STATE["ssh_connected"],
        "ssh_error":             STATE["ssh_error"],
        "mode":                  STATE["mode"],
        "mosfet_on":             STATE["mosfet_on"],
        "mosfet_enabled":        _mosfet_enabled(),
        "telemetry_listener_ok": STATE["telemetry_listener_ok"],
        "onboard_starting":      STATE["onboard_starting"],
        "onboard_progress":      progress,
        "telemetry_recording":   STATE["telemetry_recording"],
        "telemetry_record_file": STATE.get("telemetry_record_file", ""),
        "video_recording":       STATE["video_recording"],
        "video_record_session":  STATE.get("video_record_session", ""),
        "video_record_mode":     STATE.get("video_record_mode", ""),
        "video_record_files":    list(STATE.get("video_record_files", [])),
        "link_health":           link,
        "arm_last_pwm":          STATE.get("arm_last_pwm"),
        "preset_running":        STATE.get("preset_running", False),
        "preset_active_name":    STATE.get("preset_active_name", ""),
        "manual_pwm_enabled":    STATE.get("manual_pwm_enabled", False),
        "manual_aux_pwm":        STATE.get("manual_aux_pwm", list(_manual_aux_defaults())),
        "manual_thr_pwm":        STATE.get("manual_thr_pwm", list(_manual_thr_defaults())),
        "claw_hold":             STATE.get("claw_hold", False),
        "arm_claw_stop_us":      int(config.get("arm_claw_stop_us", 1515)),
        "arm_motion_enabled":    _robot_armed(),
        "arm_pi_enabled":        STATE.get("telemetry", {}).get("arm_enabled"),
    })


# ─────────────────────────────────────────────────────────────────────────────
# CAMERA PROXY
# ─────────────────────────────────────────────────────────────────────────────

# UI slot 1 = forward, slot 2 = arm
_CAMERA_UI_URL_KEY = {1: "forward_camera_url", 2: "arm_camera_url"}
# Pi USB cameras can take >1s to accept a second HTTP client when both feeds start.
_CAMERA_CONNECT_TIMEOUT = (5, 120)
_CAMERA_CONNECT_RETRIES = 3


def _open_camera_upstream(cam_url: str):
    """Open one MJPEG upstream with short retries before returning 502 to the browser."""
    last_exc = None
    for attempt in range(_CAMERA_CONNECT_RETRIES):
        try:
            resp = _requests.get(cam_url, stream=True, timeout=_CAMERA_CONNECT_TIMEOUT)
            resp.raise_for_status()
            return resp
        except Exception as exc:
            last_exc = exc
            if attempt + 1 < _CAMERA_CONNECT_RETRIES:
                time.sleep(0.4 * (attempt + 1))
    raise last_exc


@app.route("/camera/<int:cam_num>")
def camera_stream(cam_num):
    if cam_num not in (1, 2):
        return "", 404

    url_key = _CAMERA_UI_URL_KEY.get(cam_num, f"camera{cam_num}_url")
    cam_url = str(config.get(url_key, "")).strip()
    if not HAVE_REQUESTS:
        return Response("requests not installed", status=503)
    if not cam_url:
        return Response("camera URL not configured", status=503)

    # One browser request == one upstream connection. Connect before streaming so
    # unreachable cameras return 502 (img.onerror) instead of 200 with an empty
    # body that leaves the UI stuck on "No Signal" until the client watchdog fires.
    try:
        upstream = _open_camera_upstream(cam_url)
    except Exception as exc:
        return Response(str(exc), status=502)

    content_type = upstream.headers.get(
        "Content-Type", "multipart/x-mixed-replace; boundary=frame"
    )

    def _gen(resp=upstream):
        # If the upstream drops, end the response and let the browser reconnect
        # with a fresh request (client watchdog/retry). Do not loop server-side:
        # aborted browser feeds used to leak threads and starve camera_stream.py.
        try:
            for chunk in resp.iter_content(chunk_size=8192):
                if chunk:
                    yield chunk
        finally:
            try:
                resp.close()
            except Exception:
                pass

    return Response(
        stream_with_context(_gen()),
        mimetype=content_type,
        headers={"Cache-Control": "no-cache, no-store, must-revalidate"},
    )


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
# TOPSIDE CV  —  crab detection overlay  +  COLMAP frame recorder
# Both run topside on the proxied MJPEG feeds (cv2 reads MJPEG-over-HTTP via
# FFMPEG).  We reuse the inference math in extra/crab_detector.py but NOT its
# GStreamer/H.264 stream code — this UI's cameras are MJPEG, not RTP.
# ─────────────────────────────────────────────────────────────────────────────

import shutil

try:
    import cv2 as _cv2
    import numpy as np
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False

_HERE = os.path.dirname(os.path.abspath(__file__))
_EXTRA_DIR = os.path.join(_HERE, "extra")

_crab_lock = threading.Lock()
_crab = {"sess": None, "input": None, "cd": None}


def _crab_engine():
    """Lazy-load crab_model.onnx once; reuse crab_detector.py pre/post/draw."""
    with _crab_lock:
        if _crab["sess"] is None:
            import onnxruntime as ort
            if _EXTRA_DIR not in sys.path:
                sys.path.insert(0, _EXTRA_DIR)
            import crab_detector as cd
            model = os.path.join(_EXTRA_DIR, "crab_model.onnx")
            sess = ort.InferenceSession(model, providers=["CPUExecutionProvider"])
            _crab["sess"] = sess
            _crab["input"] = sess.get_inputs()[0].name
            _crab["cd"] = cd
        return _crab["sess"], _crab["input"], _crab["cd"]


def _crab_mjpeg(cam_url):
    """Yield annotated JPEG bytes — green-crab boxes + count burned into frame.

    ponytail: CPU inference of YOLO11m runs ~2-4 fps; switch the onnxruntime
    provider to CUDA/DirectML if a faster overlay is needed.
    """
    sess, input_name, cd = _crab_engine()
    cap = None
    try:
        while True:
            if cap is None or not cap.isOpened():
                if cap is not None:
                    cap.release()
                cap = _cv2.VideoCapture(cam_url)
                cap.set(_cv2.CAP_PROP_BUFFERSIZE, 1)
                if not cap.isOpened():
                    time.sleep(1.0)
                    continue
            ok, frame = cap.read()
            if not ok or frame is None:
                cap.release()
                cap = None
                time.sleep(0.3)
                continue
            h, w = frame.shape[:2]
            outputs = sess.run(None, {input_name: cd.preprocess(frame)})
            dets = cd.postprocess(outputs, w, h)
            cd.draw_overlay(frame, dets)
            ok, jpeg = _cv2.imencode(".jpg", frame, [_cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                yield jpeg.tobytes()
    finally:
        if cap is not None:
            cap.release()


@app.route("/camera/<int:cam_num>/crab")
def camera_crab(cam_num):
    if cam_num not in (1, 2) or not HAVE_CV2:
        return "", 404
    cam_url = config.get(_CAMERA_UI_URL_KEY.get(cam_num, ""), "")
    if not cam_url:
        return "", 404

    def _gen():
        try:
            for jpeg in _crab_mjpeg(cam_url):
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n"
                       b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                       + jpeg + b"\r\n")
        except (GeneratorExit, Exception):
            return

    return Response(stream_with_context(_gen()),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


# --- COLMAP frame recorder (arm camera, 10 fps, <=720p, aspect preserved) -----
COLMAP_DIR = os.path.join(_HERE, "colmap_captures")
COLMAP_STAGE = os.path.join(COLMAP_DIR, "_staging")
COLMAP_FPS = 10
COLMAP_MAX_H = 720

_colmap_lock = threading.Lock()
_colmap = {"running": False, "thread": None, "count": 0}


def _colmap_fit(frame):
    """Downscale to <=720p height keeping aspect ratio; never upscale, no distortion."""
    h, w = frame.shape[:2]
    if h <= COLMAP_MAX_H:
        return frame
    scale = COLMAP_MAX_H / float(h)
    return _cv2.resize(frame, (int(round(w * scale)), COLMAP_MAX_H),
                       interpolation=_cv2.INTER_AREA)


def _colmap_loop(snap_url):
    """Grab frames via snapshot proxy — avoids a second long-lived MJPEG pull from the Pi."""
    os.makedirs(COLMAP_STAGE, exist_ok=True)
    period = 1.0 / COLMAP_FPS
    next_t = 0.0
    try:
        while _colmap["running"]:
            now = time.time()
            if now < next_t:
                time.sleep(0.005)
                continue
            next_t = now + period
            try:
                r = _requests.get(snap_url, timeout=3)
                r.raise_for_status()
                frame = _cv2.imdecode(
                    np.frombuffer(r.content, dtype=np.uint8), _cv2.IMREAD_COLOR
                )
            except Exception:
                time.sleep(0.3)
                continue
            if frame is None:
                continue
            idx = _colmap["count"] + 1
            path = os.path.join(COLMAP_STAGE, f"frame_{idx:06d}.jpg")
            if _cv2.imwrite(path, _colmap_fit(frame)):
                _colmap["count"] = idx
    finally:
        pass


@app.route("/api/colmap/toggle", methods=["POST"])
def api_colmap_toggle():
    if not HAVE_CV2:
        return jsonify({"ok": False, "msg": "opencv not installed topside"}), 503
    if not HAVE_REQUESTS:
        return jsonify({"ok": False, "msg": "requests not installed"}), 503
    arm_url = str(config.get("arm_camera_url", "")).strip()
    if not arm_url:
        return jsonify({"ok": False, "msg": "arm camera URL not set"}), 400
    snap_url = request.host_url.rstrip("/") + "/camera/2/snapshot"
    with _colmap_lock:
        if _colmap["running"]:
            _colmap["running"] = False
            return jsonify({"ok": True, "recording": False, "staged": _colmap["count"]})
        os.makedirs(COLMAP_STAGE, exist_ok=True)
        # Continue numbering from frames already staged — toggle on/off accumulates.
        existing = [f for f in os.listdir(COLMAP_STAGE) if f.endswith(".jpg")]
        _colmap["count"] = len(existing)
        _colmap["running"] = True
        t = threading.Thread(target=_colmap_loop, args=(snap_url,),
                             daemon=True, name="colmap-rec")
        _colmap["thread"] = t
        t.start()
        return jsonify({"ok": True, "recording": True, "staged": _colmap["count"]})


@app.route("/api/colmap/save", methods=["POST"])
def api_colmap_save():
    with _colmap_lock:
        _colmap["running"] = False              # stop recording before sealing
    t = _colmap["thread"]
    if t is not None:
        t.join(timeout=2.0)
    frames = (sorted(f for f in os.listdir(COLMAP_STAGE) if f.endswith(".jpg"))
              if os.path.isdir(COLMAP_STAGE) else [])
    if not frames:
        return jsonify({"ok": False, "msg": "no frames recorded"}), 400
    dest = os.path.join(COLMAP_DIR, "colmap_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(dest, exist_ok=True)
    for f in frames:
        shutil.move(os.path.join(COLMAP_STAGE, f), os.path.join(dest, f))
    _colmap["count"] = 0
    return jsonify({"ok": True, "folder": dest, "count": len(frames)})


@app.route("/api/colmap/status")
def api_colmap_status():
    return jsonify({"recording": _colmap["running"], "staged": _colmap["count"]})


# ─────────────────────────────────────────────────────────────────────────────
# FLASK API ROUTES
# ─────────────────────────────────────────────────────────────────────────────

@app.route("/favicon.ico")
def favicon():
    return Response(status=204)


@app.context_processor
def _inject_static_ver():
    def static_ver(path: str) -> str:
        full = ROV_ROOT / "static" / path
        try:
            return str(int(full.stat().st_mtime))
        except OSError:
            return "0"

    return {"static_ver": static_ver}


@app.after_request
def _no_cache_static(response):
    if request.path.startswith("/static/"):
        response.headers["Cache-Control"] = "no-cache, must-revalidate"
    return response


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
                    ok_m, msg_m = ssh._start_mavproxy_fresh()
                    if ok_m:
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
                    f"MAVProxy TCP :{MAVPROXY_TCP_PORT} / :{MAVPROXY_ARM_TCP_PORT} not listening — "
                    f"need {MAVPROXY_ONBOARD_OUT} and {MAVPROXY_ARM_ONBOARD_OUT}"
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

            # Step 2–4: stab → arm → cam sequentially (SSH lock + MAVProxy tcpin = 1 client/port)
            cam0_dev = config.get("camera0_device", "/dev/video0")
            cam1_dev = config.get("camera1_device", "/dev/video2")
            cam_args = f"--cam0 {cam0_dev} --cam1 {cam1_dev}"

            service_specs = [
                ("mosfet", "mosfet", "onboard_mosfet", 15.0, ""),
                ("stabilization", "stab", "onboard_stab", 75.0, ""),
                ("arm_ctrl", "arm", "onboard_arm", 45.0, ""),
                ("camera", "cam", "onboard_cam", 30.0, cam_args),
            ]
            service_labels = {
                "mosfet": "mosfet_service.py (servo power rail)",
                "stabilization": "stabilization.py",
                "arm_ctrl": "new_ar.py (arm controller)",
                "camera": "camera_stream.py (MJPEG feeds)",
            }
            service_results: dict[str, tuple[bool, str]] = {}

            for step, svc, state_key, timeout, extra_args in service_specs:
                _emit_onboard_progress(
                    step, "starting",
                    f"Launching {service_labels[step]}...",
                )
                ok, msg = ssh.supervisor_start_and_wait(
                    svc, timeout_sec=timeout, extra_args=extra_args,
                )
                service_results[step] = (ok, msg)
                with _state_lock:
                    STATE[state_key] = ok
                _emit_onboard_progress(step, "done" if ok else "error", msg)
                emit_status()

            ok_mos, msg_mos = service_results.get("mosfet", (False, "missing result"))
            ok_s, msg_s = service_results.get("stabilization", (False, "missing result"))
            ok_a, msg_a = service_results.get("arm_ctrl", (False, "missing result"))
            ok_c, msg_c = service_results.get("camera", (False, "missing result"))

            core_ok = ok_m and ok_s
            if core_ok and ok_mos and ok_a and ok_c:
                summary = "✓ All onboard programs running (MAVProxy, MOSFET, stabilization, new_ar, cameras)"
            elif core_ok and ok_a and ok_c:
                parts_warn = []
                if not ok_mos:
                    parts_warn.append("MOSFET service")
                summary = (
                    "✓ ROV ready — optional failed: " + ", ".join(parts_warn)
                    if parts_warn
                    else "✓ ROV ready — cameras unavailable (check device paths / opencv install)"
                )
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
            if core_ok:
                _on_onboard_stack_ready()
            else:
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
    ssh.exec("sleep 0.75", timeout=5)
    STATE["onboard_stab"]     = False
    STATE["onboard_mosfet"]   = False
    STATE["onboard_arm"]      = False
    STATE["onboard_cam"]      = False
    STATE["onboard_mavproxy"] = False
    STATE["onboard_starting"] = False
    _reset_onboard_telemetry_state()
    emit_status()
    socketio.emit("telemetry", _telemetry_emit_payload())
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
        if not HAVE_PYSERIAL:
            pip_cmd = f'"{PYTHON}" -m pip install pyserial'
            results["arm_sender"] = {
                "ok": False,
                "msg": (
                    f"pyserial not installed for {PYTHON}. "
                    f"Run in a terminal: {pip_cmd}"
                ),
            }
            STATE["arm_running"] = False
            emit_status()
            return jsonify({"ok": False, "results": results})

        cmd = [
            PYTHON, str(ROV_ROOT / "topside" / "arm_sender.py"),
            "--ip",   config["pi_ip"],
            "--port", config["serial_port"],
            "--udp-port", str(int(config.get("arm_udp_port", 5006))),
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


@app.route("/api/arm_diagnostic", methods=["GET"])
def api_arm_diagnostic():
    tel = STATE.get("telemetry", {})
    rx = tel.get("arm_rx_count")
    arm_age = _arm_telemetry_age_sec()
    arm_tel_stale = arm_age is None or arm_age > 5.0
    hold_neutral = tel.get("arm_hold_neutral")
    checks = [
        {
            "name": "ROV mode (DRIVE/ARMED or STABILIZE)",
            "ok": _robot_armed(),
            "detail": STATE.get("mode", "disarmed"),
        },
        {
            "name": "arm_sender running (topside)",
            "ok": bool(STATE.get("arm_running")),
            "detail": "Start via Open Control" if not STATE.get("arm_running") else "OK",
        },
        {
            "name": "Onboard arm (new_ar.py on Pi)",
            "ok": bool(STATE.get("onboard_arm")),
            "detail": "Start Onboard" if not STATE.get("onboard_arm") else "OK",
        },
        {
            "name": "MOSFET service (Pi)",
            "ok": not _mosfet_enabled() or bool(STATE.get("onboard_mosfet")),
            "detail": (
                "Start Onboard"
                if _mosfet_enabled() and not STATE.get("onboard_mosfet")
                else ("N/A" if not _mosfet_enabled() else "OK")
            ),
        },
        {
            "name": "Servo power rail (MOSFET ON)",
            "ok": not _mosfet_enabled() or bool(STATE.get("mosfet_on")),
            "detail": (
                "Toggle MOSFET ON in control bar"
                if _mosfet_enabled() and not STATE.get("mosfet_on")
                else ("N/A" if not _mosfet_enabled() else "OK")
            ),
        },
        {
            "name": "Pi MAVLink to Pix6",
            "ok": bool(tel.get("arm_mavlink_ok")),
            "detail": "Check MAVProxy + onboard arm log" if not tel.get("arm_mavlink_ok") else "OK",
        },
        {
            "name": "Pi arm motion enabled",
            "ok": bool(tel.get("arm_enabled")),
            "detail": "Switch to DRIVE/ARMED" if not tel.get("arm_enabled") else "OK",
        },
        {
            "name": "UDP from arm_sender (Pi rx count)",
            "ok": not arm_tel_stale and rx is not None and int(rx) > 0,
            "detail": (
                "Waiting for arm telemetry from Pi"
                if arm_tel_stale
                else f"rx={rx if rx is not None else '?'} — plug in arm USB if 0"
            ),
        },
        {
            "name": "Arm BNO055 read fresh",
            "ok": not arm_tel_stale and bool(tel.get("arm_imu_ok")) and not bool(tel.get("arm_imu_stale")),
            "detail": (
                f"IMU read age {tel.get('arm_imu_read_age_sec')}s — check BNO055 wiring/I2C"
                if tel.get("arm_imu_stale")
                else (
                    "Waiting for BNO055"
                    if not tel.get("arm_imu_ok")
                    else "OK"
                )
            ),
        },
        {
            "name": "Not stuck in hold-neutral",
            "ok": not arm_tel_stale and hold_neutral is False,
            "detail": (
                f"Arm telemetry stale ({arm_age}s) — restart onboard or check UDP 5008"
                if arm_tel_stale
                else (
                    "Waiting for arm_sender packets"
                    if hold_neutral
                    else "OK"
                )
            ),
        },
    ]
    return jsonify({
        "ok": True,
        "checks": checks,
        "all_ok": all(c["ok"] for c in checks),
        "hint": "Mission Planner: SERVO9–16_FUNCTION=1 (RCPassThru), BRD_SAFETYENABLE=0",
        "arm_joint_us": tel.get("arm_joint_us"),
        "mode": STATE.get("mode"),
    })


@app.route("/api/arm_jog", methods=["POST"])
def api_arm_jog():
    """Pulse one joint via Pi manual PWM (connectivity test). Requires DRIVE/ARMED."""
    if not _robot_armed():
        return jsonify({"ok": False, "msg": "Switch to DRIVE/ARMED first"}), 403
    data = request.get_json(force=True) or {}
    try:
        joint_i = int(data.get("joint", 5))
    except (TypeError, ValueError):
        return jsonify({"ok": False, "msg": "Invalid joint"}), 400
    if joint_i not in JOINT_TO_AUX:
        return jsonify({"ok": False, "msg": "Joint must be 1–7"}), 400
    aux = JOINT_TO_AUX[joint_i]
    pwm = _clamp_arm_pwm(data.get("pwm", 1600))
    center = _clamp_arm_pwm(data.get("center_pwm", 1500))
    label = ARM_JOINT_NAMES[joint_i - 1]
    hold_sec = float(data.get("hold_sec", 1.0))

    ok1, msg1 = _send_pi_arm_control({
        "cmd": "manual_pwm", "enabled": True, "aux": aux, "pwm": pwm,
    })
    if not ok1:
        return jsonify({"ok": False, "msg": msg1}), 500

    def _restore():
        time.sleep(hold_sec)
        _send_pi_arm_control({
            "cmd": "manual_pwm", "enabled": True, "aux": aux, "pwm": center,
        })
        time.sleep(0.05)
        _send_pi_arm_control({"cmd": "manual_pwm", "enabled": False})

    threading.Thread(target=_restore, daemon=True, name=f"arm-jog-{label}").start()
    return jsonify({
        "ok": True,
        "msg": f"Jog {label} → {pwm} µs for {hold_sec:.1f}s",
        "joint": joint_i,
        "aux": aux,
        "pwm": pwm,
    })


@app.route("/api/arm_imu_zero", methods=["POST"])
def api_arm_imu_zero():
    ok, msg = _send_pi_arm_control({"cmd": "arm_imu_zero"})
    if not ok:
        return jsonify({"ok": False, "msg": msg}), 500
    time.sleep(0.25)
    tel = STATE.get("telemetry", {})
    offset = tel.get("arm_imu_zero_offset")
    angle = tel.get("arm_imu_angle_deg")
    if offset is not None:
        config["arm_imu_zero_offset"] = float(offset)
        save_config_file()
    emit_status()
    return jsonify({
        "ok": True,
        "msg": "Arm IMU zeroed — flat is now 0°",
        "offset_deg": offset,
        "angle_deg": angle,
    })


@app.route("/api/arm_claw_stop", methods=["GET", "POST"])
def api_arm_claw_stop():
    if request.method == "GET":
        return jsonify({
            "ok": True,
            "stop_us": int(config.get("arm_claw_stop_us", 1515)),
        })

    if not _robot_armed():
        return jsonify({"ok": False, "msg": "Arm disabled while DISARMED"}), 403

    data = request.get_json(force=True) or {}
    stop_us = data.get("stop_us")
    if stop_us is None and data.get("from_manual"):
        aux = list(STATE.get("manual_aux_pwm", list(_manual_aux_defaults())))
        if len(aux) < 7:
            return jsonify({"ok": False, "msg": "No manual AUX7 (Claw) value"}), 400
        stop_us = aux[6]
    try:
        stop_us = _clamp_arm_pwm(stop_us)
    except (TypeError, ValueError):
        return jsonify({"ok": False, "msg": "stop_us must be 500–2500"}), 400

    config["arm_claw_stop_us"] = int(stop_us)
    save_config_file()
    ok, msg = _send_pi_arm_control({"cmd": "arm_claw_stop", "stop_us": int(stop_us)})
    emit_status()
    return jsonify({
        "ok": ok,
        "msg": msg or f"Claw stop PWM set to {int(stop_us)} µs",
        "stop_us": int(stop_us),
    })


@app.route("/api/claw_hold", methods=["GET", "POST"])
def api_claw_hold():
    if request.method == "GET":
        return jsonify({
            "ok": True,
            "enabled": bool(STATE.get("claw_hold", False)),
        })

    if not _robot_armed():
        return jsonify({
            "ok": False,
            "msg": "Claw hold disabled while DISARMED",
        }), 403

    data = request.get_json(force=True) or {}
    enabled = bool(data.get("enabled", False))
    STATE["claw_hold"] = enabled
    config["claw_hold"] = enabled
    save_config_file()
    ok, msg = _send_pi_arm_control({"cmd": "claw_hold", "enabled": enabled})
    emit_status()
    return jsonify({"ok": ok, "msg": msg, "enabled": enabled})


@app.route("/api/mosfet", methods=["POST"])
def api_mosfet():
    if not _mosfet_enabled():
        return jsonify({
            "ok": False,
            "msg": "MOSFET disabled — hardware not installed",
            "mosfet_on": False,
            "mosfet_enabled": False,
        }), 400
    data = request.get_json(force=True) or {}
    state = bool(data.get("state", False))
    ok, msg = _send_mosfet_command(state)
    if not ok:
        return jsonify({"ok": False, "msg": msg, "mosfet_on": STATE.get("mosfet_on", False)}), 500
    STATE["mosfet_on"] = state
    emit_status()
    return jsonify({"ok": True, "msg": msg, "mosfet_on": state, "mosfet_enabled": True})


@app.route("/api/manual_pwm", methods=["GET"])
def api_manual_pwm_get():
    return jsonify({
        "ok": True,
        "enabled": bool(STATE.get("manual_pwm_enabled")),
        "aux_pwm": list(STATE.get("manual_aux_pwm", list(_manual_aux_defaults()))),
        "thr_pwm": list(STATE.get("manual_thr_pwm", list(_manual_thr_defaults()))),
        "aux_labels": MANUAL_AUX_LABELS,
        "thr_labels": MANUAL_THR_LABELS,
    })


def _manual_pwm_json(extra: dict | None = None) -> dict:
    """Common manual PWM fields for API responses."""
    payload = {
        "enabled": STATE.get("manual_pwm_enabled", False),
        "aux_pwm": list(STATE.get("manual_aux_pwm", list(_manual_aux_defaults()))),
        "thr_pwm": list(STATE.get("manual_thr_pwm", list(_manual_thr_defaults()))),
    }
    if extra:
        payload.update(extra)
    return payload


@app.route("/api/manual_pwm", methods=["POST"])
def api_manual_pwm():
    if not _robot_armed():
        return jsonify({
            "ok": False,
            "msg": "Manual AUX disabled while DISARMED",
            "enabled": False,
        }), 403

    # Ensure Pi arm motion is unlocked before manual override commands.
    _sync_arm_unlock()

    data = request.get_json(force=True) or {}
    action = (data.get("action") or "").strip().lower()

    if action == "toggle":
        enabled = bool(data.get("enabled"))
        ok_arm, msg_arm = _send_pi_arm_control({"cmd": "manual_pwm", "enabled": enabled})
        ok_stab, msg_stab = _send_pi_stab_control({"cmd": "manual_pwm", "enabled": enabled})
        ok = ok_arm and ok_stab
        msg = "; ".join(filter(None, [msg_arm if ok_arm else f"arm: {msg_arm}",
                                      msg_stab if ok_stab else f"stab: {msg_stab}"]))
        if ok_arm or ok_stab:
            STATE["manual_pwm_enabled"] = enabled
        emit_status()
        return jsonify({"ok": ok, "msg": msg, **_manual_pwm_json()})

    if action == "center":
        ok_arm, msg_arm = _send_pi_arm_control({
            "cmd": "manual_pwm", "center": True, "enabled": True,
        })
        ok_stab, msg_stab = _send_pi_stab_control({
            "cmd": "manual_pwm", "center": True, "enabled": True,
        })
        ok = ok_arm and ok_stab
        msg = "; ".join(filter(None, [msg_arm if ok_arm else f"arm: {msg_arm}",
                                      msg_stab if ok_stab else f"stab: {msg_stab}"]))
        if ok_arm or ok_stab:
            STATE["manual_pwm_enabled"] = True
            STATE["manual_aux_pwm"] = list(_manual_aux_defaults())
            STATE["manual_thr_pwm"] = list(_manual_thr_defaults())
        emit_status()
        return jsonify({"ok": ok, "msg": msg, **_manual_pwm_json()})

    if action == "set":
        parsed = None
        aux = data.get("aux")
        motor = data.get("motor")
        pwm = data.get("pwm")
        if aux is not None and pwm is not None:
            try:
                aux_i = int(aux)
                pwm_i = _clamp_arm_pwm(pwm)
                if 1 <= aux_i <= 7:
                    parsed = ("aux", aux_i, pwm_i,
                              f"AUX{aux_i} ({MANUAL_AUX_LABELS[aux_i - 1]})")
            except (TypeError, ValueError):
                parsed = None
        elif motor is not None and pwm is not None:
            try:
                motor_i = int(motor)
                pwm_i = _clamp_thr_pwm(pwm)
                if 1 <= motor_i <= 8:
                    parsed = ("motor", motor_i, pwm_i,
                              f"M{motor_i} ({MANUAL_THR_LABELS[motor_i - 1]})")
            except (TypeError, ValueError):
                parsed = None
        elif data.get("line"):
            parsed = _parse_manual_pwm_line(str(data.get("line")))

        if not parsed:
            return jsonify({
                "ok": False,
                "msg": (
                    "Use AUX 1–7 / J1–J6 / claw, or M1–M8 / flh etc. plus PWM "
                    "(e.g. 'J1 1500', 'M3 1600', 'flh 1600')"
                ),
            })

        kind, idx, pwm_i, label = parsed
        if kind == "aux":
            ok, msg = _send_pi_arm_control({
                "cmd": "manual_pwm",
                "enabled": True,
                "aux": idx,
                "pwm": pwm_i,
            })
            if ok:
                STATE["manual_pwm_enabled"] = True
                aux_list = list(STATE.get("manual_aux_pwm", list(_manual_aux_defaults())))
                while len(aux_list) < 7:
                    aux_list.append(1500)
                aux_list[idx - 1] = pwm_i
                STATE["manual_aux_pwm"] = aux_list
            emit_status()
            return jsonify({
                "ok": ok,
                "msg": msg,
                "kind": kind,
                "aux": idx,
                "pwm": pwm_i,
                "label": label,
                **_manual_pwm_json(),
            })

        ok, msg = _send_pi_stab_control({
            "cmd": "manual_pwm",
            "enabled": True,
            "motor": idx,
            "pwm": pwm_i,
        })
        if ok:
            STATE["manual_pwm_enabled"] = True
            thr_list = list(STATE.get("manual_thr_pwm", list(_manual_thr_defaults())))
            while len(thr_list) < 8:
                thr_list.append(NEUTRAL_THR_PWM)
            thr_list[idx - 1] = pwm_i
            STATE["manual_thr_pwm"] = thr_list
        emit_status()
        return jsonify({
            "ok": ok,
            "msg": msg,
            "kind": kind,
            "motor": idx,
            "pwm": pwm_i,
            "label": label,
            **_manual_pwm_json(),
        })

    return jsonify({"ok": False, "msg": "action must be toggle, set, or center"})


@app.route("/api/mode", methods=["POST"])
def api_mode():
    data = request.get_json(force=True) or {}
    mode = data.get("mode", "disarmed")
    if mode not in ("disarmed", "armed", "stabilize"):
        return jsonify({"ok": False, "msg": "invalid mode"})
    STATE["mode"] = mode
    if mode == "disarmed":
        STATE["ctrl_stabilize"] = False
        STATE["ctrl_depth_hold"] = False
        STATE["ctrl_yaw_hold"] = False
        STATE["manual_pwm_enabled"] = False
        STATE["manual_thr_pwm"] = list(_manual_thr_defaults())
    _apply_disarmed_arm_lockout()
    if mode in ("armed", "stabilize"):
        if _mosfet_enabled():
            _sync_arm_power()
        _sync_arm_claw_hold()
        _sync_arm_imu_cal()
        _sync_arm_claw_stop()
        _subscribe_arm_telemetry()
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
    if not _robot_armed():
        return jsonify({"ok": False, "msg": "Arm presets disabled while DISARMED"}), 403
    normalize_arm_presets()
    preset = config.get("arm_presets", {}).get(_slug_preset_name(name))
    if not preset:
        return jsonify({"ok": False, "msg": f"Unknown preset: {name}"}), 404
    try:
        ok, msg = _start_preset_sequence(_slug_preset_name(name), preset)
        if not ok:
            return jsonify({"ok": False, "msg": msg}), 409
        return jsonify({
            "ok": True,
            "msg": msg,
            "preset": name,
            "sequential": True,
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
        return jsonify({"ok": True, "recording": True, "file": str(path), "session": stamp})

    STATE["telemetry_recording"] = False
    prev = STATE.get("telemetry_record_file", "")
    emit_status()
    return jsonify({"ok": True, "recording": False, "file": prev})


@app.route("/api/video_record", methods=["POST"])
def api_video_record():
    data = request.get_json(force=True) or {}
    action = data.get("action", "toggle")
    mode = str(data.get("mode", "overlay")).strip().lower()
    if mode not in ("overlay", "raw", "both"):
        return jsonify({"ok": False, "msg": f"Unknown video mode: {mode}"}), 400

    if action == "start" or (action == "toggle" and not STATE["video_recording"]):
        VIDEO_DIR.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        STATE["video_recording"] = True
        STATE["video_record_session"] = stamp
        STATE["video_record_mode"] = mode
        STATE["video_record_files"] = []
        emit_status()
        return jsonify({
            "ok": True,
            "recording": True,
            "session": stamp,
            "mode": mode,
        })

    session = STATE.get("video_record_session", "")
    video_files = list(STATE.get("video_record_files", []))
    prev_mode = STATE.get("video_record_mode", "")
    STATE["video_recording"] = False
    STATE["video_record_mode"] = ""
    emit_status()
    return jsonify({
        "ok": True,
        "recording": False,
        "session": session,
        "mode": prev_mode,
        "video_files": video_files,
    })


@app.route("/api/video_record/upload", methods=["POST"])
def api_video_record_upload():
    session = (request.form.get("session") or "").strip()
    camera = (request.form.get("camera") or "unknown").strip().lower()
    variant = (request.form.get("variant") or "overlay").strip().lower()
    upload = request.files.get("video")
    if not session or not upload:
        return jsonify({"ok": False, "msg": "Missing session or video payload"}), 400
    if camera not in ("forward", "arm"):
        return jsonify({"ok": False, "msg": f"Unknown camera: {camera}"}), 400
    if variant not in ("overlay", "raw"):
        return jsonify({"ok": False, "msg": f"Unknown variant: {variant}"}), 400

    ext = Path(upload.filename or "").suffix.lower()
    if ext not in (".webm", ".mp4", ".mkv"):
        ext = ".webm"

    try:
        VIDEO_DIR.mkdir(parents=True, exist_ok=True)
        path = VIDEO_DIR / f"dive_{session}_{camera}_{variant}{ext}"
        upload.save(str(path))
    except Exception as e:
        return jsonify({"ok": False, "msg": str(e)}), 500

    saved = str(path)
    with _state_lock:
        files = list(STATE.get("video_record_files", []))
        if saved not in files:
            files.append(saved)
        STATE["video_record_files"] = files
    emit_status()
    return jsonify({"ok": True, "file": saved, "camera": camera, "variant": variant, "session": session})


@app.route("/api/status")
def api_status():
    with _state_lock:
        progress = list(STATE["onboard_progress"])
    return jsonify({
        "thrust_running":        STATE["thrust_running"],
        "arm_running":           STATE["arm_running"],
        "onboard_stab":          STATE["onboard_stab"],
        "onboard_mosfet":        STATE["onboard_mosfet"],
        "onboard_arm":           STATE["onboard_arm"],
        "onboard_cam":           STATE["onboard_cam"],
        "onboard_mavproxy":      STATE["onboard_mavproxy"],
        "ssh_connected":         STATE["ssh_connected"],
        "mode":                  STATE["mode"],
        "mosfet_on":             STATE["mosfet_on"],
        "mosfet_enabled":        _mosfet_enabled(),
        "telemetry_listener_ok": STATE["telemetry_listener_ok"],
        "onboard_starting":      STATE["onboard_starting"],
        "onboard_progress":      progress,
        "telemetry":             STATE["telemetry"],
        "telemetry_recording":   STATE["telemetry_recording"],
        "telemetry_record_file": STATE.get("telemetry_record_file", ""),
        "video_recording":       STATE["video_recording"],
        "video_record_session":  STATE.get("video_record_session", ""),
        "video_record_mode":     STATE.get("video_record_mode", ""),
        "video_record_files":    list(STATE.get("video_record_files", [])),
        "link_health":           _compute_link_health(),
        "arm_last_pwm":          STATE.get("arm_last_pwm"),
        "preset_running":        STATE.get("preset_running", False),
        "preset_active_name":    STATE.get("preset_active_name", ""),
        "manual_pwm_enabled":    STATE.get("manual_pwm_enabled", False),
        "manual_aux_pwm":        STATE.get("manual_aux_pwm", list(_manual_aux_defaults())),
        "manual_thr_pwm":        STATE.get("manual_thr_pwm", list(_manual_thr_defaults())),
        "claw_hold":             STATE.get("claw_hold", False),
        "arm_claw_stop_us":      int(config.get("arm_claw_stop_us", 1515)),
        "arm_motion_enabled":    _robot_armed(),
        "arm_pi_enabled":        STATE.get("telemetry", {}).get("arm_enabled"),
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
            "onboard_mosfet": STATE["onboard_mosfet"],
            "onboard_stab": STATE["onboard_stab"],
            "onboard_arm": STATE["onboard_arm"],
            "onboard_cam": STATE["onboard_cam"],
        })


@app.route("/api/onboard_log/<name>")
def api_onboard_log(name):
    allowed = {"mosfet", "stab", "arm", "cam", "colmap", "crabs"}
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
    socketio.emit("telemetry", _telemetry_emit_payload())
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
    STATE["ctrl_stabilize"] = bool(data.get("stabilize", False))
    STATE["ctrl_depth_hold"] = bool(data.get("depth_hold", False))
    STATE["ctrl_yaw_hold"] = bool(data.get("yaw_hold", False))


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
    _start_arm_telemetry_listener()
    _start_arm_telemetry_subscribe_loop()
    _start_control_sender()
    _sync_arm_power()
    _apply_disarmed_arm_lockout()
    threading.Thread(target=_monitor_loop, daemon=True).start()

    url = f"http://localhost:{args.port}"
    print(f"\n{'='*55}")
    print(f"  DreadYachet ROV Control UI")
    print(f"  Open: {url}")
    print(f"  Telemetry listening on UDP port {config['telemetry_port']}")
    print(f"  Arm IMU telemetry on UDP port {config.get('arm_telemetry_port', 5008)}")
    print(f"  Control packets → Pi UDP port {config['thrust_udp_port']}")
    print(f"{'='*55}\n")

    if not args.no_browser:
        threading.Timer(1.2, lambda: webbrowser.open(url)).start()

    socketio.run(app, host=args.host, port=args.port, debug=False, allow_unsafe_werkzeug=True)


if __name__ == "__main__":
    main()
