#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
"""
ROV Web Control UI — DreadYachet ROV Main Control System
=========================================================

A fully web-based dashboard for the DreadYachet ROV.

Features:
  - SSH to Pi → launch/monitor onboard stabilization.py + new_ar.py
  - Local process launch for thrust_sender.py + arm_sender.py
  - Live MJPEG camera feed proxying from Pi
  - Real-time telemetry via WebSocket (parsed from thrust_sender stdout)
  - Direction HUD overlay on camera feeds
  - MOSFET / servo power toggle (via UDP to Pi)
  - Drive mode selection (Disarmed / Armed / Stabilize)
  - COLMAP and Crabs sequence SSH commands
  - Battery / pressure / temperature telemetry bar

Dependencies (topside):
    pip install flask flask-socketio paramiko requests

Usage:
    python rov_ui.py
    Then open http://localhost:8080 (auto-opens in browser)

Works on Windows and Ubuntu.
"""

# ─────────────────────────────────────────────────────────────────────────────
# IMPORTS
# ─────────────────────────────────────────────────────────────────────────────

import os
import sys
import re
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

# Silence Flask startup chatter
log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)

try:
    from flask import Flask, Response, request, jsonify, render_template_string, stream_with_context
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

# ─────────────────────────────────────────────────────────────────────────────
# DEFAULT CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

DEFAULT_CONFIG = {
    "pi_ip":              "192.168.2.249",
    "pi_user":            "uruc",
    "pi_password":        "yahboom",
    "pi_ssh_port":        22,
    "pi_rov_path":        "/home/uruc/URUCDreadYachet",
    "serial_port":        "COM3" if IS_WINDOWS else "/dev/ttyACM0",
    "camera1_url":        "http://192.168.2.249:8160",
    "camera2_url":        "http://192.168.2.249:8161",
    "thrust_udp_port":    5005,
    "telemetry_port":     5006,
    "arm_udp_port":       5006,
    "mosfet_control_port": 5007,
    "colmap_command":     "python3 colmap_run.py",
    "crabs_command":      "python3 crabs.py",
}

config = DEFAULT_CONFIG.copy()

# ─────────────────────────────────────────────────────────────────────────────
# GLOBAL STATE
# ─────────────────────────────────────────────────────────────────────────────

STATE = {
    "thrust_running":      False,
    "arm_running":         False,
    "onboard_stab":        False,
    "onboard_arm":         False,
    "ssh_connected":       False,
    "ssh_error":           "",
    "mode":                "disarmed",
    "mosfet_on":           False,
    "last_telemetry_time": 0.0,
    "telemetry": {
        "rx_state":           "NO_TELEMETRY",
        "gain_percent":       100,
        "cmd_forward":        0.0,
        "cmd_lateral":        0.0,
        "cmd_yaw":            0.0,
        "cmd_vertical":       0.0,
        "stabilize":          False,
        "depth_hold_request": False,
        "depth_hold_active":  False,
        "yaw_hold_request":   False,
        "yaw_hold_active":    False,
        "depth_m":            None,
        "hold_depth_m":       None,
        "yaw_deg":            None,
        "hold_yaw_deg":       None,
        "roll_deg":           None,
        "pitch_deg":          None,
        "h_group":            0.0,
        "v_group":            0.0,
        "pressure_hpa":       None,
        "temperature_c":      None,
    },
    "logs": {
        "thrust": [],
        "arm":    [],
        "onboard_stab": [],
        "onboard_arm":  [],
    },
}

_state_lock = threading.Lock()
MAX_LOG_LINES = 200

# ─────────────────────────────────────────────────────────────────────────────
# SSH MANAGER
# ─────────────────────────────────────────────────────────────────────────────

class SSHManager:
    def __init__(self):
        self._client = None
        self._lock = threading.Lock()
        self._channel_stab = None
        self._channel_arm = None

    def connect(self, host, user, password, port=22):
        if not HAVE_PARAMIKO:
            return False, "paramiko not installed. Run: pip install paramiko"
        try:
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect(
                host, port=port, username=user, password=password,
                timeout=10, banner_timeout=15, auth_timeout=15
            )
            with self._lock:
                if self._client:
                    try: self._client.close()
                    except: pass
                self._client = client
            return True, "Connected"
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

    def exec(self, cmd, timeout=20):
        with self._lock:
            if self._client is None:
                return "", "Not connected", "not_connected"
        try:
            _, stdout, stderr = self._client.exec_command(cmd, timeout=timeout)
            out = stdout.read().decode("utf-8", errors="replace")
            err = stderr.read().decode("utf-8", errors="replace")
            return out.strip(), err.strip(), None
        except Exception as e:
            return "", "", str(e)

    def start_onboard_process(self, script_rel, log_name, extra_args=""):
        rov_path = config["pi_rov_path"]
        script = f"{rov_path}/{script_rel}"
        log_file = f"/tmp/rov_{log_name}.log"
        cmd = (
            f"cd {rov_path} && "
            f"nohup python3 {script} {extra_args} > {log_file} 2>&1 & "
            f"echo $!"
        )
        out, err, error = self.exec(cmd)
        if error:
            return False, error
        pid = out.strip()
        return True, f"PID {pid}"

    def stop_onboard_process(self, script_name):
        self.exec(f"pkill -f '{script_name}' 2>/dev/null || true")

    def is_onboard_running(self, script_name):
        out, _, error = self.exec(f"pgrep -f '{script_name}'")
        if error:
            return False
        return bool(out.strip())

    def get_onboard_log(self, log_name, lines=20):
        log_file = f"/tmp/rov_{log_name}.log"
        out, _, _ = self.exec(f"tail -n {lines} {log_file} 2>/dev/null || echo ''")
        return out

    def send_mosfet(self, state: bool):
        """Send MOSFET control packet directly via UDP from topside."""
        payload = json.dumps({"cmd": "mosfet", "state": state}).encode()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(1.0)
            s.sendto(payload, (config["pi_ip"], config["mosfet_control_port"]))
            s.close()
            return True, "sent"
        except Exception as e:
            return False, str(e)

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
        # Allow pygame without a physical display on headless setups
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
    """Read stdout from a local process, parse telemetry, emit logs."""
    for raw_line in proc.stdout:
        line = raw_line.rstrip()
        if not line:
            continue

        # Parse thrust_sender output for live telemetry
        if name == "thrust":
            _parse_thrust_line(line)

        # Store log lines
        with _state_lock:
            log_list = STATE["logs"].get(name, [])
            log_list.append(line)
            if len(log_list) > MAX_LOG_LINES:
                del log_list[:-MAX_LOG_LINES]

        socketio.emit("process_log", {"name": name, "line": line})

    # Process died
    with _state_lock:
        if name == "thrust":
            STATE["thrust_running"] = False
        elif name == "arm":
            STATE["arm_running"] = False

    emit_status()


# ─────────────────────────────────────────────────────────────────────────────
# TELEMETRY PARSER  (from thrust_sender.py printed stdout)
# ─────────────────────────────────────────────────────────────────────────────

def _parse_thrust_line(line: str):
    """
    Parse a telemetry status line printed by thrust_sender.py every 0.5 s.

    Format (pipe-separated sections):
      RX=<state> | GAIN=<n>% scale=<f> H=<f> Vgrp=<f> sum=<f> |
      CMD F=<±f> L=<±f> Y=<±f> V=<±f> |
      stab=<bool> dh_req=<bool> dh_act=<bool> yh_req=<bool> yh_act=<bool> |
      depth=<±f m|N/A> holdD=<±f m|N/A> yaw=<±f deg|N/A> holdY=<±f deg|N/A>
    """
    if "RX=" not in line or "CMD" not in line:
        return

    secs = line.split(" | ")
    if len(secs) < 5:
        return

    tel = STATE["telemetry"]

    # ── Section 0: RX=<state>
    m = re.search(r"RX=(\S+)", secs[0])
    if m:
        tel["rx_state"] = m.group(1)

    # ── Section 1: GAIN / H group / V group
    for part in secs[1].split():
        k, _, v = part.partition("=")
        try:
            if k == "GAIN":
                tel["gain_percent"] = int(v.rstrip("%"))
            elif k == "H":
                tel["h_group"] = float(v)
            elif k == "Vgrp":
                tel["v_group"] = float(v)
        except ValueError:
            pass

    # ── Section 2: CMD F= L= Y= V=
    for part in secs[2].split():
        k, _, v = part.partition("=")
        try:
            if k == "F":   tel["cmd_forward"]  = float(v)
            elif k == "L": tel["cmd_lateral"]  = float(v)
            elif k == "Y": tel["cmd_yaw"]      = float(v)
            elif k == "V": tel["cmd_vertical"] = float(v)
        except ValueError:
            pass

    # ── Section 3: boolean flags
    for part in secs[3].split():
        k, _, v = part.partition("=")
        bval = v == "True"
        if k == "stab":       tel["stabilize"]          = bval
        elif k == "dh_req":   tel["depth_hold_request"] = bval
        elif k == "dh_act":   tel["depth_hold_active"]  = bval
        elif k == "yh_req":   tel["yaw_hold_request"]   = bval
        elif k == "yh_act":   tel["yaw_hold_active"]    = bval

    # ── Section 4: depth / yaw with units, may be "N/A"
    s4 = secs[4]

    def _parse_unit(text, key_pat, unit):
        m = re.search(rf"\b{key_pat}=(N/A|[+-]?[\d.]+) {unit}", text)
        if not m:
            return None
        v = m.group(1)
        return None if v == "N/A" else float(v)

    tel["depth_m"]     = _parse_unit(s4, "depth", "m")
    tel["hold_depth_m"]= _parse_unit(s4, "holdD", "m")
    tel["yaw_deg"]     = _parse_unit(s4, "yaw",   "deg")
    tel["hold_yaw_deg"]= _parse_unit(s4, "holdY", "deg")

    STATE["last_telemetry_time"] = time.time()
    socketio.emit("telemetry", dict(tel))


# ─────────────────────────────────────────────────────────────────────────────
# OPTIONAL: Full telemetry via shared UDP listener (SO_REUSEADDR)
# Receives the same packets as thrust_sender on port 5006 for richer data
# (roll, pitch, pressure, temperature). Non-essential; falls back gracefully.
# ─────────────────────────────────────────────────────────────────────────────

def _start_shared_telemetry_listener():
    """Try to co-receive telemetry UDP on port 5006 for extra fields."""
    def _listen():
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if hasattr(socket, "SO_REUSEPORT"):
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            s.bind(("0.0.0.0", config["telemetry_port"]))
            s.settimeout(1.0)
        except Exception:
            return  # Port already claimed exclusively; just skip

        while True:
            try:
                data, _ = s.recvfrom(4096)
                try:
                    pkt = json.loads(data.decode("utf-8"))
                except Exception:
                    continue

                tel = STATE["telemetry"]
                for field in ("roll_deg", "pitch_deg", "pressure_hpa",
                              "pressure_temperature_c"):
                    if field in pkt:
                        ui_key = "temperature_c" if field == "pressure_temperature_c" else field
                        tel[ui_key] = pkt[field]

                STATE["last_telemetry_time"] = time.time()
                socketio.emit("telemetry", dict(tel))

            except socket.timeout:
                pass
            except Exception:
                pass

    t = threading.Thread(target=_listen, daemon=True)
    t.start()


# ─────────────────────────────────────────────────────────────────────────────
# BACKGROUND MONITOR: updates process status → emits to UI every second
# ─────────────────────────────────────────────────────────────────────────────

def _monitor_loop():
    while True:
        time.sleep(1.0)
        STATE["thrust_running"] = is_local_running("thrust")
        STATE["arm_running"]    = is_local_running("arm")

        if ssh.is_connected():
            STATE["ssh_connected"]  = True
            STATE["onboard_stab"]   = ssh.is_onboard_running("stabilization.py")
            STATE["onboard_arm"]    = ssh.is_onboard_running("new_ar.py")
        else:
            STATE["ssh_connected"] = False
            STATE["onboard_stab"]  = False
            STATE["onboard_arm"]   = False

        tel_age = time.time() - STATE["last_telemetry_time"]
        if tel_age > 2.0:
            STATE["telemetry"]["rx_state"] = "NO_TELEMETRY"

        emit_status()


def emit_status():
    socketio.emit("status", {
        "thrust_running":  STATE["thrust_running"],
        "arm_running":     STATE["arm_running"],
        "onboard_stab":    STATE["onboard_stab"],
        "onboard_arm":     STATE["onboard_arm"],
        "ssh_connected":   STATE["ssh_connected"],
        "ssh_error":       STATE["ssh_error"],
        "mode":            STATE["mode"],
        "mosfet_on":       STATE["mosfet_on"],
    })


# ─────────────────────────────────────────────────────────────────────────────
# CAMERA PROXY
# ─────────────────────────────────────────────────────────────────────────────

def _make_no_signal_mjpeg():
    """Return a minimal MJPEG no-signal frame using only stdlib."""
    # Tiny 1×1 gray JPEG bytes (hardcoded, no PIL needed)
    # We return 'None' and let the frontend handle it via CSS placeholder
    return None


@app.route("/camera/<int:cam_num>")
def camera_stream(cam_num):
    if cam_num not in (1, 2):
        return "", 404

    cam_url = config.get(f"camera{cam_num}_url", "")

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


# ─────────────────────────────────────────────────────────────────────────────
# FLASK API ROUTES
# ─────────────────────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return render_template_string(HTML_TEMPLATE)


@app.route("/api/config", methods=["GET", "POST"])
def api_config():
    global config
    if request.method == "POST":
        data = request.get_json(force=True) or {}
        for k, v in data.items():
            if k in config:
                config[k] = v
        return jsonify({"ok": True, "config": config})
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
    ssh.disconnect()
    STATE["ssh_connected"] = False
    STATE["ssh_error"] = ""
    emit_status()
    return jsonify({"ok": True})


@app.route("/api/onboard/start", methods=["POST"])
def api_start_onboard():
    if not ssh.is_connected():
        return jsonify({"ok": False, "msg": "SSH not connected"})

    results = {}

    ok_s, msg_s = ssh.start_onboard_process(
        "onboard/stabilization.py", "stab"
    )
    results["stabilization"] = {"ok": ok_s, "msg": msg_s}
    STATE["onboard_stab"] = ok_s

    ok_a, msg_a = ssh.start_onboard_process(
        "onboard/new_ar.py", "arm"
    )
    results["new_ar"] = {"ok": ok_a, "msg": msg_a}
    STATE["onboard_arm"] = ok_a

    emit_status()
    return jsonify({"ok": ok_s and ok_a, "results": results})


@app.route("/api/onboard/stop", methods=["POST"])
def api_stop_onboard():
    ssh.stop_onboard_process("stabilization.py")
    ssh.stop_onboard_process("new_ar.py")
    STATE["onboard_stab"] = False
    STATE["onboard_arm"] = False
    emit_status()
    return jsonify({"ok": True})


@app.route("/api/topside/start", methods=["POST"])
def api_start_topside():
    data = request.get_json(force=True) or {}
    for k in ("pi_ip", "serial_port"):
        if k in data:
            config[k] = data[k]

    results = {}

    if not is_local_running("thrust"):
        cmd = [PYTHON, str(ROV_ROOT / "topside" / "thrust_sender.py"), config["pi_ip"]]
        ok, msg = start_local_process("thrust", cmd, cwd=ROV_ROOT)
        results["thrust_sender"] = {"ok": ok, "msg": msg}
        STATE["thrust_running"] = ok
    else:
        results["thrust_sender"] = {"ok": True, "msg": "already running"}

    if not is_local_running("arm"):
        cmd = [
            PYTHON, str(ROV_ROOT / "topside" / "arm_sender.py"),
            "--ip",   config["pi_ip"],
            "--port", config["serial_port"],
        ]
        ok, msg = start_local_process("arm", cmd, cwd=ROV_ROOT)
        results["arm_sender"] = {"ok": ok, "msg": msg}
        STATE["arm_running"] = ok
    else:
        results["arm_sender"] = {"ok": True, "msg": "already running"}

    emit_status()
    return jsonify({"ok": True, "results": results})


@app.route("/api/topside/stop", methods=["POST"])
def api_stop_topside():
    stop_local_process("thrust")
    stop_local_process("arm")
    STATE["thrust_running"] = False
    STATE["arm_running"] = False
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


@app.route("/api/status")
def api_status():
    return jsonify({
        "thrust_running": STATE["thrust_running"],
        "arm_running":    STATE["arm_running"],
        "onboard_stab":   STATE["onboard_stab"],
        "onboard_arm":    STATE["onboard_arm"],
        "ssh_connected":  STATE["ssh_connected"],
        "mode":           STATE["mode"],
        "mosfet_on":      STATE["mosfet_on"],
        "telemetry":      STATE["telemetry"],
    })


@app.route("/api/logs/<name>")
def api_logs(name):
    with _state_lock:
        lines = list(STATE["logs"].get(name, []))
    return jsonify({"lines": lines})


@app.route("/api/onboard_log/<name>")
def api_onboard_log(name):
    allowed = {"stab", "arm", "colmap", "crabs"}
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


@socketio.on("request_status")
def on_request_status():
    emit_status()


# ─────────────────────────────────────────────────────────────────────────────
# HTML TEMPLATE
# ─────────────────────────────────────────────────────────────────────────────

HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1.0"/>
<title>DreadYachet ROV</title>
<script src="/socket.io/socket.io.js"></script>
<style>
:root {
  --bg:      #080b12;
  --bg2:     #0e1220;
  --bg3:     #141828;
  --border:  #1c2238;
  --border2: #243060;
  --accent:  #00d4ff;
  --accent2: #7b5ea7;
  --green:   #00e08a;
  --amber:   #ffb320;
  --red:     #ff3d5a;
  --text:    #dde2f0;
  --dim:     #6b7390;
  --mono:    "JetBrains Mono","Consolas","Courier New",monospace;
}
*,*::before,*::after { box-sizing:border-box; margin:0; padding:0; }
html,body { height:100%; background:var(--bg); color:var(--text);
            font-family:system-ui,-apple-system,"Segoe UI","Ubuntu",sans-serif;
            font-size:14px; overflow:hidden; }

/* ── scrollbar ── */
::-webkit-scrollbar { width:6px; height:6px; }
::-webkit-scrollbar-track { background:var(--bg2); }
::-webkit-scrollbar-thumb { background:var(--border2); border-radius:3px; }

/* ── views ── */
.view { display:none; height:100vh; flex-direction:column; }
.view.active { display:flex; }

/* ════════════════════════════════════════
   LAUNCH SCREEN
   ════════════════════════════════════════ */
#launch {
  overflow-y:auto;
  align-items:center;
  justify-content:flex-start;
  padding:32px 16px 48px;
  gap:24px;
}
.launch-header { text-align:center; }
.launch-header h1 {
  font-size:2.2rem; font-weight:700; letter-spacing:-0.5px;
  background:linear-gradient(135deg,#00d4ff,#7b5ea7);
  -webkit-background-clip:text; -webkit-text-fill-color:transparent;
}
.launch-header p { color:var(--dim); margin-top:6px; }

.config-grid {
  display:grid;
  grid-template-columns:repeat(auto-fill, minmax(260px,1fr));
  gap:12px;
  width:100%; max-width:860px;
}
.config-group {
  background:var(--bg2); border:1px solid var(--border);
  border-radius:10px; padding:16px;
}
.config-group h3 { font-size:0.75rem; text-transform:uppercase;
  letter-spacing:1px; color:var(--accent); margin-bottom:12px; }
.field { display:flex; flex-direction:column; gap:4px; margin-bottom:10px; }
.field:last-child { margin-bottom:0; }
.field label { font-size:0.72rem; color:var(--dim); }
.field input {
  background:var(--bg3); border:1px solid var(--border); border-radius:6px;
  color:var(--text); padding:7px 10px; font-size:0.85rem;
  font-family:var(--mono); outline:none; transition:border-color .2s;
}
.field input:focus { border-color:var(--accent); }
.field input[type=password] { letter-spacing:2px; }

/* ── launch action panels ── */
.launch-panels {
  display:grid; grid-template-columns:1fr 1fr;
  gap:16px; width:100%; max-width:860px;
}
@media(max-width:600px){ .launch-panels { grid-template-columns:1fr; } }

.panel {
  background:var(--bg2); border:1px solid var(--border);
  border-radius:12px; padding:20px; display:flex; flex-direction:column; gap:12px;
}
.panel h2 { font-size:1rem; font-weight:600; }
.panel-desc { font-size:0.8rem; color:var(--dim); line-height:1.5; }

.program-status { display:flex; flex-direction:column; gap:6px; }
.prog-row {
  display:flex; align-items:center; gap:8px;
  font-size:0.8rem; font-family:var(--mono);
}
.dot { width:8px; height:8px; border-radius:50%; background:var(--dim);
       flex-shrink:0; transition:background .3s; }
.dot.running { background:var(--green); box-shadow:0 0 6px var(--green); }
.dot.error   { background:var(--red);   box-shadow:0 0 6px var(--red); }

/* ── buttons ── */
.btn {
  display:inline-flex; align-items:center; justify-content:center;
  gap:6px; padding:9px 18px; border-radius:8px; border:none;
  font-size:0.85rem; font-weight:600; cursor:pointer;
  transition:all .15s; white-space:nowrap;
}
.btn-primary   { background:var(--accent);  color:#000; }
.btn-primary:hover { filter:brightness(1.15); transform:translateY(-1px); }
.btn-secondary { background:var(--bg3); border:1px solid var(--border2);
                 color:var(--text); }
.btn-secondary:hover { border-color:var(--accent); color:var(--accent); }
.btn-danger    { background:var(--red); color:#fff; }
.btn-danger:hover { filter:brightness(1.15); }
.btn-success   { background:var(--green); color:#000; }
.btn-success:hover { filter:brightness(1.1); }
.btn-amber     { background:var(--amber); color:#000; }
.btn:disabled  { opacity:0.45; cursor:not-allowed; transform:none; }

.btn-row { display:flex; gap:8px; flex-wrap:wrap; }

.proceed-row { width:100%; max-width:860px; text-align:center; }
.proceed-row .btn { padding:12px 40px; font-size:1rem; }

/* ── ssh status pill ── */
.ssh-badge {
  display:inline-flex; align-items:center; gap:6px;
  padding:4px 12px; border-radius:20px;
  font-size:0.75rem; font-weight:600;
  background:var(--bg3); border:1px solid var(--border);
}
.ssh-badge.ok { border-color:var(--green); color:var(--green); }
.ssh-badge.err { border-color:var(--red); color:var(--red); }

/* ════════════════════════════════════════
   CONTROL SCREEN
   ════════════════════════════════════════ */
#control { height:100vh; display:none; flex-direction:column; overflow:hidden; }
#control.active { display:flex; }

/* ── top bar ── */
.topbar {
  display:flex; align-items:center; gap:12px; padding:0 16px;
  height:44px; background:var(--bg2); border-bottom:1px solid var(--border);
  flex-shrink:0;
}
.topbar-title { font-weight:700; font-size:0.95rem; color:var(--accent);
  white-space:nowrap; }
.topbar-spacer { flex:1; }
.status-pill {
  display:flex; align-items:center; gap:5px;
  padding:3px 10px; border-radius:20px;
  font-size:0.72rem; font-weight:600;
  border:1px solid var(--border); background:var(--bg3); color:var(--dim);
}
.status-pill.ok   { border-color:var(--green); color:var(--green); }
.status-pill.warn { border-color:var(--amber); color:var(--amber); }
.status-pill.err  { border-color:var(--red);   color:var(--red); }
.status-dot { width:6px; height:6px; border-radius:50%; background:currentColor; }

/* ── camera section ── */
.cameras {
  flex:1; display:grid; grid-template-columns:1fr 1fr;
  gap:2px; background:var(--border); overflow:hidden; min-height:0;
}
@media(max-width:700px){ .cameras { grid-template-columns:1fr; } }

.cam-wrap {
  position:relative; background:#000; overflow:hidden;
  display:flex; align-items:center; justify-content:center;
}
.cam-img {
  width:100%; height:100%; object-fit:contain;
  display:block;
}
.cam-no-signal {
  position:absolute; inset:0; display:flex; flex-direction:column;
  align-items:center; justify-content:center; gap:8px;
  background:var(--bg); color:var(--dim);
}
.cam-no-signal .ns-icon { font-size:2.5rem; opacity:0.4; }
.cam-no-signal .ns-text { font-size:0.75rem; letter-spacing:2px;
  text-transform:uppercase; opacity:0.5; }

/* ── HUD canvas overlaid on camera ── */
.cam-hud {
  position:absolute; inset:0; pointer-events:none;
}

/* ── camera label ── */
.cam-label {
  position:absolute; top:8px; left:10px;
  font-size:0.65rem; letter-spacing:1.5px; text-transform:uppercase;
  color:rgba(0,212,255,0.7); font-weight:700; pointer-events:none;
}

/* ── telemetry overlay on cam ── */
.cam-telemetry {
  position:absolute; top:8px; right:10px;
  font-family:var(--mono); font-size:0.68rem; line-height:1.8;
  color:rgba(220,230,255,0.8);
  text-shadow:0 1px 4px #000; pointer-events:none; text-align:right;
}
.cam-telemetry .val-hi  { color:rgba(0,224,138,0.9); }
.cam-telemetry .val-warn{ color:rgba(255,179,32,0.9); }

/* ── control bar ── */
.controlbar {
  display:flex; align-items:center; gap:6px;
  padding:6px 12px; background:var(--bg2);
  border-top:1px solid var(--border); flex-shrink:0; flex-wrap:wrap;
}
.control-label {
  font-size:0.65rem; text-transform:uppercase; letter-spacing:1px;
  color:var(--dim); margin-right:2px;
}
.ctrl-sep { width:1px; height:24px; background:var(--border); margin:0 4px; }

/* Mode buttons */
.mode-btn {
  padding:6px 14px; border-radius:6px; border:1px solid var(--border);
  background:var(--bg3); color:var(--dim); font-size:0.78rem; font-weight:600;
  cursor:pointer; transition:all .15s;
}
.mode-btn:hover { border-color:var(--accent); color:var(--accent); }
.mode-btn.active-disarmed { border-color:var(--red);   color:var(--red);   background:rgba(255,61,90,.12); }
.mode-btn.active-armed    { border-color:var(--amber); color:var(--amber); background:rgba(255,179,32,.12); }
.mode-btn.active-stabilize{ border-color:var(--green); color:var(--green); background:rgba(0,224,138,.12); }

/* MOSFET toggle */
.mosfet-toggle {
  display:flex; align-items:center; gap:8px;
  padding:5px 12px; border-radius:6px;
  border:1px solid var(--border); background:var(--bg3);
  cursor:pointer; transition:all .2s;
}
.mosfet-toggle.on { border-color:var(--green); background:rgba(0,224,138,.1); }
.mosfet-toggle .mosfet-indicator {
  width:10px; height:10px; border-radius:50%;
  background:var(--dim); transition:background .2s;
}
.mosfet-toggle.on .mosfet-indicator {
  background:var(--green); box-shadow:0 0 8px var(--green);
}
.mosfet-toggle span { font-size:0.78rem; font-weight:600; color:var(--dim); }
.mosfet-toggle.on span { color:var(--green); }

.action-btn {
  padding:6px 14px; border-radius:6px; border:1px solid var(--border2);
  background:var(--bg3); color:var(--accent2); font-size:0.78rem; font-weight:600;
  cursor:pointer; transition:all .15s;
}
.action-btn:hover { background:var(--accent2); color:#fff; }
.action-btn:active { transform:scale(0.97); }

/* ── telemetry bar ── */
.telembar {
  display:flex; align-items:center; gap:0;
  padding:5px 12px; background:var(--bg3);
  border-top:1px solid var(--border);
  font-family:var(--mono); font-size:0.7rem; flex-shrink:0;
  overflow-x:auto; white-space:nowrap;
}
.telem-cell {
  display:flex; flex-direction:column; padding:2px 10px;
  border-right:1px solid var(--border);
}
.telem-cell:last-child { border-right:none; }
.telem-cell .tc-label { font-size:0.6rem; color:var(--dim); text-transform:uppercase;
  letter-spacing:0.8px; }
.telem-cell .tc-val   { font-size:0.8rem; color:var(--text); font-weight:600; }
.telem-cell .tc-val.good { color:var(--green); }
.telem-cell .tc-val.warn { color:var(--amber); }
.telem-cell .tc-val.bad  { color:var(--red); }

/* ── log drawer (bottom of control screen) ── */
.log-drawer {
  position:fixed; bottom:0; left:0; right:0;
  height:180px; background:var(--bg2);
  border-top:2px solid var(--accent);
  transform:translateY(100%); transition:transform .25s;
  display:flex; flex-direction:column; z-index:100;
}
.log-drawer.open { transform:translateY(0); }
.log-header {
  display:flex; align-items:center; padding:6px 12px;
  border-bottom:1px solid var(--border); flex-shrink:0;
  font-size:0.75rem; font-weight:600; color:var(--accent);
}
.log-tabs { display:flex; gap:4px; margin-left:12px; }
.log-tab {
  padding:2px 10px; border-radius:4px; cursor:pointer;
  font-size:0.7rem; background:var(--bg3); color:var(--dim);
  border:1px solid var(--border);
}
.log-tab.active { background:var(--accent); color:#000; }
.log-close { margin-left:auto; cursor:pointer; color:var(--dim);
  font-size:1rem; padding:2px 6px; border-radius:4px; }
.log-close:hover { color:var(--text); }
.log-content {
  flex:1; overflow-y:auto; padding:8px 12px;
  font-family:var(--mono); font-size:0.7rem; line-height:1.6;
  color:var(--dim);
}
.log-line { white-space:pre-wrap; word-break:break-all; }
.log-line.err { color:var(--red); }
.log-line.ok  { color:var(--green); }

/* ── toast notifications ── */
.toast-container {
  position:fixed; top:52px; right:12px; z-index:200;
  display:flex; flex-direction:column; gap:6px;
}
.toast {
  padding:8px 14px; border-radius:8px; font-size:0.8rem;
  border-left:3px solid var(--accent);
  background:var(--bg2); box-shadow:0 4px 16px rgba(0,0,0,.5);
  animation:fadeIn .2s ease;
  max-width:300px;
}
.toast.err { border-color:var(--red); }
.toast.ok  { border-color:var(--green); }
@keyframes fadeIn { from{opacity:0;transform:translateX(20px)} to{opacity:1;transform:none} }
</style>
</head>
<body>

<!-- ══════════════════════════════════════════════════
     LAUNCH SCREEN
     ══════════════════════════════════════════════════ -->
<div id="launch" class="view active">

  <div class="launch-header">
    <h1>DreadYachet ROV</h1>
    <p>Mission Control &amp; Launch System</p>
  </div>

  <!-- Config grid -->
  <div class="config-grid">

    <div class="config-group">
      <h3>Pi Connection</h3>
      <div class="field"><label>Pi IP Address</label>
        <input id="cfg-pi_ip" type="text" value="192.168.2.249"/></div>
      <div class="field"><label>SSH User</label>
        <input id="cfg-pi_user" type="text" value="uruc"/></div>
      <div class="field"><label>SSH Password</label>
        <input id="cfg-pi_password" type="password" value="yahboom"/></div>
      <div class="field"><label>SSH Port</label>
        <input id="cfg-pi_ssh_port" type="text" value="22"/></div>
      <div class="field"><label>ROV Project Path on Pi</label>
        <input id="cfg-pi_rov_path" type="text" value="/home/uruc/URUCDreadYachet"/></div>
      <div class="btn-row">
        <button class="btn btn-secondary" onclick="sshConnect()">Connect SSH</button>
        <span id="ssh-status" class="ssh-badge">Disconnected</span>
      </div>
    </div>

    <div class="config-group">
      <h3>Topside Hardware</h3>
      <div class="field"><label>Serial Port (Arm Controller)</label>
        <input id="cfg-serial_port" type="text" value="/dev/ttyACM0"/></div>
      <div class="field"><label>Camera 1 URL (MJPEG)</label>
        <input id="cfg-camera1_url" type="text" value="http://192.168.2.249:8160"/></div>
      <div class="field"><label>Camera 2 URL (MJPEG)</label>
        <input id="cfg-camera2_url" type="text" value="http://192.168.2.249:8161"/></div>
    </div>

    <div class="config-group">
      <h3>Network Ports</h3>
      <div class="field"><label>Thrust UDP Port (→ Pi)</label>
        <input id="cfg-thrust_udp_port" type="text" value="5005"/></div>
      <div class="field"><label>Telemetry Port (← Pi)</label>
        <input id="cfg-telemetry_port" type="text" value="5006"/></div>
      <div class="field"><label>Arm UDP Port (→ Pi)</label>
        <input id="cfg-arm_udp_port" type="text" value="5006"/></div>
      <div class="field"><label>MOSFET Control Port</label>
        <input id="cfg-mosfet_control_port" type="text" value="5007"/></div>
    </div>

    <div class="config-group">
      <h3>Onboard Commands</h3>
      <div class="field"><label>COLMAP Command (on Pi)</label>
        <input id="cfg-colmap_command" type="text" value="python3 colmap_run.py"/></div>
      <div class="field"><label>Crabs Command (on Pi)</label>
        <input id="cfg-crabs_command" type="text" value="python3 crabs.py"/></div>
    </div>

  </div><!-- /config-grid -->

  <!-- Launch panels -->
  <div class="launch-panels">

    <!-- Onboard panel -->
    <div class="panel">
      <h2>🔵 Onboard Programs <small style="font-size:.7rem;color:var(--dim)">(Pi)</small></h2>
      <p class="panel-desc">Launches <code>stabilization.py</code> (thruster control) and
        <code>new_ar.py</code> (arm + MOSFET control) on the ROV Pi via SSH.</p>
      <div class="program-status">
        <div class="prog-row">
          <div class="dot" id="dot-stab"></div>
          <span>stabilization.py</span>
        </div>
        <div class="prog-row">
          <div class="dot" id="dot-arm"></div>
          <span>new_ar.py</span>
        </div>
      </div>
      <div class="btn-row">
        <button class="btn btn-primary" id="btn-start-onboard" onclick="startOnboard()">
          Start Onboard
        </button>
        <button class="btn btn-danger" onclick="stopOnboard()">Stop</button>
      </div>
      <div id="onboard-msg" style="font-size:.75rem;color:var(--dim);min-height:18px;font-family:var(--mono)"></div>
    </div>

    <!-- Topside panel -->
    <div class="panel">
      <h2>🟡 Topside Programs <small style="font-size:.7rem;color:var(--dim)">(this PC)</small></h2>
      <p class="panel-desc">Launches <code>thrust_sender.py</code> (joystick → thrusters) and
        <code>arm_sender.py</code> (serial → arm) on this computer.</p>
      <div class="program-status">
        <div class="prog-row">
          <div class="dot" id="dot-thrust"></div>
          <span>thrust_sender.py</span>
        </div>
        <div class="prog-row">
          <div class="dot" id="dot-armlocal"></div>
          <span>arm_sender.py</span>
        </div>
      </div>
      <div class="btn-row">
        <button class="btn btn-primary" id="btn-start-topside" onclick="startTopside()">
          Start Topside
        </button>
        <button class="btn btn-danger" onclick="stopTopside()">Stop</button>
      </div>
      <div id="topside-msg" style="font-size:.75rem;color:var(--dim);min-height:18px;font-family:var(--mono)"></div>
    </div>

  </div><!-- /launch-panels -->

  <div class="proceed-row">
    <button class="btn btn-success" onclick="openControl()" id="btn-proceed">
      Open Control Screen →
    </button>
  </div>

</div><!-- /launch -->

<!-- ══════════════════════════════════════════════════
     CONTROL SCREEN
     ══════════════════════════════════════════════════ -->
<div id="control" class="view">

  <!-- Top bar -->
  <div class="topbar">
    <span class="topbar-title">⚓ DreadYachet ROV</span>

    <span id="pill-connection" class="status-pill">
      <span class="status-dot"></span> SSH: --
    </span>
    <span id="pill-telemetry" class="status-pill">
      <span class="status-dot"></span> Telem: --
    </span>
    <span id="pill-mode" class="status-pill">
      <span class="status-dot"></span> Mode: --
    </span>

    <span class="topbar-spacer"></span>

    <span style="font-family:var(--mono);font-size:.75rem;color:var(--dim)">
      GAIN: <span id="tb-gain" style="color:var(--accent)">--</span>%
    </span>
    <button class="btn btn-secondary" style="padding:4px 12px;font-size:.75rem"
            onclick="toggleLog()">Logs</button>
    <button class="btn btn-secondary" style="padding:4px 12px;font-size:.75rem"
            onclick="showLaunch()">← Launch</button>
  </div>

  <!-- Camera feeds -->
  <div class="cameras">

    <!-- Camera 1 -->
    <div class="cam-wrap">
      <div class="cam-no-signal" id="no-sig-1">
        <div class="ns-icon">📡</div>
        <div class="ns-text">No Signal — Camera 1</div>
      </div>
      <img id="cam1" class="cam-img" style="display:none" alt="Camera 1"/>
      <canvas id="hud1" class="cam-hud"></canvas>
      <div class="cam-label">CAM 1 / FORWARD</div>
      <div class="cam-telemetry" id="cam1-tel">
        <div>DEPTH <span class="val-hi" id="c1-depth">--</span>m</div>
        <div>HOLD  <span id="c1-hold-d">--</span>m</div>
        <div>YAW   <span class="val-hi" id="c1-yaw">--</span>°</div>
      </div>
    </div>

    <!-- Camera 2 -->
    <div class="cam-wrap">
      <div class="cam-no-signal" id="no-sig-2">
        <div class="ns-icon">📡</div>
        <div class="ns-text">No Signal — Camera 2</div>
      </div>
      <img id="cam2" class="cam-img" style="display:none" alt="Camera 2"/>
      <canvas id="hud2" class="cam-hud"></canvas>
      <div class="cam-label">CAM 2 / SIDE</div>
      <div class="cam-telemetry" id="cam2-tel">
        <div>ROLL  <span class="val-hi" id="c2-roll">--</span>°</div>
        <div>PITCH <span class="val-hi" id="c2-pitch">--</span>°</div>
        <div>STAB  <span id="c2-stab">--</span></div>
      </div>
    </div>

  </div><!-- /cameras -->

  <!-- Control bar -->
  <div class="controlbar">

    <span class="control-label">Power</span>
    <div class="mosfet-toggle" id="mosfet-toggle" onclick="toggleMosfet()">
      <div class="mosfet-indicator"></div>
      <span id="mosfet-label">MOSFET OFF</span>
    </div>

    <div class="ctrl-sep"></div>

    <span class="control-label">Mode</span>
    <button class="mode-btn active-disarmed" id="mode-disarmed"
            onclick="setMode('disarmed')">DISARMED</button>
    <button class="mode-btn" id="mode-armed"
            onclick="setMode('armed')">DRIVE / ARMED</button>
    <button class="mode-btn" id="mode-stabilize"
            onclick="setMode('stabilize')">STABILIZE</button>

    <div class="ctrl-sep"></div>

    <span class="control-label">Actions</span>
    <button class="action-btn" onclick="startColmap()">▶ COLMAP</button>
    <button class="action-btn" onclick="startCrabs()">🦀 CRABS</button>

  </div><!-- /controlbar -->

  <!-- Telemetry bar -->
  <div class="telembar">
    <div class="telem-cell">
      <span class="tc-label">State</span>
      <span class="tc-val" id="tel-state">--</span>
    </div>
    <div class="telem-cell">
      <span class="tc-label">Depth</span>
      <span class="tc-val" id="tel-depth">--</span>
    </div>
    <div class="telem-cell">
      <span class="tc-label">Hold D</span>
      <span class="tc-val" id="tel-hold-d">--</span>
    </div>
    <div class="telem-cell">
      <span class="tc-label">Yaw</span>
      <span class="tc-val" id="tel-yaw">--</span>
    </div>
    <div class="telem-cell">
      <span class="tc-label">Hold Y</span>
      <span class="tc-val" id="tel-hold-y">--</span>
    </div>
    <div class="telem-cell">
      <span class="tc-label">Roll</span>
      <span class="tc-val" id="tel-roll">--</span>
    </div>
    <div class="telem-cell">
      <span class="tc-label">Pitch</span>
      <span class="tc-val" id="tel-pitch">--</span>
    </div>
    <div class="telem-cell">
      <span class="tc-label">H Grp</span>
      <span class="tc-val" id="tel-hgrp">--</span>
    </div>
    <div class="telem-cell">
      <span class="tc-label">V Grp</span>
      <span class="tc-val" id="tel-vgrp">--</span>
    </div>
    <div class="telem-cell">
      <span class="tc-label">Pressure</span>
      <span class="tc-val" id="tel-press">--</span>
    </div>
    <div class="telem-cell">
      <span class="tc-label">Temp</span>
      <span class="tc-val" id="tel-temp">--</span>
    </div>
    <div class="telem-cell">
      <span class="tc-label">Depth Hold</span>
      <span class="tc-val" id="tel-dh">--</span>
    </div>
    <div class="telem-cell">
      <span class="tc-label">Yaw Hold</span>
      <span class="tc-val" id="tel-yh">--</span>
    </div>
    <div class="telem-cell">
      <span class="tc-label">Thrust →Pi</span>
      <span class="tc-val" id="tel-thrust-proc">--</span>
    </div>
    <div class="telem-cell">
      <span class="tc-label">Arm →Pi</span>
      <span class="tc-val" id="tel-arm-proc">--</span>
    </div>
  </div>

</div><!-- /control -->

<!-- Log drawer -->
<div id="log-drawer" class="log-drawer">
  <div class="log-header">
    PROCESS LOGS
    <div class="log-tabs">
      <div class="log-tab active" onclick="switchLog('thrust')" id="lt-thrust">Thrust</div>
      <div class="log-tab" onclick="switchLog('arm')" id="lt-arm">Arm</div>
      <div class="log-tab" onclick="switchLog('onboard_stab')" id="lt-stab">Onboard Stab</div>
      <div class="log-tab" onclick="switchLog('onboard_arm')" id="lt-arm2">Onboard Arm</div>
    </div>
    <span class="log-close" onclick="toggleLog()">✕</span>
  </div>
  <div class="log-content" id="log-content"></div>
</div>

<!-- Toast container -->
<div class="toast-container" id="toast-container"></div>

<script>
// ─────────────────────────────────────────────────────────────
// SOCKET.IO
// ─────────────────────────────────────────────────────────────
const socket = io({ transports: ['websocket', 'polling'] });
let _tel = {};
let _status = {};
let _currentLog = 'thrust';
const _logs = { thrust: [], arm: [], onboard_stab: [], onboard_arm: [] };

socket.on('connect', () => {
  socket.emit('request_status');
});

socket.on('telemetry', (data) => {
  _tel = data;
  updateTelemetry();
  drawHUD('hud1', data);
  drawHUD('hud2', data);
});

socket.on('status', (data) => {
  _status = data;
  updateStatus();
});

socket.on('process_log', ({ name, line }) => {
  const logs = _logs[name] || (_logs[name] = []);
  logs.push(line);
  if (logs.length > 300) logs.splice(0, logs.length - 300);
  if (name === _currentLog) appendLogLine(line);
});

// ─────────────────────────────────────────────────────────────
// CONFIG HELPERS
// ─────────────────────────────────────────────────────────────
function getCfg() {
  const keys = ['pi_ip','pi_user','pi_password','pi_ssh_port','pi_rov_path',
                 'serial_port','camera1_url','camera2_url',
                 'thrust_udp_port','telemetry_port','arm_udp_port',
                 'mosfet_control_port','colmap_command','crabs_command'];
  const obj = {};
  keys.forEach(k => {
    const el = document.getElementById('cfg-' + k);
    if (el) obj[k] = el.value;
  });
  return obj;
}

async function saveConfig() {
  await fetch('/api/config', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(getCfg()),
  });
}

// ─────────────────────────────────────────────────────────────
// SSH
// ─────────────────────────────────────────────────────────────
async function sshConnect() {
  await saveConfig();
  const cfg = getCfg();
  const badge = document.getElementById('ssh-status');
  badge.textContent = 'Connecting…';
  badge.className = 'ssh-badge';

  const r = await fetch('/api/ssh/connect', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(cfg),
  });
  const d = await r.json();
  badge.textContent = d.ok ? '✓ Connected' : '✕ ' + d.msg;
  badge.className = 'ssh-badge ' + (d.ok ? 'ok' : 'err');
  toast(d.ok ? 'SSH connected to ' + cfg.pi_ip : 'SSH failed: ' + d.msg, d.ok ? 'ok' : 'err');
}

// ─────────────────────────────────────────────────────────────
// LAUNCH ACTIONS
// ─────────────────────────────────────────────────────────────
async function startOnboard() {
  await saveConfig();
  const msg = document.getElementById('onboard-msg');
  msg.textContent = 'Starting…';
  const r = await fetch('/api/onboard/start', { method: 'POST' });
  const d = await r.json();
  msg.textContent = d.ok ? '✓ Programs started' : '✕ ' + JSON.stringify(d);
  toast(d.ok ? 'Onboard programs started' : 'Onboard start failed', d.ok ? 'ok' : 'err');
}

async function stopOnboard() {
  await fetch('/api/onboard/stop', { method: 'POST' });
  document.getElementById('onboard-msg').textContent = 'Stopped';
  toast('Onboard programs stopped');
}

async function startTopside() {
  await saveConfig();
  const msg = document.getElementById('topside-msg');
  msg.textContent = 'Starting…';
  const r = await fetch('/api/topside/start', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(getCfg()),
  });
  const d = await r.json();
  msg.textContent = d.ok ? '✓ Programs started' : '✕ ' + JSON.stringify(d.results || d);
  toast(d.ok ? 'Topside programs started' : 'Topside start failed', d.ok ? 'ok' : 'err');
}

async function stopTopside() {
  await fetch('/api/topside/stop', { method: 'POST' });
  document.getElementById('topside-msg').textContent = 'Stopped';
  toast('Topside programs stopped');
}

// ─────────────────────────────────────────────────────────────
// CONTROL ACTIONS
// ─────────────────────────────────────────────────────────────
let _mosfetOn = false;

async function toggleMosfet() {
  _mosfetOn = !_mosfetOn;
  const r = await fetch('/api/mosfet', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ state: _mosfetOn }),
  });
  const d = await r.json();
  updateMosfetUI(_mosfetOn);
  toast('MOSFET ' + (_mosfetOn ? 'ON' : 'OFF'), _mosfetOn ? 'ok' : '');
}

function updateMosfetUI(on) {
  const toggle = document.getElementById('mosfet-toggle');
  const label  = document.getElementById('mosfet-label');
  if (on) {
    toggle.classList.add('on');
    label.textContent = 'MOSFET ON';
  } else {
    toggle.classList.remove('on');
    label.textContent = 'MOSFET OFF';
  }
}

let _currentMode = 'disarmed';

async function setMode(mode) {
  const r = await fetch('/api/mode', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ mode }),
  });
  const d = await r.json();
  if (d.ok) { _currentMode = mode; updateModeUI(mode); }
  toast('Mode: ' + mode.toUpperCase(), mode === 'disarmed' ? '' : 'ok');
}

function updateModeUI(mode) {
  ['disarmed','armed','stabilize'].forEach(m => {
    const btn = document.getElementById('mode-' + m);
    btn.className = 'mode-btn' + (mode === m ? ' active-' + m : '');
  });
}

async function startColmap() {
  const r = await fetch('/api/colmap', { method: 'POST' });
  const d = await r.json();
  toast(d.ok ? '▶ COLMAP started' : 'COLMAP failed: ' + d.msg, d.ok ? 'ok' : 'err');
}

async function startCrabs() {
  const r = await fetch('/api/crabs', { method: 'POST' });
  const d = await r.json();
  toast(d.ok ? '🦀 Crabs started' : 'Crabs failed: ' + d.msg, d.ok ? 'ok' : 'err');
}

// ─────────────────────────────────────────────────────────────
// STATUS UPDATES
// ─────────────────────────────────────────────────────────────
function updateStatus() {
  const s = _status;

  // Launch screen dots
  setDot('dot-stab',     s.onboard_stab);
  setDot('dot-arm',      s.onboard_arm);
  setDot('dot-thrust',   s.thrust_running);
  setDot('dot-armlocal', s.arm_running);

  // SSH badge on launch screen
  const badge = document.getElementById('ssh-status');
  if (s.ssh_connected) {
    badge.textContent = '✓ Connected';
    badge.className = 'ssh-badge ok';
  } else if (s.ssh_error) {
    badge.textContent = '✕ ' + s.ssh_error.substring(0, 40);
    badge.className = 'ssh-badge err';
  }

  // Control screen pills
  const pillConn = document.getElementById('pill-connection');
  pillConn.innerHTML = `<span class="status-dot"></span> SSH: ${s.ssh_connected ? 'ONLINE' : 'OFFLINE'}`;
  pillConn.className = 'status-pill ' + (s.ssh_connected ? 'ok' : 'err');

  const pillMode = document.getElementById('pill-mode');
  const modeColors = { disarmed:'err', armed:'warn', stabilize:'ok' };
  pillMode.innerHTML = `<span class="status-dot"></span> ${(s.mode||'--').toUpperCase()}`;
  pillMode.className = 'status-pill ' + (modeColors[s.mode] || '');

  // Sync mode buttons & MOSFET
  if (s.mode) { _currentMode = s.mode; updateModeUI(s.mode); }
  if (typeof s.mosfet_on !== 'undefined') { _mosfetOn = s.mosfet_on; updateMosfetUI(s.mosfet_on); }

  // Telem bar process status
  const tp = document.getElementById('tel-thrust-proc');
  const ap = document.getElementById('tel-arm-proc');
  tp.textContent = s.thrust_running ? 'RUN' : 'STOP';
  tp.className   = 'tc-val ' + (s.thrust_running ? 'good' : 'bad');
  ap.textContent = s.arm_running ? 'RUN' : 'STOP';
  ap.className   = 'tc-val ' + (s.arm_running ? 'good' : 'bad');
}

function setDot(id, running) {
  const el = document.getElementById(id);
  if (!el) return;
  el.className = 'dot ' + (running ? 'running' : '');
}

// ─────────────────────────────────────────────────────────────
// TELEMETRY UPDATES
// ─────────────────────────────────────────────────────────────
function fmtNum(v, dec=2, unit='') {
  if (v === null || v === undefined) return '--';
  return parseFloat(v).toFixed(dec) + unit;
}

function updateTelemetry() {
  const t = _tel;

  // Pill
  const pillTel = document.getElementById('pill-telemetry');
  const telOk = t.rx_state === 'OK';
  pillTel.innerHTML = `<span class="status-dot"></span> ${t.rx_state || '--'}`;
  pillTel.className = 'status-pill ' + (telOk ? 'ok' : t.rx_state === 'NO_TELEMETRY' ? '' : 'warn');

  // Gain
  document.getElementById('tb-gain').textContent = t.gain_percent ?? '--';

  // Camera overlays
  document.getElementById('c1-depth').textContent  = fmtNum(t.depth_m, 2);
  document.getElementById('c1-hold-d').textContent = fmtNum(t.hold_depth_m, 2);
  document.getElementById('c1-yaw').textContent    = fmtNum(t.yaw_deg, 1);
  document.getElementById('c2-roll').textContent   = fmtNum(t.roll_deg, 1);
  document.getElementById('c2-pitch').textContent  = fmtNum(t.pitch_deg, 1);

  const stabEl = document.getElementById('c2-stab');
  stabEl.textContent = t.stabilize ? 'ON' : 'OFF';
  stabEl.className = t.stabilize ? 'val-hi' : '';

  // Telem bar
  const state = t.rx_state || '--';
  const stateEl = document.getElementById('tel-state');
  stateEl.textContent = state;
  stateEl.className = 'tc-val ' + (state === 'OK' ? 'good' : state === 'NO_TELEMETRY' ? '' : 'warn');

  setText('tel-depth',  fmtNum(t.depth_m, 2, 'm'));
  setText('tel-hold-d', fmtNum(t.hold_depth_m, 2, 'm'));
  setText('tel-yaw',    fmtNum(t.yaw_deg, 1, '°'));
  setText('tel-hold-y', fmtNum(t.hold_yaw_deg, 1, '°'));
  setText('tel-roll',   fmtNum(t.roll_deg, 1, '°'));
  setText('tel-pitch',  fmtNum(t.pitch_deg, 1, '°'));
  setText('tel-hgrp',   fmtNum(t.h_group, 2));
  setText('tel-vgrp',   fmtNum(t.v_group, 2));
  setText('tel-press',  t.pressure_hpa ? fmtNum(t.pressure_hpa, 0, 'hPa') : '--');
  setText('tel-temp',   t.temperature_c ? fmtNum(t.temperature_c, 1, '°C') : '--');

  const dhEl = document.getElementById('tel-dh');
  dhEl.textContent = t.depth_hold_active ? 'HOLD' : (t.depth_hold_request ? 'WAIT' : 'OFF');
  dhEl.className = 'tc-val ' + (t.depth_hold_active ? 'good' : t.depth_hold_request ? 'warn' : '');

  const yhEl = document.getElementById('tel-yh');
  yhEl.textContent = t.yaw_hold_active ? 'HOLD' : (t.yaw_hold_request ? 'WAIT' : 'OFF');
  yhEl.className = 'tc-val ' + (t.yaw_hold_active ? 'good' : t.yaw_hold_request ? 'warn' : '');
}

function setText(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}

// ─────────────────────────────────────────────────────────────
// DIRECTION HUD CANVAS
// ─────────────────────────────────────────────────────────────
function drawHUD(canvasId, t) {
  const canvas = document.getElementById(canvasId);
  if (!canvas) return;
  const wrap = canvas.parentElement;
  canvas.width  = wrap.clientWidth;
  canvas.height = wrap.clientHeight;

  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const W = canvas.width;
  const H = canvas.height;
  const cx = W * 0.5;
  const cy = H * 0.5;
  const R  = Math.min(W, H) * 0.20;

  const fwd  = t.cmd_forward  || 0;
  const lat  = t.cmd_lateral  || 0;
  const yaw  = t.cmd_yaw      || 0;
  const vert = t.cmd_vertical || 0;

  // Outer ring
  ctx.beginPath();
  ctx.arc(cx, cy, R, 0, Math.PI * 2);
  ctx.strokeStyle = 'rgba(0,212,255,0.25)';
  ctx.lineWidth = 1.5;
  ctx.stroke();

  // Crosshair
  ctx.strokeStyle = 'rgba(0,212,255,0.18)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy);
  ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R);
  ctx.stroke();

  // Center dot
  ctx.beginPath();
  ctx.arc(cx, cy, 3, 0, Math.PI * 2);
  ctx.fillStyle = 'rgba(0,212,255,0.5)';
  ctx.fill();

  // Forward/back arrow (vertical in canvas = backward/forward in ROV)
  if (Math.abs(fwd) > 0.02) {
    const ey = cy - fwd * R * 0.85;
    drawArrow(ctx, cx, cy, cx, ey, '#00e08a', Math.abs(fwd));
  }

  // Lateral arrow
  if (Math.abs(lat) > 0.02) {
    const ex = cx + lat * R * 0.85;
    drawArrow(ctx, cx, cy, ex, cy, '#ffb320', Math.abs(lat));
  }

  // Yaw arc
  if (Math.abs(yaw) > 0.02) {
    const startA = -Math.PI / 2;
    const sweepA = yaw * Math.PI * 0.9;
    ctx.beginPath();
    ctx.arc(cx, cy, R * 0.88, startA, startA + sweepA, yaw < 0);
    ctx.strokeStyle = `rgba(255,100,100,${Math.min(1, Math.abs(yaw) * 0.7 + 0.3)})`;
    ctx.lineWidth = 3;
    ctx.stroke();
    // yaw arrowhead at end of arc
    const endA = startA + sweepA;
    const ax = cx + R * 0.88 * Math.cos(endA);
    const ay = cy + R * 0.88 * Math.sin(endA);
    ctx.beginPath();
    ctx.arc(ax, ay, 4, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(255,100,100,0.9)';
    ctx.fill();
  }

  // Vertical indicator — small bar to the right of the circle
  if (Math.abs(vert) > 0.02) {
    const bx  = cx + R * 1.35;
    const bh  = R * 0.8;
    const mid = cy;
    // Background track
    ctx.beginPath();
    ctx.roundRect(bx - 4, mid - bh, 8, bh * 2, 4);
    ctx.fillStyle = 'rgba(0,212,255,0.08)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(0,212,255,0.2)';
    ctx.lineWidth = 1;
    ctx.stroke();
    // Active bar
    const barH = Math.abs(vert) * bh;
    const barY = vert > 0 ? mid - barH : mid;
    ctx.beginPath();
    ctx.roundRect(bx - 4, barY, 8, barH, 3);
    ctx.fillStyle = vert > 0
      ? `rgba(0,224,138,${Math.abs(vert) * 0.7 + 0.3})`
      : `rgba(255,61,90,${Math.abs(vert) * 0.7 + 0.3})`;
    ctx.fill();
    // Label
    ctx.fillStyle = 'rgba(0,212,255,0.6)';
    ctx.font = `${Math.max(9, R * 0.18)}px monospace`;
    ctx.textAlign = 'center';
    ctx.fillText('V', bx, mid - bh - 4);
  }

  // Mode indicator at bottom of HUD
  const modeLabels = { disarmed:'DISARMED', armed:'ARMED', stabilize:'STABILIZE' };
  const modeColors = { disarmed:'rgba(255,61,90,0.8)', armed:'rgba(255,179,32,0.8)',
                       stabilize:'rgba(0,224,138,0.8)' };
  const mode = _currentMode || 'disarmed';
  ctx.font = `bold ${Math.max(9, R * 0.17)}px sans-serif`;
  ctx.textAlign = 'center';
  ctx.fillStyle = modeColors[mode] || 'rgba(200,200,200,0.6)';
  ctx.fillText(modeLabels[mode] || mode.toUpperCase(), cx, cy + R + 18);
}

function drawArrow(ctx, x1, y1, x2, y2, color, opacity) {
  const dx = x2 - x1, dy = y2 - y1;
  const len = Math.sqrt(dx * dx + dy * dy);
  if (len < 4) return;
  const angle = Math.atan2(dy, dx);
  const headLen = Math.min(14, len * 0.35);

  ctx.globalAlpha = Math.min(1, opacity * 0.7 + 0.3);
  ctx.strokeStyle = color;
  ctx.fillStyle   = color;
  ctx.lineWidth   = 2.5;

  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(x2 - headLen * Math.cos(angle - Math.PI / 6),
             y2 - headLen * Math.sin(angle - Math.PI / 6));
  ctx.lineTo(x2 - headLen * Math.cos(angle + Math.PI / 6),
             y2 - headLen * Math.sin(angle + Math.PI / 6));
  ctx.closePath();
  ctx.fill();

  ctx.globalAlpha = 1;
}

// ─────────────────────────────────────────────────────────────
// CAMERA SETUP
// ─────────────────────────────────────────────────────────────
function setupCamera(imgId, noSigId, camNum) {
  const img    = document.getElementById(imgId);
  const noSig  = document.getElementById(noSigId);
  let retryT   = null;

  function load() {
    img.src = `/camera/${camNum}?t=${Date.now()}`;
  }

  img.onload = () => {
    noSig.style.display = 'none';
    img.style.display   = 'block';
  };

  img.onerror = () => {
    noSig.style.display = 'flex';
    img.style.display   = 'none';
    if (!retryT) {
      retryT = setTimeout(() => { retryT = null; load(); }, 5000);
    }
  };

  load();
}

// ─────────────────────────────────────────────────────────────
// VIEW SWITCHING
// ─────────────────────────────────────────────────────────────
function openControl() {
  document.getElementById('launch').classList.remove('active');
  document.getElementById('control').classList.add('active');
  setupCamera('cam1', 'no-sig-1', 1);
  setupCamera('cam2', 'no-sig-2', 2);
  window.addEventListener('resize', resizeHUDs);
  resizeHUDs();
  startHUDLoop();
}

function showLaunch() {
  document.getElementById('control').classList.remove('active');
  document.getElementById('launch').classList.add('active');
  window.removeEventListener('resize', resizeHUDs);
}

function resizeHUDs() {
  ['hud1','hud2'].forEach(id => {
    const c = document.getElementById(id);
    if (!c) return;
    c.width  = c.parentElement.clientWidth;
    c.height = c.parentElement.clientHeight;
  });
  drawHUD('hud1', _tel);
  drawHUD('hud2', _tel);
}

let _hudLoop = null;
function startHUDLoop() {
  if (_hudLoop) return;
  _hudLoop = setInterval(() => {
    drawHUD('hud1', _tel);
    drawHUD('hud2', _tel);
  }, 100);
}

// ─────────────────────────────────────────────────────────────
// LOGS
// ─────────────────────────────────────────────────────────────
let _logOpen = false;

function toggleLog() {
  _logOpen = !_logOpen;
  document.getElementById('log-drawer').classList.toggle('open', _logOpen);
  if (_logOpen) refreshLogView();
}

function switchLog(name) {
  _currentLog = name;
  ['thrust','arm','onboard_stab','onboard_arm'].forEach(n => {
    const id = n === 'onboard_stab' ? 'lt-stab' : n === 'onboard_arm' ? 'lt-arm2'
             : n === 'arm' ? 'lt-arm' : 'lt-thrust';
    document.getElementById(id).classList.toggle('active', n === name);
  });
  refreshLogView();
}

function refreshLogView() {
  const content = document.getElementById('log-content');
  const lines = _logs[_currentLog] || [];
  content.innerHTML = lines.map(l =>
    `<div class="log-line">${escapeHtml(l)}</div>`
  ).join('');
  content.scrollTop = content.scrollHeight;
}

function appendLogLine(line) {
  if (!_logOpen) return;
  const content = document.getElementById('log-content');
  const div = document.createElement('div');
  div.className = 'log-line';
  div.textContent = line;
  content.appendChild(div);
  if (content.children.length > 300) content.removeChild(content.firstChild);
  content.scrollTop = content.scrollHeight;
}

function escapeHtml(s) {
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

// ─────────────────────────────────────────────────────────────
// TOASTS
// ─────────────────────────────────────────────────────────────
function toast(msg, type='') {
  const container = document.getElementById('toast-container');
  const el = document.createElement('div');
  el.className = 'toast ' + type;
  el.textContent = msg;
  container.appendChild(el);
  setTimeout(() => { el.style.opacity='0'; el.style.transition='opacity .3s';
    setTimeout(() => el.remove(), 350); }, 3500);
}

// ─────────────────────────────────────────────────────────────
// INIT
// ─────────────────────────────────────────────────────────────
window.addEventListener('load', () => {
  // Auto-detect Windows serial port default
  if (navigator.platform.includes('Win')) {
    const sp = document.getElementById('cfg-serial_port');
    if (sp && sp.value.startsWith('/dev/')) sp.value = 'COM3';
  }
  // Poll status every 2s as fallback
  setInterval(() => socket.emit('request_status'), 2000);
});
</script>
</body>
</html>
"""


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
    _start_shared_telemetry_listener()
    threading.Thread(target=_monitor_loop, daemon=True).start()

    url = f"http://localhost:{args.port}"
    print(f"\n{'='*55}")
    print(f"  DreadYachet ROV Control UI")
    print(f"  Open: {url}")
    print(f"{'='*55}\n")

    if not args.no_browser:
        # Short delay so Flask has time to bind before browser opens
        threading.Timer(1.2, lambda: webbrowser.open(url)).start()

    socketio.run(app, host=args.host, port=args.port, debug=False, allow_unsafe_werkzeug=True)


if __name__ == "__main__":
    main()
