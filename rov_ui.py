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

DEFAULT_CONFIG = {
    "pi_ip":               "192.168.2.249",
    "pi_user":             "uruc",
    "pi_password":         "yahboom",
    "pi_ssh_port":         22,
    "pi_rov_path":         "/home/uruc/URUCDreadYachet",
    "serial_port":         "COM3" if IS_WINDOWS else "/dev/ttyACM0",
    "camera1_url":         "http://192.168.2.249:8160",
    "camera2_url":         "http://192.168.2.249:8161",
    "thrust_udp_port":     5005,
    "telemetry_port":      5006,
    "arm_udp_port":        5006,
    "mosfet_control_port": 5007,
    "colmap_command":      "python3 colmap_run.py",
    "crabs_command":       "python3 crabs.py",
    "mavproxy_bin":        "/home/uruc/mav_env/bin/mavproxy.py",
    "mavproxy_serial":     "/dev/ttyACM1",
    "mavproxy_baud":       "115200",
    "mavproxy_out1":       "udp:10.42.0.1:14550",
    "mavproxy_out2":       "udp:127.0.0.1:14551",
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
    "onboard_mavproxy":    False,
    "ssh_connected":       False,
    "ssh_error":           "",
    "mode":                "disarmed",
    "mosfet_on":           False,
    "last_telemetry_time": 0.0,
    "telemetry_packets":   0,
    "telemetry_listener_ok": False,
    "onboard_starting":      False,
    "onboard_progress":      [],
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
        "depth_recapture_pending": False,
        "yaw_recapture_pending":   False,
    },
    "logs": {
        "thrust":       [],
        "arm":          [],
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
        script   = f"{rov_path}/{script_rel}"
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
        payload = json.dumps({"cmd": "mosfet", "state": state}).encode()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(1.0)
            s.sendto(payload, (config["pi_ip"], config["mosfet_control_port"]))
            s.close()
            return True, "sent"
        except Exception as e:
            return False, str(e)

    def start_mavproxy(self):
        self.exec("pkill -f mavproxy 2>/dev/null; pkill -f MAVProxy 2>/dev/null; sleep 0.5")
        bin_  = config["mavproxy_bin"]
        ser   = config["mavproxy_serial"]
        baud  = config["mavproxy_baud"]
        out1  = config["mavproxy_out1"]
        out2  = config["mavproxy_out2"]
        cmd = (
            f"nohup {bin_} "
            f"--master={ser} "
            f"--baudrate {baud} "
            f"--out={out1} "
            f"--out={out2} "
            f"--daemon "
            f"> /tmp/rov_mavproxy.log 2>&1 &"
        )
        _, _, error = self.exec(cmd, timeout=10)
        if error:
            return False, error
        return True, "MAVProxy started"

    def stop_mavproxy(self):
        self.exec("pkill -f mavproxy 2>/dev/null; pkill -f MAVProxy 2>/dev/null || true")

    def is_mavproxy_running(self):
        out, _, error = self.exec("pgrep -f mavproxy")
        if error:
            return False
        return bool(out.strip())

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
    """Poll until an onboard process is running or timeout."""
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if check_fn():
            return True, f"{label} running"
        time.sleep(0.5)
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
    tel["depth_recapture_pending"] = bool(pkt.get("depth_recapture_pending", False))
    tel["yaw_recapture_pending"]   = bool(pkt.get("yaw_recapture_pending", False))
    STATE["last_telemetry_time"]   = time.time()
    STATE["telemetry_packets"]     = STATE.get("telemetry_packets", 0) + 1
    socketio.emit("telemetry", dict(tel))


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

def _monitor_loop():
    while True:
        time.sleep(1.0)
        STATE["thrust_running"] = is_local_running("thrust")
        STATE["arm_running"]    = is_local_running("arm")

        if ssh.is_connected():
            STATE["ssh_connected"]    = True
            STATE["onboard_mavproxy"] = ssh.is_mavproxy_running()
            STATE["onboard_stab"]     = ssh.is_onboard_running("stabilization.py")
            STATE["onboard_arm"]      = ssh.is_onboard_running("new_ar.py")
        else:
            STATE["ssh_connected"]    = False
            STATE["onboard_mavproxy"] = False
            STATE["onboard_stab"]     = False
            STATE["onboard_arm"]      = False

        tel_age = time.time() - STATE["last_telemetry_time"]
        if tel_age > 2.0:
            STATE["telemetry"]["rx_state"] = "NO_TELEMETRY"

        emit_status()


def emit_status():
    with _state_lock:
        progress = list(STATE["onboard_progress"])
    socketio.emit("status", {
        "thrust_running":        STATE["thrust_running"],
        "arm_running":           STATE["arm_running"],
        "onboard_stab":          STATE["onboard_stab"],
        "onboard_arm":           STATE["onboard_arm"],
        "onboard_mavproxy":      STATE["onboard_mavproxy"],
        "ssh_connected":         STATE["ssh_connected"],
        "ssh_error":             STATE["ssh_error"],
        "mode":                  STATE["mode"],
        "mosfet_on":             STATE["mosfet_on"],
        "telemetry_listener_ok": STATE["telemetry_listener_ok"],
        "onboard_starting":      STATE["onboard_starting"],
        "onboard_progress":      progress,
    })


# ─────────────────────────────────────────────────────────────────────────────
# CAMERA PROXY
# ─────────────────────────────────────────────────────────────────────────────

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

    if STATE["onboard_starting"]:
        return jsonify({"ok": False, "msg": "Onboard start already in progress"})

    STATE["onboard_starting"] = True
    STATE["onboard_progress"] = []
    emit_status()

    def _do_start():
        try:
            # Step 1: MAVProxy
            _emit_onboard_progress("mavproxy", "starting", "Launching MAVProxy bridge...")
            ok_m, msg_m = ssh.start_mavproxy()
            if ok_m:
                ok_m, msg_m = _wait_onboard_running(
                    ssh.is_mavproxy_running, "MAVProxy", timeout_sec=15.0
                )
            STATE["onboard_mavproxy"] = ok_m
            _emit_onboard_progress(
                "mavproxy", "done" if ok_m else "error", msg_m
            )
            emit_status()

            if ok_m:
                for i in range(4):
                    time.sleep(0.5)
                    _emit_onboard_progress(
                        "mavproxy", "wait",
                        f"Waiting for MAVProxy to initialize... ({i + 1}/4)"
                    )

            # Step 2: stabilization.py
            _emit_onboard_progress("stabilization", "starting", "Launching stabilization.py...")
            ok_s, msg_s = ssh.start_onboard_process("onboard/stabilization.py", "stab")
            if ok_s:
                ok_s, msg_s = _wait_onboard_running(
                    lambda: ssh.is_onboard_running("stabilization.py"),
                    "stabilization.py",
                    timeout_sec=10.0,
                )
            STATE["onboard_stab"] = ok_s
            _emit_onboard_progress(
                "stabilization", "done" if ok_s else "error", msg_s
            )
            emit_status()

            # Step 3: new_ar.py (arm — optional; thrusters work without it)
            _emit_onboard_progress("arm_ctrl", "starting", "Launching new_ar.py (arm controller)...")
            ok_a, msg_a = ssh.start_onboard_process("onboard/new_ar.py", "arm")
            if ok_a:
                ok_a, msg_a = _wait_onboard_running(
                    lambda: ssh.is_onboard_running("new_ar.py"),
                    "new_ar.py",
                    timeout_sec=20.0,
                )
            if not ok_a:
                log_tail = ssh.get_onboard_log("arm", lines=8)
                if log_tail:
                    last_line = log_tail.strip().splitlines()[-1][:120]
                    msg_a = f"{msg_a} | Log: {last_line}"
            STATE["onboard_arm"] = ok_a
            _emit_onboard_progress(
                "arm_ctrl", "done" if ok_a else "error", msg_a
            )
            emit_status()

            core_ok = ok_m and ok_s
            all_ok = core_ok and ok_a
            if all_ok:
                summary = "✓ All onboard programs running (MAVProxy, stabilization, new_ar)"
            elif core_ok:
                summary = "✓ Thruster control ready (MAVProxy + stabilization). Arm controller failed — see log."
            else:
                parts = []
                if not ok_m:
                    parts.append("MAVProxy")
                if not ok_s:
                    parts.append("stabilization")
                if not ok_a:
                    parts.append("new_ar")
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
    ssh.stop_onboard_process("stabilization.py")
    ssh.stop_onboard_process("new_ar.py")
    ssh.stop_mavproxy()
    STATE["onboard_stab"]     = False
    STATE["onboard_arm"]      = False
    STATE["onboard_mavproxy"] = False
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


@app.route("/api/status")
def api_status():
    with _state_lock:
        progress = list(STATE["onboard_progress"])
    return jsonify({
        "thrust_running":        STATE["thrust_running"],
        "arm_running":           STATE["arm_running"],
        "onboard_stab":          STATE["onboard_stab"],
        "onboard_arm":           STATE["onboard_arm"],
        "onboard_mavproxy":      STATE["onboard_mavproxy"],
        "ssh_connected":         STATE["ssh_connected"],
        "mode":                  STATE["mode"],
        "mosfet_on":             STATE["mosfet_on"],
        "telemetry_listener_ok": STATE["telemetry_listener_ok"],
        "onboard_starting":      STATE["onboard_starting"],
        "onboard_progress":      progress,
        "telemetry":             STATE["telemetry"],
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
        })


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
    with _state_lock:
        for entry in STATE["onboard_progress"]:
            socketio.emit("onboard_progress", entry)


@socketio.on("request_status")
def on_request_status():
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
    _forward_ctrl_to_pi(data)


# ─────────────────────────────────────────────────────────────────────────────
# HTML TEMPLATE
# ─────────────────────────────────────────────────────────────────────────────

HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1.0"/>
<title>DreadYachet ROV</title>
<script src="https://cdn.socket.io/4.7.5/socket.io.min.js" crossorigin="anonymous"></script>
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
::-webkit-scrollbar { width:6px; height:6px; }
::-webkit-scrollbar-track { background:var(--bg2); }
::-webkit-scrollbar-thumb { background:var(--border2); border-radius:3px; }

/* ── views ── */
.view { display:none; height:100vh; flex-direction:column; }
.view.active { display:flex; }

/* ════════════════════════════════════════ LAUNCH SCREEN */
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
  display:grid; grid-template-columns:repeat(auto-fill, minmax(260px,1fr));
  gap:12px; width:100%; max-width:900px;
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

.launch-panels {
  display:grid; grid-template-columns:1fr 1fr;
  gap:16px; width:100%; max-width:900px;
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
.dot.warn    { background:var(--amber); box-shadow:0 0 6px var(--amber); }

/* progress log */
.progress-log {
  font-size:0.72rem; font-family:var(--mono); color:var(--dim);
  min-height:60px; max-height:120px; overflow-y:auto;
  background:var(--bg3); border:1px solid var(--border);
  border-radius:6px; padding:8px;
}
.progress-log .pl-step { margin-bottom:3px; }
.progress-log .pl-step.ok   { color:var(--green); }
.progress-log .pl-step.err  { color:var(--red); }
.progress-log .pl-step.wait { color:var(--amber); }
.progress-log .pl-step.info { color:var(--accent); }

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
.proceed-row { width:100%; max-width:900px; text-align:center; }
.proceed-row .btn { padding:12px 40px; font-size:1rem; }

/* ── ssh status pill ── */
.ssh-badge {
  display:inline-flex; align-items:center; gap:6px;
  padding:4px 12px; border-radius:20px;
  font-size:0.75rem; font-weight:600;
  background:var(--bg3); border:1px solid var(--border);
}
.ssh-badge.ok  { border-color:var(--green); color:var(--green); }
.ssh-badge.err { border-color:var(--red);   color:var(--red); }

/* ════════════════════════════════════════ CONTROL SCREEN */
#control { height:100vh; display:none; flex-direction:column; overflow:hidden; }
#control.active { display:flex; }

/* ── top bar ── */
.topbar {
  display:flex; align-items:center; gap:8px; padding:0 12px;
  height:46px; background:var(--bg2); border-bottom:1px solid var(--border);
  flex-shrink:0; overflow-x:auto;
}
.topbar-title { font-weight:700; font-size:0.95rem; color:var(--accent);
  white-space:nowrap; margin-right:4px; }
.topbar-spacer { flex:1; min-width:8px; }
.status-pill {
  display:flex; align-items:center; gap:5px;
  padding:3px 10px; border-radius:20px;
  font-size:0.7rem; font-weight:600; white-space:nowrap;
  border:1px solid var(--border); background:var(--bg3); color:var(--dim);
  flex-shrink:0;
}
.status-pill.ok   { border-color:var(--green); color:var(--green); }
.status-pill.warn { border-color:var(--amber); color:var(--amber); }
.status-pill.err  { border-color:var(--red);   color:var(--red); }
.status-dot { width:6px; height:6px; border-radius:50%; background:currentColor; }

/* ── cameras ── */
.cameras {
  flex:1; display:grid; grid-template-columns:1fr 1fr;
  gap:2px; background:var(--border); overflow:hidden; min-height:0;
}
@media(max-width:700px){ .cameras { grid-template-columns:1fr; } }
.cam-wrap {
  position:relative; background:#000; overflow:hidden;
  display:flex; align-items:center; justify-content:center;
}
.cam-img { width:100%; height:100%; object-fit:contain; display:block; }
.cam-no-signal {
  position:absolute; inset:0; display:flex; flex-direction:column;
  align-items:center; justify-content:center; gap:8px;
  background:var(--bg); color:var(--dim);
}
.cam-no-signal .ns-icon { font-size:2.5rem; opacity:0.4; }
.cam-no-signal .ns-text { font-size:0.75rem; letter-spacing:2px;
  text-transform:uppercase; opacity:0.5; }
.cam-hud { position:absolute; inset:0; pointer-events:none; }

/* Disarmed overlay */
.disarmed-banner {
  position:absolute; bottom:12px; left:50%; transform:translateX(-50%);
  background:rgba(255,61,90,0.92); color:#fff;
  padding:8px 20px; border-radius:8px; font-weight:700; font-size:0.85rem;
  letter-spacing:0.5px; z-index:5; pointer-events:none;
  box-shadow:0 4px 20px rgba(0,0,0,0.5);
}
.disarmed-banner.hidden { display:none; }

.cam-label {
  position:absolute; top:8px; left:10px;
  font-size:0.65rem; letter-spacing:1.5px; text-transform:uppercase;
  color:rgba(0,212,255,0.7); font-weight:700; pointer-events:none;
}
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
.ctrl-sep { width:1px; height:24px; background:var(--border); margin:0 4px; flex-shrink:0; }

.mode-btn {
  padding:6px 14px; border-radius:6px; border:1px solid var(--border);
  background:var(--bg3); color:var(--dim); font-size:0.78rem; font-weight:600;
  cursor:pointer; transition:all .15s; white-space:nowrap;
}
.mode-btn:hover { border-color:var(--accent); color:var(--accent); }
.mode-btn.active-disarmed  { border-color:var(--red);   color:var(--red);   background:rgba(255,61,90,.12); }
.mode-btn.active-armed     { border-color:var(--amber); color:var(--amber); background:rgba(255,179,32,.12); }
.mode-btn.active-stabilize { border-color:var(--green); color:var(--green); background:rgba(0,224,138,.12); }

/* Controller state indicators */
.ctrl-flag {
  padding:4px 10px; border-radius:5px;
  border:1px solid var(--border); background:var(--bg3);
  font-size:0.72rem; font-weight:700; font-family:var(--mono);
  color:var(--dim); cursor:pointer; transition:all .15s; white-space:nowrap;
}
.ctrl-flag.active-stab  { border-color:var(--green); color:var(--green); background:rgba(0,224,138,.12); }
.ctrl-flag.active-depth { border-color:var(--accent); color:var(--accent); background:rgba(0,212,255,.12); }
.ctrl-flag.active-yaw   { border-color:var(--accent2); color:var(--accent2); background:rgba(123,94,167,.12); }
.ctrl-flag:hover { border-color:var(--text); }

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
  cursor:pointer; transition:all .15s; white-space:nowrap;
}
.action-btn:hover { background:var(--accent2); color:#fff; }

/* ── telemetry bar ── */
.telembar {
  display:flex; align-items:center; gap:0;
  padding:4px 12px; background:var(--bg3);
  border-top:1px solid var(--border);
  font-family:var(--mono); font-size:0.7rem; flex-shrink:0;
  overflow-x:auto; white-space:nowrap;
}
.telem-cell {
  display:flex; flex-direction:column; padding:2px 10px;
  border-right:1px solid var(--border);
}
.telem-cell:last-child { border-right:none; }
.telem-cell .tc-label { font-size:0.58rem; color:var(--dim); text-transform:uppercase; letter-spacing:0.8px; }
.telem-cell .tc-val   { font-size:0.78rem; color:var(--text); font-weight:600; }
.telem-cell .tc-val.good { color:var(--green); }
.telem-cell .tc-val.warn { color:var(--amber); }
.telem-cell .tc-val.bad  { color:var(--red); }

/* ── log drawer ── */
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

/* ── KEYBINDS MODAL ── */
.modal-overlay {
  position:fixed; inset:0; background:rgba(0,0,0,0.75);
  z-index:300; display:flex; align-items:center; justify-content:center;
}
.modal-box {
  background:var(--bg2); border:1px solid var(--border2);
  border-radius:14px; width:min(760px,95vw); max-height:88vh;
  display:flex; flex-direction:column; overflow:hidden;
  box-shadow:0 8px 40px rgba(0,0,0,0.6);
}
.modal-header {
  display:flex; align-items:center; padding:16px 20px;
  border-bottom:1px solid var(--border); flex-shrink:0;
}
.modal-header h2 { font-size:1.1rem; font-weight:700; color:var(--accent); flex:1; }
.modal-body { overflow-y:auto; padding:20px; }
.kb-section { margin-bottom:24px; }
.kb-section h3 {
  font-size:0.72rem; text-transform:uppercase; letter-spacing:1px;
  color:var(--accent2); margin-bottom:12px; padding-bottom:6px;
  border-bottom:1px solid var(--border);
}
.kb-table { width:100%; border-collapse:collapse; }
.kb-table tr { border-bottom:1px solid var(--border); }
.kb-table tr:last-child { border-bottom:none; }
.kb-table td { padding:8px 6px; font-size:0.82rem; vertical-align:top; }
.kb-table td:first-child {
  font-family:var(--mono); color:var(--accent); font-size:0.8rem;
  white-space:nowrap; width:160px; padding-right:16px;
}
.kb-table td:last-child { color:var(--text); }
.kb-note { font-size:0.75rem; color:var(--dim); margin-top:4px; }

/* Layout selector in modal */
.layout-select { display:flex; gap:10px; margin-top:8px; }
.layout-btn {
  padding:7px 20px; border-radius:7px; border:1px solid var(--border2);
  background:var(--bg3); color:var(--dim); font-size:0.8rem; font-weight:600;
  cursor:pointer; transition:all .15s;
}
.layout-btn.active { border-color:var(--accent); color:var(--accent); background:rgba(0,212,255,.1); }
.layout-btn:hover  { border-color:var(--text); color:var(--text); }

/* Gamepad diagram */
.gp-diagram {
  background:var(--bg3); border:1px solid var(--border); border-radius:10px;
  padding:16px; margin-top:8px; text-align:center;
  font-family:var(--mono); font-size:0.72rem; color:var(--dim); line-height:2;
}

/* ── toast notifications ── */
.toast-container {
  position:fixed; top:52px; right:12px; z-index:400;
  display:flex; flex-direction:column; gap:6px;
}
.toast {
  padding:8px 14px; border-radius:8px; font-size:0.8rem;
  border-left:3px solid var(--accent);
  background:var(--bg2); box-shadow:0 4px 16px rgba(0,0,0,.5);
  animation:fadeIn .2s ease;
  max-width:320px; pointer-events:none;
}
.toast.err { border-color:var(--red); }
.toast.ok  { border-color:var(--green); }
.toast.warn { border-color:var(--amber); }
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
        <input id="cfg-serial_port" type="text" value="COM3"/></div>
      <div class="field"><label>Camera 1 URL (MJPEG)</label>
        <input id="cfg-camera1_url" type="text" value="http://192.168.2.249:8160"/></div>
      <div class="field"><label>Camera 2 URL (MJPEG)</label>
        <input id="cfg-camera2_url" type="text" value="http://192.168.2.249:8161"/></div>
    </div>

    <div class="config-group">
      <h3>Network Ports</h3>
      <div class="field"><label>Thrust UDP Port (→ Pi, 5005)</label>
        <input id="cfg-thrust_udp_port" type="text" value="5005"/></div>
      <div class="field"><label>Telemetry Port (← Pi, 5006)</label>
        <input id="cfg-telemetry_port" type="text" value="5006"/></div>
      <div class="field"><label>Arm UDP Port (→ Pi)</label>
        <input id="cfg-arm_udp_port" type="text" value="5006"/></div>
      <div class="field"><label>MOSFET Control Port</label>
        <input id="cfg-mosfet_control_port" type="text" value="5007"/></div>
    </div>

    <div class="config-group">
      <h3>MAVProxy (on Pi)</h3>
      <div class="field"><label>MAVProxy Binary Path</label>
        <input id="cfg-mavproxy_bin" type="text" value="/home/uruc/mav_env/bin/mavproxy.py"/></div>
      <div class="field"><label>Pixhawk Serial Port</label>
        <input id="cfg-mavproxy_serial" type="text" value="/dev/ttyACM1"/></div>
      <div class="field"><label>Baud Rate</label>
        <input id="cfg-mavproxy_baud" type="text" value="115200"/></div>
      <div class="field"><label>UDP Out 1 (topside)</label>
        <input id="cfg-mavproxy_out1" type="text" value="udp:10.42.0.1:14550"/></div>
      <div class="field"><label>UDP Out 2 (local Pi)</label>
        <input id="cfg-mavproxy_out2" type="text" value="udp:127.0.0.1:14551"/></div>
    </div>

    <div class="config-group">
      <h3>Onboard Commands</h3>
      <div class="field"><label>COLMAP Command (on Pi)</label>
        <input id="cfg-colmap_command" type="text" value="python3 colmap_run.py"/></div>
      <div class="field"><label>Crabs Command (on Pi)</label>
        <input id="cfg-crabs_command" type="text" value="python3 crabs.py"/></div>
    </div>

  </div>

  <div class="launch-panels">

    <!-- Onboard panel -->
    <div class="panel">
      <h2>🔵 Onboard Programs <small style="font-size:.7rem;color:var(--dim)">(Pi)</small></h2>
      <p class="panel-desc">Launches MAVProxy, <code>stabilization.py</code> (thruster control) and
        <code>new_ar.py</code> (arm + MOSFET) on the ROV Pi via SSH.</p>
      <div class="program-status">
        <div class="prog-row"><div class="dot" id="dot-mavproxy"></div><span>mavproxy (UDP bridge)</span></div>
        <div class="prog-row"><div class="dot" id="dot-stab"></div><span>stabilization.py</span></div>
        <div class="prog-row"><div class="dot" id="dot-arm"></div><span>new_ar.py</span></div>
      </div>
      <div class="btn-row">
        <button class="btn btn-primary" id="btn-start-onboard" onclick="startOnboard()">Start Onboard</button>
        <button class="btn btn-danger" onclick="stopOnboard()">Stop</button>
      </div>
      <div id="onboard-summary" style="font-size:.8rem;font-weight:600;min-height:20px;font-family:var(--mono)"></div>
      <div class="progress-log" id="onboard-progress-log" style="display:none"></div>
    </div>

    <!-- Topside panel -->
    <div class="panel">
      <h2>🟡 Topside Programs <small style="font-size:.7rem;color:var(--dim)">(this PC)</small></h2>
      <p class="panel-desc">Launches <code>arm_sender.py</code> (serial → arm control) on this PC.
        Gamepad thruster control is built into this web UI — no separate script needed.</p>
      <div class="program-status">
        <div class="prog-row">
          <div class="dot" id="dot-gamepad-launch"></div>
          <span id="gamepad-launch-label">Gamepad — press any button to connect</span>
        </div>
        <div class="prog-row"><div class="dot" id="dot-armlocal"></div><span>arm_sender.py</span></div>
        <div class="prog-row">
          <div class="dot" id="dot-telem-launch"></div>
          <span id="telem-launch-label">Telemetry listener (UDP 5006)</span>
        </div>
      </div>
      <div class="btn-row">
        <button class="btn btn-secondary" id="btn-activate-gp" onclick="activateGamepad()">Activate Gamepad</button>
        <button class="btn btn-primary" id="btn-start-topside" onclick="startTopside()">Start Arm Sender</button>
        <button class="btn btn-danger" onclick="stopTopside()">Stop</button>
      </div>
      <div id="topside-msg" style="font-size:.75rem;color:var(--dim);min-height:18px;font-family:var(--mono)"></div>
    </div>

  </div>

  <div class="proceed-row">
    <button class="btn btn-success" onclick="openControl()">Open Control Screen →</button>
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
    <span id="pill-gamepad" class="status-pill err">
      <span class="status-dot"></span> GP: NONE
    </span>

    <span class="topbar-spacer"></span>

    <span style="font-family:var(--mono);font-size:.75rem;color:var(--dim);white-space:nowrap;flex-shrink:0">
      GAIN: <span id="tb-gain" style="color:var(--accent)">100</span>%
    </span>
    <button class="btn btn-secondary" style="padding:4px 12px;font-size:.75rem;flex-shrink:0"
            onclick="activateGamepad()">🎮 GP</button>
    <button class="btn btn-secondary" style="padding:4px 12px;font-size:.75rem;flex-shrink:0"
            onclick="showKeybinds()">⌨ Keybinds</button>
    <button class="btn btn-secondary" style="padding:4px 12px;font-size:.75rem;flex-shrink:0"
            onclick="toggleLog()">Logs</button>
    <button class="btn btn-secondary" style="padding:4px 12px;font-size:.75rem;flex-shrink:0"
            onclick="showLaunch()">← Launch</button>
  </div>

  <!-- Camera feeds -->
  <div class="cameras">

    <div class="cam-wrap">
      <div class="cam-no-signal" id="no-sig-1">
        <div class="ns-icon">📡</div>
        <div class="ns-text">No Signal — Camera 1</div>
      </div>
      <img id="cam1" class="cam-img" style="display:none" alt="Camera 1"/>
      <canvas id="hud1" class="cam-hud"></canvas>
      <div id="disarmed-banner" class="disarmed-banner hidden">⚠ DISARMED — select DRIVE/ARMED to move thrusters</div>
      <div class="cam-label">CAM 1 / FORWARD</div>
      <div class="cam-telemetry" id="cam1-tel">
        <div>DEPTH <span class="val-hi" id="c1-depth">--</span>m</div>
        <div>HOLD  <span id="c1-hold-d">--</span>m</div>
        <div>YAW   <span class="val-hi" id="c1-yaw">--</span>°</div>
      </div>
    </div>

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

  </div>

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

    <span class="control-label">Hold</span>
    <button class="ctrl-flag" id="flag-stab"    onclick="toggleStabilize()">STAB: OFF</button>
    <button class="ctrl-flag" id="flag-depth"   onclick="toggleDepthHold()">DEPTH: OFF</button>
    <button class="ctrl-flag" id="flag-yaw"     onclick="toggleYawHold()">YAW: OFF</button>

    <div class="ctrl-sep"></div>

    <span class="control-label">Actions</span>
    <button class="action-btn" onclick="startColmap()">▶ COLMAP</button>
    <button class="action-btn" onclick="startCrabs()">🦀 CRABS</button>

  </div>

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
      <span class="tc-label">CMD F</span>
      <span class="tc-val" id="tel-cmd-f">--</span>
    </div>
    <div class="telem-cell">
      <span class="tc-label">CMD L</span>
      <span class="tc-val" id="tel-cmd-l">--</span>
    </div>
    <div class="telem-cell">
      <span class="tc-label">CMD Y</span>
      <span class="tc-val" id="tel-cmd-y">--</span>
    </div>
    <div class="telem-cell">
      <span class="tc-label">CMD V</span>
      <span class="tc-val" id="tel-cmd-v">--</span>
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
      <div class="log-tab active" onclick="switchLog('arm')" id="lt-arm">Arm Sender</div>
      <div class="log-tab" onclick="switchLog('onboard_stab')" id="lt-stab">Onboard Stab</div>
      <div class="log-tab" onclick="switchLog('onboard_arm')" id="lt-arm2">Onboard Arm</div>
    </div>
    <span class="log-close" onclick="toggleLog()">✕</span>
  </div>
  <div class="log-content" id="log-content"></div>
</div>

<!-- ══════════════════════════════════════════════════
     KEYBINDS MODAL
     ══════════════════════════════════════════════════ -->
<div id="keybinds-modal" class="modal-overlay" style="display:none" onclick="hideKeybindsOutside(event)">
  <div class="modal-box">
    <div class="modal-header">
      <h2>⌨ Controls &amp; Keybinds</h2>
      <button class="btn btn-secondary" style="padding:4px 12px;font-size:.8rem"
              onclick="hideKeybinds()">Close ✕</button>
    </div>
    <div class="modal-body">

      <div class="kb-section">
        <h3>Drive Modes (click buttons in control bar)</h3>
        <table class="kb-table">
          <tr><td>DISARMED</td><td>All thruster commands zeroed — ROV won't move. Safe state.</td></tr>
          <tr><td>DRIVE / ARMED</td><td>Full manual control via gamepad. Stabilization flags controlled independently.</td></tr>
          <tr><td>STABILIZE</td><td>Manual control + auto pitch/roll correction from IMU. Enables STAB flag automatically.</td></tr>
        </table>
      </div>

      <div class="kb-section">
        <h3>Keyboard Shortcuts (work on Control screen)</h3>
        <table class="kb-table">
          <tr><td>S</td><td>Toggle pitch/roll stabilization ON/OFF</td></tr>
          <tr><td>D</td><td>Toggle depth hold ON/OFF</td></tr>
          <tr><td>Y</td><td>Toggle yaw hold ON/OFF</td></tr>
          <tr><td>↑ Arrow</td><td>Increase gain by 10%</td></tr>
          <tr><td>↓ Arrow</td><td>Decrease gain by 10%</td></tr>
          <tr><td>ESC</td><td>Emergency stop — sets DISARMED mode and clears all flags</td></tr>
        </table>
      </div>

      <div class="kb-section">
        <h3>Gamepad Axes (Axis Layout: <span id="kb-layout-display" style="color:var(--accent)">ORIGINAL</span>)</h3>
        <table class="kb-table">
          <tr><td>Left Stick Y</td><td>Vertical / Heave — up = ascend, down = descend</td></tr>
          <tr><td>Right Stick Y</td><td>Forward / Backward</td></tr>
          <tr><td id="kb-axis-left-x-label">Left Stick X</td><td id="kb-axis-left-x-desc">Yaw (rotate) — left = turn left, right = turn right</td></tr>
          <tr><td id="kb-axis-right-x-label">Right Stick X</td><td id="kb-axis-right-x-desc">Lateral strafe — left = strafe left, right = strafe right</td></tr>
        </table>
        <p class="kb-note">Axis layout matches thrust_sender.py CONTROL_LAYOUT setting.</p>
        <div class="layout-select" style="margin-top:12px">
          <span style="font-size:.8rem;color:var(--dim);line-height:2">Layout:</span>
          <button class="layout-btn active" id="layout-btn-original" onclick="setLayout('original')">
            Original (Left X = Yaw)
          </button>
          <button class="layout-btn" id="layout-btn-swapped" onclick="setLayout('swapped')">
            Swapped (Left X = Strafe)
          </button>
        </div>
      </div>

      <div class="kb-section">
        <h3>Gamepad Buttons</h3>
        <table class="kb-table">
          <tr><td>Button 9</td><td>Toggle stabilization (same as S key) — typically Start/Menu button</td></tr>
          <tr><td>D-pad Up</td><td>Increase gain by 10%</td></tr>
          <tr><td>D-pad Down</td><td>Decrease gain by 10%</td></tr>
        </table>
        <p class="kb-note">Button numbering is browser Gamepad API index — may vary by controller.
          Check browser console for button debug output when pressing buttons.</p>
      </div>

      <div class="kb-section">
        <h3>Control Parameters</h3>
        <table class="kb-table">
          <tr><td>Deadzone</td><td>5% (axes under 0.05 are treated as zero)</td></tr>
          <tr><td>Gain Range</td><td>10% – 100%, step 10%, default 100%</td></tr>
          <tr><td>Combined Thrust Limit</td><td>H-group + V-group ≤ 150% (same as thrust_sender.py)</td></tr>
          <tr><td>Send Rate</td><td>50 Hz (matches thrust_sender.py SEND_HZ)</td></tr>
          <tr><td>Control Timeout (Pi)</td><td>0.8 s — Pi zeros thrusters if no packet received</td></tr>
        </table>
      </div>

      <div class="kb-section">
        <h3>MOSFET Toggle</h3>
        <table class="kb-table">
          <tr><td>MOSFET OFF</td><td>Physical power cut to servos / accessories. Does NOT disarm thrusters.</td></tr>
          <tr><td>MOSFET ON</td><td>Power enabled. Set mode to ARMED or STABILIZE to move thrusters.</td></tr>
        </table>
      </div>

    </div>
  </div>
</div>

<!-- Toast container -->
<div class="toast-container" id="toast-container"></div>

<script>
// ─────────────────────────────────────────────────────────────
// GLOBAL STATE (must be first — before Socket.IO connect)
// ─────────────────────────────────────────────────────────────
let _tel    = {};
let _status = {};
let _currentLog  = 'arm';
const _logs = { thrust: [], arm: [], onboard_stab: [], onboard_arm: [] };
let _onboardPollTimer = null;
const _onboardProgressSeen = new Set();
let socket = null;

function socketEmit(event, data) {
  if (socket && socket.connected) socket.emit(event, data);
}

// ─────────────────────────────────────────────────────────────
// SOCKET.IO
// ─────────────────────────────────────────────────────────────
if (typeof io !== 'undefined') {
  socket = io({ transports: ['websocket', 'polling'] });

  socket.on('connect', () => {
    socketEmit('request_status');
  });

  socket.on('telemetry', (data) => {
    _tel = data;
    updateTelemetry();
    updateCtrlCmdsFromTelemetry();
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

  socket.on('onboard_progress', (entry) => {
    handleOnboardProgress(entry);
  });
} else {
  console.error('Socket.IO client failed to load — live updates use HTTP polling fallback');
}

function handleOnboardProgress({ step, status, msg, time }) {
  const key = `${time || ''}|${step}|${status}|${msg}`;
  if (_onboardProgressSeen.has(key)) return;
  _onboardProgressSeen.add(key);

  const logEl = document.getElementById('onboard-progress-log');
  if (logEl) {
    logEl.style.display = 'block';
    const icons = { starting: '⟳', wait: '…', done: '✓', error: '✕', complete: '★' };
    const cls   = { starting: 'info', wait: 'wait', done: 'ok', error: 'err', complete: 'ok' };
    const div = document.createElement('div');
    div.className = 'pl-step ' + (cls[status] || 'info');
    div.textContent = `${icons[status] || '?'} [${step}] ${msg || status}`;
    logEl.appendChild(div);
    logEl.scrollTop = logEl.scrollHeight;
  }

  // Update status dots immediately from progress events
  if (step === 'mavproxy') {
    if (status === 'done') setDot('dot-mavproxy', true);
    if (status === 'error') setDotError('dot-mavproxy');
  }
  if (step === 'stabilization') {
    if (status === 'done') setDot('dot-stab', true);
    if (status === 'error') setDotError('dot-stab');
  }
  if (step === 'arm_ctrl') {
    if (status === 'done') setDot('dot-arm', true);
    if (status === 'error') setDotError('dot-arm');
  }

  const summary = document.getElementById('onboard-summary');
  if (summary && msg) {
    if (status === 'done' || status === 'complete') {
      summary.style.color = 'var(--green)';
      summary.textContent = msg;
    } else if (status === 'error') {
      summary.style.color = 'var(--red)';
      summary.textContent = msg;
    } else if (status === 'starting' || status === 'wait') {
      summary.style.color = 'var(--amber)';
      summary.textContent = msg;
    }
  }

  if (step === 'complete') {
    toast(msg, status === 'done' ? 'ok' : 'err');
    const btn = document.getElementById('btn-start-onboard');
    if (btn) btn.disabled = false;
    stopOnboardPoll();
  }
}

function startOnboardPoll() {
  stopOnboardPoll();

  async function pollOnce() {
    try {
      const r = await fetch('/api/onboard/progress');
      const d = await r.json();
      if (d.events && d.events.length) {
        const lastSeen = _onboardPollTimer ? (_onboardPollTimer._lastCount || 0) : 0;
        for (let i = lastSeen; i < d.events.length; i++) {
          handleOnboardProgress(d.events[i]);
        }
        if (_onboardPollTimer) _onboardPollTimer._lastCount = d.events.length;
      }
      if (d.onboard_mavproxy) setDot('dot-mavproxy', true);
      if (d.onboard_stab)     setDot('dot-stab', true);
      if (d.onboard_arm)      setDot('dot-arm', true);
      if (!d.starting) stopOnboardPoll();
    } catch (_) {}
  }

  _onboardPollTimer = setInterval(pollOnce, 800);
  _onboardPollTimer._lastCount = 0;
  pollOnce();
}

function stopOnboardPoll() {
  if (_onboardPollTimer) {
    clearInterval(_onboardPollTimer);
    _onboardPollTimer = null;
  }
}

// ─────────────────────────────────────────────────────────────
// CONFIG HELPERS
// ─────────────────────────────────────────────────────────────
function getCfg() {
  const keys = ['pi_ip','pi_user','pi_password','pi_ssh_port','pi_rov_path',
                 'serial_port','camera1_url','camera2_url',
                 'thrust_udp_port','telemetry_port','arm_udp_port',
                 'mosfet_control_port','colmap_command','crabs_command',
                 'mavproxy_bin','mavproxy_serial','mavproxy_baud',
                 'mavproxy_out1','mavproxy_out2'];
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
  const cfg   = getCfg();
  const badge = document.getElementById('ssh-status');
  badge.textContent = 'Connecting…';
  badge.className   = 'ssh-badge';

  const r = await fetch('/api/ssh/connect', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(cfg),
  });
  const d = await r.json();
  badge.textContent = d.ok ? '✓ Connected' : '✕ ' + d.msg;
  badge.className   = 'ssh-badge ' + (d.ok ? 'ok' : 'err');
  toast(d.ok ? 'SSH connected to ' + cfg.pi_ip : 'SSH failed: ' + d.msg, d.ok ? 'ok' : 'err');
}

// ─────────────────────────────────────────────────────────────
// LAUNCH ACTIONS
// ─────────────────────────────────────────────────────────────
async function startOnboard() {
  await saveConfig();
  const logEl = document.getElementById('onboard-progress-log');
  const summary = document.getElementById('onboard-summary');
  if (logEl) { logEl.innerHTML = ''; logEl.style.display = 'block'; }
  if (summary) { summary.textContent = 'Starting onboard programs…'; summary.style.color = 'var(--amber)'; }
  _onboardProgressSeen.clear();
  const btn = document.getElementById('btn-start-onboard');
  if (btn) btn.disabled = true;

  startOnboardPoll();

  const r = await fetch('/api/onboard/start', { method: 'POST' });
  const d = await r.json();
  if (!d.ok) {
    if (logEl) {
      const div = document.createElement('div');
      div.className = 'pl-step err';
      div.textContent = '✕ ' + d.msg;
      logEl.appendChild(div);
    }
    if (summary) { summary.textContent = '✕ ' + d.msg; summary.style.color = 'var(--red)'; }
    if (btn) btn.disabled = false;
    stopOnboardPoll();
    toast('Onboard start failed: ' + d.msg, 'err');
  }
}

async function stopOnboard() {
  await fetch('/api/onboard/stop', { method: 'POST' });
  toast('Onboard programs stopped');
  const logEl = document.getElementById('onboard-progress-log');
  if (logEl) { logEl.innerHTML = ''; logEl.style.display = 'none'; }
}

async function startTopside() {
  await saveConfig();
  const msg = document.getElementById('topside-msg');
  msg.textContent = 'Starting arm_sender.py…';
  const r = await fetch('/api/topside/start', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(getCfg()),
  });
  const d = await r.json();
  const armRes = d.results && d.results.arm_sender;
  msg.textContent = armRes ? `arm_sender: ${armRes.ok ? '✓ '+armRes.msg : '✕ '+armRes.msg}` : JSON.stringify(d);
  toast(armRes && armRes.ok ? 'Arm sender started' : 'Arm sender failed', armRes && armRes.ok ? 'ok' : 'err');
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
  await fetch('/api/mosfet', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ state: _mosfetOn }),
  });
  updateMosfetUI(_mosfetOn);
  toast('MOSFET ' + (_mosfetOn ? 'ON' : 'OFF'), _mosfetOn ? 'ok' : '');
}

function updateMosfetUI(on) {
  const toggle = document.getElementById('mosfet-toggle');
  const label  = document.getElementById('mosfet-label');
  if (on) { toggle.classList.add('on');    label.textContent = 'MOSFET ON'; }
  else    { toggle.classList.remove('on'); label.textContent = 'MOSFET OFF'; }
}

let _currentMode = 'disarmed';

async function setMode(mode) {
  const r = await fetch('/api/mode', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ mode }),
  });
  const d = await r.json();
  if (d.ok) {
    _currentMode = mode;
    updateModeUI(mode);

    // Auto-configure ctrl flags based on mode
    if (mode === 'disarmed') {
      // Clear all flags (they're ignored anyway, but keep UI clean)
      _ctrlState.stabilize  = false;
      _ctrlState.depth_hold = false;
      _ctrlState.yaw_hold   = false;
    } else if (mode === 'stabilize') {
      // Stabilize mode enables stabilization automatically
      _ctrlState.stabilize = true;
    } else if (mode === 'armed') {
      // Armed: manual control, stabilize off by default
      _ctrlState.stabilize = false;
    }
    updateFlagUI();
  }
  const modeNames = { disarmed:'DISARMED', armed:'DRIVE/ARMED', stabilize:'STABILIZE' };
  toast('Mode: ' + (modeNames[mode] || mode.toUpperCase()), mode === 'disarmed' ? '' : 'ok');
}

function updateModeUI(mode) {
  ['disarmed','armed','stabilize'].forEach(m => {
    const btn = document.getElementById('mode-' + m);
    if (btn) btn.className = 'mode-btn' + (mode === m ? ' active-' + m : '');
  });

  const banner = document.getElementById('disarmed-banner');
  if (banner) banner.classList.toggle('hidden', mode !== 'disarmed');

  const pillMode = document.getElementById('pill-mode');
  const modeColors = { disarmed:'err', armed:'warn', stabilize:'ok' };
  if (pillMode) {
    pillMode.innerHTML = `<span class="status-dot"></span> ${(mode||'--').toUpperCase()}`;
    pillMode.className = 'status-pill ' + (modeColors[mode] || '');
  }
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
// CONTROL FLAG TOGGLES (clickable from control bar + S/D/Y keys)
// ─────────────────────────────────────────────────────────────
function toggleStabilize() {
  if (_currentMode === 'disarmed') { toast('Switch to ARMED or STABILIZE mode first', 'warn'); return; }
  _ctrlState.stabilize = !_ctrlState.stabilize;
  updateFlagUI();
  toast(`Stabilization: ${_ctrlState.stabilize ? 'ON' : 'OFF'}`, _ctrlState.stabilize ? 'ok' : '');
}

function toggleDepthHold() {
  if (_currentMode === 'disarmed') { toast('Switch to ARMED or STABILIZE mode first', 'warn'); return; }
  _ctrlState.depth_hold = !_ctrlState.depth_hold;
  updateFlagUI();
  toast(`Depth Hold: ${_ctrlState.depth_hold ? 'ON' : 'OFF'}`, _ctrlState.depth_hold ? 'ok' : '');
}

function toggleYawHold() {
  if (_currentMode === 'disarmed') { toast('Switch to ARMED or STABILIZE mode first', 'warn'); return; }
  _ctrlState.yaw_hold = !_ctrlState.yaw_hold;
  updateFlagUI();
  toast(`Yaw Hold: ${_ctrlState.yaw_hold ? 'ON' : 'OFF'}`, _ctrlState.yaw_hold ? 'ok' : '');
}

function updateFlagUI() {
  const stabBtn  = document.getElementById('flag-stab');
  const depthBtn = document.getElementById('flag-depth');
  const yawBtn   = document.getElementById('flag-yaw');

  if (stabBtn) {
    stabBtn.textContent = `STAB: ${_ctrlState.stabilize ? 'ON' : 'OFF'}`;
    stabBtn.className   = 'ctrl-flag' + (_ctrlState.stabilize ? ' active-stab' : '');
  }
  if (depthBtn) {
    depthBtn.textContent = `DEPTH: ${_ctrlState.depth_hold ? 'ON' : 'OFF'}`;
    depthBtn.className   = 'ctrl-flag' + (_ctrlState.depth_hold ? ' active-depth' : '');
  }
  if (yawBtn) {
    yawBtn.textContent = `YAW: ${_ctrlState.yaw_hold ? 'ON' : 'OFF'}`;
    yawBtn.className   = 'ctrl-flag' + (_ctrlState.yaw_hold ? ' active-yaw' : '');
  }
}

// ─────────────────────────────────────────────────────────────
// STATUS UPDATES
// ─────────────────────────────────────────────────────────────
function updateStatus() {
  const s = _status;

  setDot('dot-mavproxy', s.onboard_mavproxy);
  setDot('dot-stab',     s.onboard_stab);
  setDot('dot-arm',      s.onboard_arm);
  setDot('dot-armlocal', s.arm_running);

  // Telemetry listener status on launch screen
  const telemDot = document.getElementById('dot-telem-launch');
  const telemLbl = document.getElementById('telem-launch-label');
  if (telemDot) {
    if (s.telemetry_listener_ok) {
      telemDot.className = 'dot running';
      if (telemLbl) telemLbl.textContent = 'Telemetry listener active (UDP 5006)';
    } else {
      telemDot.className = 'dot error';
      if (telemLbl) telemLbl.textContent = 'Telemetry listener FAILED — port 5006 in use?';
    }
  }

  // Replay onboard progress if we connected mid-start
  if (s.onboard_progress && s.onboard_progress.length) {
    const last = s.onboard_progress[s.onboard_progress.length - 1];
    const summary = document.getElementById('onboard-summary');
    if (summary && last.msg) {
      summary.textContent = last.msg;
      summary.style.color = last.status === 'error' ? 'var(--red)'
        : (last.status === 'done' || last.step === 'complete') ? 'var(--green)' : 'var(--amber)';
    }
  }

  const badge = document.getElementById('ssh-status');
  if (s.ssh_connected) {
    badge.textContent = '✓ Connected';
    badge.className   = 'ssh-badge ok';
  } else if (s.ssh_error) {
    badge.textContent = '✕ ' + s.ssh_error.substring(0, 40);
    badge.className   = 'ssh-badge err';
  }

  const pillConn = document.getElementById('pill-connection');
  if (pillConn) {
    pillConn.innerHTML  = `<span class="status-dot"></span> SSH: ${s.ssh_connected ? 'ONLINE' : 'OFFLINE'}`;
    pillConn.className  = 'status-pill ' + (s.ssh_connected ? 'ok' : 'err');
  }

  if (s.mode) { _currentMode = s.mode; updateModeUI(s.mode); }
  if (typeof s.mosfet_on !== 'undefined') { _mosfetOn = s.mosfet_on; updateMosfetUI(s.mosfet_on); }

  const ap = document.getElementById('tel-arm-proc');
  if (ap) {
    ap.textContent = s.arm_running ? 'RUN' : 'STOP';
    ap.className   = 'tc-val ' + (s.arm_running ? 'good' : 'bad');
  }
}

function setDot(id, running) {
  const el = document.getElementById(id);
  if (!el) return;
  el.className = 'dot ' + (running ? 'running' : '');
}

function setDotError(id) {
  const el = document.getElementById(id);
  if (!el) return;
  el.className = 'dot error';
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

  // Telemetry pill
  const pillTel = document.getElementById('pill-telemetry');
  const telOk   = t.rx_state === 'OK';
  if (pillTel) {
    let label = t.rx_state || '--';
    if (label === 'NO_TELEMETRY') label = 'NO TELEM';
    pillTel.innerHTML  = `<span class="status-dot"></span> ${label}`;
    pillTel.title = (t.rx_state === 'NO_TELEMETRY')
      ? 'Start onboard programs, then telemetry appears on UDP 5006'
      : '';
    pillTel.className  = 'status-pill ' + (
      telOk ? 'ok' : (t.rx_state === 'NO_TELEMETRY' ? 'err' : 'warn')
    );
  }

  // Gain
  document.getElementById('tb-gain').textContent = t.gain_percent ?? _ctrlState.gain_percent;

  // Camera overlays
  setText('c1-depth',  fmtNum(t.depth_m, 2));
  setText('c1-hold-d', fmtNum(t.hold_depth_m, 2));
  setText('c1-yaw',    fmtNum(t.yaw_deg, 1));
  setText('c2-roll',   fmtNum(t.roll_deg, 1));
  setText('c2-pitch',  fmtNum(t.pitch_deg, 1));
  const stabEl = document.getElementById('c2-stab');
  if (stabEl) { stabEl.textContent = t.stabilize ? 'ON' : 'OFF'; stabEl.className = t.stabilize ? 'val-hi' : ''; }

  // Telemetry bar
  const state = t.rx_state || '--';
  const stateEl = document.getElementById('tel-state');
  if (stateEl) {
    stateEl.textContent = state;
    stateEl.className   = 'tc-val ' + (state === 'OK' ? 'good' : state === 'NO_TELEMETRY' ? '' : 'warn');
  }

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
  if (dhEl) {
    dhEl.textContent = t.depth_hold_active ? 'HOLD' : (t.depth_hold_request ? 'WAIT' : 'OFF');
    dhEl.className   = 'tc-val ' + (t.depth_hold_active ? 'good' : t.depth_hold_request ? 'warn' : '');
  }
  const yhEl = document.getElementById('tel-yh');
  if (yhEl) {
    yhEl.textContent = t.yaw_hold_active ? 'HOLD' : (t.yaw_hold_request ? 'WAIT' : 'OFF');
    yhEl.className   = 'tc-val ' + (t.yaw_hold_active ? 'good' : t.yaw_hold_request ? 'warn' : '');
  }
}

function updateCtrlCmdsFromTelemetry() {
  // CMD values come from our local gamepad loop, not from telemetry
  // but update gain if telemetry has a different value (e.g. from future sync)
}

function setText(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}

// ─────────────────────────────────────────────────────────────
// GAMEPAD CONTROL — matches thrust_sender.py exactly
// ─────────────────────────────────────────────────────────────

// Config (mirrors thrust_sender.py constants)
const CTRL_CFG = {
  AXIS_LEFT_X:      0,
  AXIS_LEFT_Y:      1,
  AXIS_RIGHT_X:     3,
  AXIS_RIGHT_Y:     4,
  SIGN_YAW:         1.0,
  SIGN_VERTICAL:   -1.0,
  SIGN_LATERAL:     1.0,
  SIGN_FORWARD:    -1.0,
  DEADZONE:         0.05,
  GAIN_MIN:         10,
  GAIN_MAX:         100,
  GAIN_STEP:        10,
  GAIN_DEFAULT:     100,
  BUTTON_STABILIZE: 9,
  COMBINED_LIMIT:   1.50,
  SEND_HZ:          50,
  TELEMETRY_PORT:   5006,
};

let _ctrlLayout = 'original';  // 'original' or 'swapped'

let _ctrlState = {
  stabilize:    false,
  depth_hold:   false,
  yaw_hold:     false,
  gain_percent: CTRL_CFG.GAIN_DEFAULT,
  seq:          0,
};

// Last locally-computed commands (for HUD display even without telemetry)
let _localCmds = { forward: 0, lateral: 0, yaw: 0, vertical: 0 };

// Button / hat edge detection state
let _btnPrev       = [];
let _dpadUpPrev    = false;
let _dpadDownPrev  = false;
let _lastSendMs    = 0;

function _applyDeadzone(x, dz) {
  return Math.abs(x) < dz ? 0.0 : x;
}

function _clamp(x, lo, hi) {
  return Math.max(lo, Math.min(hi, x));
}

function _applyCombinedLimit(f, l, y, v, limit) {
  const h     = Math.max(Math.abs(f), Math.abs(l), Math.abs(y));
  const vAbs  = Math.abs(v);
  const total = h + vAbs;
  if (total <= limit || total <= 1e-6) {
    return { f, l, y, v, scale: 1.0, h, vAbs, total };
  }
  const scale = limit / total;
  return {
    f: f * scale, l: l * scale, y: y * scale, v: v * scale,
    scale, h: h * scale, vAbs: vAbs * scale, total: total * scale,
  };
}

function _adjustGain(delta) {
  _ctrlState.gain_percent = Math.round(
    _clamp(_ctrlState.gain_percent + delta, CTRL_CFG.GAIN_MIN, CTRL_CFG.GAIN_MAX)
  );
  const el = document.getElementById('tb-gain');
  if (el) el.textContent = _ctrlState.gain_percent;
  toast(`Gain: ${_ctrlState.gain_percent}%`);
}

// ── Keyboard handlers (global, match thrust_sender.py keybinds) ──
const _keyDown = {};
document.addEventListener('keydown', (e) => {
  if (_keyDown[e.key]) return;  // key held
  _keyDown[e.key] = true;

  const onCtrl = document.getElementById('control').classList.contains('active');
  if (!onCtrl) return;

  switch (e.key) {
    case 's': case 'S':
      toggleStabilize();
      break;
    case 'd': case 'D':
      toggleDepthHold();
      break;
    case 'y': case 'Y':
      toggleYawHold();
      break;
    case 'ArrowUp':
      e.preventDefault();
      _adjustGain(CTRL_CFG.GAIN_STEP);
      break;
    case 'ArrowDown':
      e.preventDefault();
      _adjustGain(-CTRL_CFG.GAIN_STEP);
      break;
    case 'Escape':
      // Emergency stop
      setMode('disarmed');
      toast('EMERGENCY STOP — DISARMED', 'err');
      break;
  }
});
document.addEventListener('keyup', (e) => { _keyDown[e.key] = false; });

// ── Gamepad connection events ──
let _gamepadActivated = false;

function activateGamepad() {
  _gamepadActivated = true;
  const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
  let found = false;
  for (let i = 0; i < gamepads.length; i++) {
    if (gamepads[i]) { found = true; break; }
  }
  if (found) {
    toast('Gamepad detected!', 'ok');
  } else {
    toast('Press ANY button on your gamepad now…', 'warn');
  }
  _updateGamepadPill();
}

window.addEventListener('gamepadconnected', (e) => {
  _gamepadActivated = true;
  toast(`Gamepad connected: ${e.gamepad.id.substring(0, 40)}`, 'ok');
  _updateGamepadPill();
});
window.addEventListener('gamepaddisconnected', () => {
  toast('Gamepad disconnected!', 'err');
  _updateGamepadPill();
});

function _findGamepad() {
  const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
  for (let i = 0; i < gamepads.length; i++) {
    if (gamepads[i]) return gamepads[i];
  }
  return null;
}

function _updateGamepadPill() {
  const gp = _findGamepad();

  const pill = document.getElementById('pill-gamepad');
  if (pill) {
    if (gp) {
      const name = gp.id.length > 22 ? gp.id.substring(0, 22) + '…' : gp.id;
      pill.innerHTML = `<span class="status-dot"></span> GP: ${name}`;
      pill.className = 'status-pill ok';
    } else {
      pill.innerHTML = `<span class="status-dot"></span> GP: NONE`;
      pill.className = 'status-pill err';
    }
  }

  const launchDot = document.getElementById('dot-gamepad-launch');
  const launchLbl = document.getElementById('gamepad-launch-label');
  if (launchDot) {
    launchDot.className = 'dot ' + (gp ? 'running' : '');
  }
  if (launchLbl) {
    launchLbl.textContent = gp
      ? `Gamepad: ${gp.id.substring(0, 36)}`
      : 'Gamepad — click Activate, then press any button';
  }

  const actBtn = document.getElementById('btn-activate-gp');
  if (actBtn && gp) actBtn.textContent = '✓ Gamepad Active';
}

let _lastHttpCtrlSend = 0;

function sendCtrlPacket(packet) {
  socketEmit('ctrl_packet', packet);
  const nowMs = performance.now();
  const useHttp = !socket || !socket.connected || (nowMs - _lastHttpCtrlSend > 250);
  if (useHttp) {
    _lastHttpCtrlSend = nowMs;
    fetch('/api/ctrl', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(packet),
    }).catch(() => {});
  }
}

// ── Main gamepad + control send loop ──
function _gamepadControlLoop() {
  requestAnimationFrame(_gamepadControlLoop);

  const nowMs = performance.now();
  const intervalMs = 1000 / CTRL_CFG.SEND_HZ;
  if (nowMs - _lastSendMs < intervalMs) return;
  _lastSendMs = nowMs;

  const onCtrl = document.getElementById('control')?.classList.contains('active');
  const gp = _findGamepad();
  _updateGamepadPill();

  // Any button press wakes gamepad in most browsers
  if (gp) {
    for (let b = 0; b < gp.buttons.length; b++) {
      if (gp.buttons[b].pressed) { _gamepadActivated = true; break; }
    }
  }

  let forward = 0, lateral = 0, yaw = 0, vertical = 0;

  if (gp && _gamepadActivated) {
    const leftX  = gp.axes[CTRL_CFG.AXIS_LEFT_X]  || 0;
    const leftY  = gp.axes[CTRL_CFG.AXIS_LEFT_Y]  || 0;
    const rightX = gp.axes[CTRL_CFG.AXIS_RIGHT_X] || 0;
    const rightY = gp.axes[CTRL_CFG.AXIS_RIGHT_Y] || 0;

    let axisYaw, axisLateral;
    if (_ctrlLayout === 'original') {
      axisYaw     = leftX;
      axisLateral = rightX;
    } else {
      axisYaw     = rightX;
      axisLateral = leftX;
    }

    const yawRaw  = _clamp(_applyDeadzone(CTRL_CFG.SIGN_YAW      * axisYaw,    CTRL_CFG.DEADZONE), -1, 1);
    const vertRaw = _clamp(_applyDeadzone(CTRL_CFG.SIGN_VERTICAL  * leftY,      CTRL_CFG.DEADZONE), -1, 1);
    const latRaw  = _clamp(_applyDeadzone(CTRL_CFG.SIGN_LATERAL   * axisLateral,CTRL_CFG.DEADZONE), -1, 1);
    const fwdRaw  = _clamp(_applyDeadzone(CTRL_CFG.SIGN_FORWARD   * rightY,     CTRL_CFG.DEADZONE), -1, 1);

    const gain = _ctrlState.gain_percent / 100.0;
    const r = _applyCombinedLimit(
      fwdRaw * gain, latRaw * gain, yawRaw * gain, vertRaw * gain,
      CTRL_CFG.COMBINED_LIMIT
    );
    forward  = r.f;
    lateral  = r.l;
    yaw      = r.y;
    vertical = r.v;

    const numBtns = gp.buttons.length;
    if (_btnPrev.length !== numBtns) _btnPrev = new Array(numBtns).fill(false);

    for (let b = 0; b < numBtns; b++) {
      const pressed = gp.buttons[b].pressed;
      if (pressed && !_btnPrev[b]) {
        console.log(`[Gamepad] Button ${b} pressed`);
        if (onCtrl && b === CTRL_CFG.BUTTON_STABILIZE) toggleStabilize();
      }
      _btnPrev[b] = pressed;
    }

    const dpadAxisY = (gp.axes.length > 7) ? (gp.axes[7] || 0) : 0;
    const dpadUp   = (gp.buttons[12] && gp.buttons[12].pressed) || dpadAxisY < -0.5;
    const dpadDown = (gp.buttons[13] && gp.buttons[13].pressed) || dpadAxisY > 0.5;
    if (onCtrl) {
      if (dpadUp   && !_dpadUpPrev)   _adjustGain( CTRL_CFG.GAIN_STEP);
      if (dpadDown && !_dpadDownPrev) _adjustGain(-CTRL_CFG.GAIN_STEP);
    }
    _dpadUpPrev   = dpadUp;
    _dpadDownPrev = dpadDown;
  }

  // Show raw stick demand when disarmed (HUD preview) but send zeros to Pi
  const sending = onCtrl && (_currentMode === 'armed' || _currentMode === 'stabilize');
  let sendForward = forward, sendLateral = lateral, sendYaw = yaw, sendVertical = vertical;
  if (!sending) { sendForward = 0; sendLateral = 0; sendYaw = 0; sendVertical = 0; }

  _localCmds = { forward, lateral, yaw, vertical };
  _tel.cmd_forward  = forward;
  _tel.cmd_lateral  = lateral;
  _tel.cmd_yaw      = yaw;
  _tel.cmd_vertical = vertical;

  const fmtCmd = v => (Math.abs(v) < 0.005 ? '0.00' : (v >= 0 ? '+' : '') + v.toFixed(2));
  setText('tel-cmd-f', fmtCmd(sendForward));
  setText('tel-cmd-l', fmtCmd(sendLateral));
  setText('tel-cmd-y', fmtCmd(sendYaw));
  setText('tel-cmd-v', fmtCmd(sendVertical));

  const packet = {
    seq:         _ctrlState.seq++,
    time:        nowMs / 1000,
    forward:     sendForward,
    lateral:     sendLateral,
    yaw:         sendYaw,
    vertical:    sendVertical,
    stabilize:   sending ? _ctrlState.stabilize  : false,
    depth_hold:  sending ? _ctrlState.depth_hold : false,
    yaw_hold:    sending ? _ctrlState.yaw_hold   : false,
    gain_percent: _ctrlState.gain_percent,
    telemetry_port: CTRL_CFG.TELEMETRY_PORT,
  };
  sendCtrlPacket(packet);
}

// ── Axis layout setter ──
function setLayout(layout) {
  _ctrlLayout = layout;

  const orig = document.getElementById('layout-btn-original');
  const swap = document.getElementById('layout-btn-swapped');
  if (orig) { orig.className = 'layout-btn' + (layout === 'original' ? ' active' : ''); }
  if (swap) { swap.className = 'layout-btn' + (layout === 'swapped'  ? ' active' : ''); }

  const disp = document.getElementById('kb-layout-display');
  if (disp) disp.textContent = layout === 'original' ? 'ORIGINAL' : 'SWAPPED';

  const lxl = document.getElementById('kb-axis-left-x-label');
  const lxd = document.getElementById('kb-axis-left-x-desc');
  const rxl = document.getElementById('kb-axis-right-x-label');
  const rxd = document.getElementById('kb-axis-right-x-desc');

  if (layout === 'original') {
    if (lxl) lxl.textContent = 'Left Stick X';
    if (lxd) lxd.textContent = 'Yaw (rotate) — left = turn left, right = turn right';
    if (rxl) rxl.textContent = 'Right Stick X';
    if (rxd) rxd.textContent = 'Lateral strafe — left = strafe left, right = strafe right';
  } else {
    if (lxl) lxl.textContent = 'Left Stick X';
    if (lxd) lxd.textContent = 'Lateral strafe — left = strafe left, right = strafe right';
    if (rxl) rxl.textContent = 'Right Stick X';
    if (rxd) rxd.textContent = 'Yaw (rotate) — left = turn left, right = turn right';
  }

  toast(`Axis layout: ${layout.toUpperCase()}`);
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

  const W  = canvas.width;
  const H  = canvas.height;
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

  // Forward arrow
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
    const endA = startA + sweepA;
    const ax = cx + R * 0.88 * Math.cos(endA);
    const ay = cy + R * 0.88 * Math.sin(endA);
    ctx.beginPath();
    ctx.arc(ax, ay, 4, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(255,100,100,0.9)';
    ctx.fill();
  }

  // Vertical indicator bar
  if (Math.abs(vert) > 0.02) {
    const bx  = cx + R * 1.35;
    const bh  = R * 0.8;
    const mid = cy;
    ctx.beginPath();
    ctx.roundRect(bx - 4, mid - bh, 8, bh * 2, 4);
    ctx.fillStyle = 'rgba(0,212,255,0.08)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(0,212,255,0.2)';
    ctx.lineWidth = 1;
    ctx.stroke();
    const barH = Math.abs(vert) * bh;
    const barY = vert > 0 ? mid - barH : mid;
    ctx.beginPath();
    ctx.roundRect(bx - 4, barY, 8, barH, 3);
    ctx.fillStyle = vert > 0
      ? `rgba(0,224,138,${Math.abs(vert) * 0.7 + 0.3})`
      : `rgba(255,61,90,${Math.abs(vert) * 0.7 + 0.3})`;
    ctx.fill();
    ctx.fillStyle = 'rgba(0,212,255,0.6)';
    ctx.font = `${Math.max(9, R * 0.18)}px monospace`;
    ctx.textAlign = 'center';
    ctx.fillText('V', bx, mid - bh - 4);
  }

  // Mode label at bottom
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
  const angle   = Math.atan2(dy, dx);
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
  const img   = document.getElementById(imgId);
  const noSig = document.getElementById(noSigId);
  let retryT  = null;

  function load() { img.src = `/camera/${camNum}?t=${Date.now()}`; }

  img.onload = () => { noSig.style.display = 'none'; img.style.display = 'block'; };
  img.onerror = () => {
    noSig.style.display = 'flex';
    img.style.display   = 'none';
    if (!retryT) { retryT = setTimeout(() => { retryT = null; load(); }, 5000); }
  };
  load();
}

// ─────────────────────────────────────────────────────────────
// VIEW SWITCHING
// ─────────────────────────────────────────────────────────────
function openControl() {
  activateGamepad();
  document.getElementById('launch').classList.remove('active');
  document.getElementById('control').classList.add('active');
  setupCamera('cam1', 'no-sig-1', 1);
  setupCamera('cam2', 'no-sig-2', 2);
  window.addEventListener('resize', resizeHUDs);
  resizeHUDs();
  startHUDLoop();
  _updateGamepadPill();
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
// KEYBINDS MODAL
// ─────────────────────────────────────────────────────────────
function showKeybinds() {
  document.getElementById('keybinds-modal').style.display = 'flex';
}
function hideKeybinds() {
  document.getElementById('keybinds-modal').style.display = 'none';
}
function hideKeybindsOutside(e) {
  if (e.target === document.getElementById('keybinds-modal')) hideKeybinds();
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
  ['arm','onboard_stab','onboard_arm'].forEach(n => {
    const id = n === 'onboard_stab' ? 'lt-stab' : n === 'onboard_arm' ? 'lt-arm2' : 'lt-arm';
    const el = document.getElementById(id);
    if (el) el.classList.toggle('active', n === name);
  });
  refreshLogView();
}

function refreshLogView() {
  const content = document.getElementById('log-content');
  const lines   = _logs[_currentLog] || [];
  content.innerHTML = lines.map(l =>
    `<div class="log-line">${escapeHtml(l)}</div>`
  ).join('');
  content.scrollTop = content.scrollHeight;
}

function appendLogLine(line) {
  if (!_logOpen) return;
  const content = document.getElementById('log-content');
  const div = document.createElement('div');
  div.className   = 'log-line';
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
  el.className   = 'toast ' + type;
  el.textContent = msg;
  container.appendChild(el);
  setTimeout(() => {
    el.style.opacity    = '0';
    el.style.transition = 'opacity .3s';
    setTimeout(() => el.remove(), 350);
  }, 3500);
}

// ─────────────────────────────────────────────────────────────
// INIT
// ─────────────────────────────────────────────────────────────
window.addEventListener('load', () => {
  // Auto-detect Windows serial port default
  if (navigator.platform.includes('Win') || navigator.userAgent.includes('Windows')) {
    const sp = document.getElementById('cfg-serial_port');
    if (sp && sp.value.startsWith('/dev/')) sp.value = 'COM3';
  }

  // Start gamepad control loop
  requestAnimationFrame(_gamepadControlLoop);
  _updateGamepadPill();

  // Poll status every 2s as WebSocket fallback
  setInterval(() => socketEmit('request_status'), 2000);
  // HTTP fallback when Socket.IO is down
  setInterval(async () => {
    if (socket && socket.connected) return;
    try {
      const r = await fetch('/api/status');
      const d = await r.json();
      _status = d;
      updateStatus();
      if (d.telemetry) { _tel = d.telemetry; updateTelemetry(); }
      if (d.onboard_progress) {
        for (const entry of d.onboard_progress) handleOnboardProgress(entry);
      }
    } catch (_) {}
  }, 2000);
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
