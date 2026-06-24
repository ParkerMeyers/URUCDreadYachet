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

DEFAULT_CONFIG = {
    "pi_ip":               "192.168.2.249",
    "pi_user":             "uruc",
    "pi_password":         "yahboom",
    "pi_ssh_port":         22,
    "pi_rov_path":         "/home/uruc/URUCDreadYachet",
    "serial_port":         "COM3" if IS_WINDOWS else "/dev/ttyACM0",
    "camera1_url":         "http://192.168.2.249:8160",
    "camera2_url":         "http://192.168.2.249:8161",
    "camera0_device":      "/dev/video0",
    "camera1_device":      "/dev/video2",
    "thrust_udp_port":     5005,
    "telemetry_port":      5006,
    "arm_udp_port":        5006,
    "mosfet_control_port": 5007,
    "colmap_command":      "python3 colmap_run.py",
    "crabs_command":       "python3 crabs.py",
    "mavproxy_bin":        "/home/uruc/mav_env/bin/mavproxy.py",
    "mavproxy_serial":     "/dev/ttyACM0",
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
    "onboard_cam":         False,
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
        "onboard_cam":  [],
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

    def start_onboard_process(self, script_rel, log_name, extra_args=""):
        rov_path = config["pi_rov_path"]
        script   = f"{rov_path}/{script_rel}"
        log_file = f"/tmp/rov_{log_name}.log"
        # </dev/null disconnects stdin so the process never hangs waiting for
        # input when launched over an SSH exec channel.
        cmd = (
            f"cd {rov_path} && "
            f"nohup python3 {script} {extra_args} "
            f"< /dev/null > {log_file} 2>&1 & "
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
        # No --daemon: we background with nohup+& and redirect to our log file.
        # --daemon double-forks and makes pgrep unreliable.
        # </dev/null prevents MAVProxy hanging on stdin when launched over SSH.
        cmd = (
            f"nohup {bin_} "
            f"--master={ser} "
            f"--baudrate {baud} "
            f"--out={out1} "
            f"--out={out2} "
            f"< /dev/null > /tmp/rov_mavproxy.log 2>&1 & echo $!"
        )
        out, _, error = self.exec(cmd, timeout=10)
        if error:
            return False, error
        pid = out.strip()
        return True, f"MAVProxy started (PID {pid})"

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
        """Check all four onboard processes in one SSH exec to avoid hammering
        the SSH channel with separate calls every monitoring cycle."""
        cmd = (
            "pgrep -f 'mavproxy'        > /dev/null 2>&1 && echo mavproxy_ok; "
            "pgrep -f 'stabilization.py'> /dev/null 2>&1 && echo stab_ok; "
            "pgrep -f 'new_ar.py'       > /dev/null 2>&1 && echo arm_ok; "
            "pgrep -f 'camera_stream.py'> /dev/null 2>&1 && echo cam_ok; "
            "true"
        )
        out, _, error = self.exec(cmd, timeout=10)
        if error:
            return {"mavproxy": False, "stab": False, "arm": False, "cam": False}
        return {
            "mavproxy": "mavproxy_ok" in out,
            "stab":     "stab_ok"     in out,
            "arm":      "arm_ok"      in out,
            "cam":      "cam_ok"      in out,
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
            else:
                STATE["ssh_connected"]    = False
                STATE["onboard_mavproxy"] = False
                STATE["onboard_stab"]     = False
                STATE["onboard_arm"]      = False
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
                    ssh.is_mavproxy_running, "MAVProxy", timeout_sec=30.0
                )
            if not ok_m:
                mav_log = ssh.get_mavproxy_log(lines=5)
                if mav_log:
                    last_line = mav_log.strip().splitlines()[-1][:150]
                    msg_m = f"{msg_m} | Log: {last_line}"
            STATE["onboard_mavproxy"] = ok_m
            _emit_onboard_progress(
                "mavproxy", "done" if ok_m else "error", msg_m
            )
            emit_status()

            if ok_m:
                for i in range(5):
                    time.sleep(1.0)
                    _emit_onboard_progress(
                        "mavproxy", "wait",
                        f"Waiting for MAVProxy to initialize... ({i + 1}/5)"
                    )

            # Step 2: stabilization.py
            _emit_onboard_progress("stabilization", "starting", "Launching stabilization.py...")
            ok_s, msg_s = ssh.start_onboard_process("onboard/stabilization.py", "stab")
            if ok_s:
                # stabilization.py blocks up to 15 s waiting for MAVLink heartbeat —
                # give it 25 s before declaring failure.
                ok_s, msg_s = _wait_onboard_running(
                    lambda: ssh.is_onboard_running("stabilization.py"),
                    "stabilization.py",
                    timeout_sec=25.0,
                )

            # If SSH dropped during the wait, try a single reconnect.
            # stabilization.py may have started fine — the process is nohup'd
            # and keeps running regardless of SSH state.
            if not ssh.is_connected():
                _emit_onboard_progress("stabilization", "wait",
                                       "SSH dropped — attempting reconnect...")
                ok_r, _ = ssh.connect(
                    config["pi_ip"], config["pi_user"], config["pi_password"],
                    port=int(config["pi_ssh_port"])
                )
                if ok_r:
                    STATE["ssh_connected"] = True
                    time.sleep(1.0)  # let the new connection settle before exec
                    if ssh.is_onboard_running("stabilization.py"):
                        ok_s   = True
                        msg_s  = "stabilization.py running"
                    else:
                        msg_s = "stabilization.py not found after SSH reconnect"
                else:
                    STATE["ssh_connected"] = False
                    msg_s = "stabilization.py timed out and SSH reconnect failed"

            if not ok_s:
                log_tail = ssh.get_onboard_log("stab", lines=8)
                if log_tail:
                    last_line = log_tail.strip().splitlines()[-1][:120]
                    msg_s = f"{msg_s} | Log: {last_line}"
            STATE["onboard_stab"] = ok_s
            _emit_onboard_progress(
                "stabilization", "done" if ok_s else "error", msg_s
            )
            emit_status()

            if not ssh.is_connected():
                _emit_onboard_progress(
                    "arm_ctrl", "error", "SSH session not active — reconnect and retry"
                )
                STATE["onboard_arm"] = False
                _emit_onboard_progress(
                    "complete", "error",
                    "✕ Failed: stabilization — SSH lost during start. Reconnect SSH and retry.",
                )
                emit_status()
                return

            # Step 3: new_ar.py (arm — optional; thrusters work without it)
            _emit_onboard_progress("arm_ctrl", "starting", "Launching new_ar.py (arm controller)...")
            ok_a, msg_a = ssh.start_onboard_process("onboard/new_ar.py", "arm")
            if ok_a:
                ok_a, msg_a = _wait_onboard_running(
                    lambda: ssh.is_onboard_running("new_ar.py"),
                    "new_ar.py",
                    timeout_sec=20.0,
                )

            # If SSH dropped during the wait, try a single reconnect.
            # new_ar.py may have started fine — it's nohup'd and keeps running
            # regardless of SSH state.
            if not ssh.is_connected():
                _emit_onboard_progress("arm_ctrl", "wait",
                                       "SSH dropped — attempting reconnect...")
                ok_r, _ = ssh.connect(
                    config["pi_ip"], config["pi_user"], config["pi_password"],
                    port=int(config["pi_ssh_port"])
                )
                if ok_r:
                    STATE["ssh_connected"] = True
                    time.sleep(1.0)  # let the new connection settle before exec
                    if ssh.is_onboard_running("new_ar.py"):
                        ok_a  = True
                        msg_a = "new_ar.py running"
                    else:
                        msg_a = "new_ar.py not found after SSH reconnect"
                else:
                    STATE["ssh_connected"] = False
                    msg_a = "new_ar.py timed out and SSH reconnect failed"

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

            # Step 4: camera_stream.py (optional — ROV works without cameras)
            # SSH can drop during the arm _wait_onboard_running poll; try to
            # reconnect once before attempting the camera launch.
            if not ssh.is_connected():
                _emit_onboard_progress("camera", "wait",
                                       "SSH dropped — reconnecting for camera start...")
                ok_r, _ = ssh.connect(
                    config["pi_ip"], config["pi_user"], config["pi_password"],
                    port=int(config["pi_ssh_port"])
                )
                STATE["ssh_connected"] = ok_r
                if not ok_r:
                    STATE["onboard_cam"] = False
                    _emit_onboard_progress(
                        "camera", "error",
                        "SSH reconnect failed — cameras not started (ROV still operational)"
                    )
                    emit_status()
                    ok_c = False
                    # Jump straight to summary
                    core_ok = ok_m and ok_s
                    if core_ok and ok_a:
                        summary = "✓ ROV ready — cameras not started (SSH dropped, reconnect and retry)"
                    elif core_ok:
                        summary = "✓ Thruster control ready — arm + cameras not started (SSH dropped)"
                    else:
                        summary = "✕ SSH dropped during startup — reconnect and retry"
                    _emit_onboard_progress("complete", "done" if core_ok else "error", summary)
                    emit_status()
                    return

            _emit_onboard_progress("camera", "starting", "Launching camera_stream.py (MJPEG feeds)...")
            cam0_dev = config.get("camera0_device", "/dev/video0")
            cam1_dev = config.get("camera1_device", "/dev/video2")
            cam_args = f"--cam0 {cam0_dev} --cam1 {cam1_dev}"
            ok_c, msg_c = ssh.start_onboard_process(
                "onboard/camera_stream.py", "cam", extra_args=cam_args
            )
            if ok_c:
                ok_c, msg_c = _wait_onboard_running(
                    lambda: ssh.is_onboard_running("camera_stream.py"),
                    "camera_stream.py",
                    timeout_sec=12.0,
                )
            if not ok_c:
                log_tail = ssh.get_onboard_log("cam", lines=8)
                if log_tail:
                    last_line = log_tail.strip().splitlines()[-1][:120]
                    msg_c = f"{msg_c} | Log: {last_line}"
            STATE["onboard_cam"] = ok_c
            _emit_onboard_progress(
                "camera", "done" if ok_c else "error", msg_c
            )
            emit_status()

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
    ssh.stop_onboard_process("stabilization.py")
    ssh.stop_onboard_process("new_ar.py")
    ssh.stop_onboard_process("camera_stream.py")
    ssh.stop_mavproxy()
    STATE["onboard_stab"]     = False
    STATE["onboard_arm"]      = False
    STATE["onboard_cam"]      = False
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
