"""SSH connection to Pi — onboard process control and file sync."""
from __future__ import annotations

import json
import shlex
import socket
import threading
import time
from pathlib import Path

from topside.config import ROV_ROOT, config, normalize_onboard_config
from topside.constants import (
    MAVPROXY_ARM_ONBOARD_OUT,
    MAVPROXY_ARM_TCP_PORT,
    MAVPROXY_ONBOARD_OUT,
    MAVPROXY_TCP_PORT,
)

try:
    import paramiko
    HAVE_PARAMIKO = True
except ImportError:
    HAVE_PARAMIKO = False


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
            t = client.get_transport()
            if t:
                t.set_keepalive(5)
            with self._lock:
                if self._client:
                    try:
                        self._client.close()
                    except Exception:
                        pass
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
            return False, f"Network error reaching {host}:{port} — {e}"
        except Exception as e:
            return False, str(e)

    def disconnect(self):
        with self._lock:
            if self._client:
                try:
                    self._client.close()
                except Exception:
                    pass
                self._client = None

    def is_connected(self):
        with self._lock:
            if self._client is None:
                return False
            try:
                t = self._client.get_transport()
                return t is not None and t.is_active()
            except Exception:
                return False

    def _invalidate_client(self):
        if self._client:
            try:
                self._client.close()
            except Exception:
                pass
            self._client = None

    def exec(self, cmd, timeout=20):
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
        try:
            out = stdout.read().decode("utf-8", errors="replace")
            err = stderr.read().decode("utf-8", errors="replace")
            return out.strip(), err.strip(), None
        except Exception as e:
            with self._lock:
                self._invalidate_client()
            return "", "", str(e)

    def _supervisor_cmd(self, *args, timeout=90):
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
        key = {"stabilization.py": "stab", "new_ar.py": "arm", "camera_stream.py": "cam"}.get(script_name, "")
        if not key:
            return False
        return bool(self.supervisor_status().get(key, {}).get("alive"))

    def stop_onboard_process(self, script_name):
        key = {"stabilization.py": "stab", "new_ar.py": "arm", "camera_stream.py": "cam"}.get(script_name)
        if key:
            self._supervisor_cmd("stop", key, timeout=15)

    def start_onboard_process(self, script_rel, log_name, extra_args=""):
        return self.supervisor_start_and_wait(log_name, 50.0, extra_args)

    def get_onboard_log(self, log_name, lines=20):
        log_file = f"/tmp/rov_{log_name}.log"
        out, _, _ = self.exec(f"tail -n {lines} {log_file} 2>/dev/null || echo ''")
        return out

    def _release_serial_port(self, ser: str) -> None:
        if not ser:
            return
        q = shlex.quote(ser)
        self.exec(
            f"fuser -k {q} 2>/dev/null; "
            "pkill -f mavproxy 2>/dev/null; pkill -f MAVProxy 2>/dev/null; "
            "sleep 0.5"
        )

    def _start_mavproxy_fresh(self):
        normalize_onboard_config()
        bin_ = config["mavproxy_bin"]
        ser = config["mavproxy_serial"]
        if not self._device_exists(ser):
            return False, (
                f"{ser} not found on Pi — plug Pix6 USB into the Raspberry Pi "
                f"(not the topside laptop). {self.mavproxy_diagnosis()}"
            )
        baud = int(config.get("mavproxy_baud", 115200))
        out1, out2 = config["mavproxy_out1"], config["mavproxy_out2"]
        out3 = config.get("mavproxy_out3", MAVPROXY_ARM_ONBOARD_OUT)
        self._release_serial_port(ser)
        self.exec("truncate -s 0 /tmp/rov_mavproxy.log 2>/dev/null || true")
        cmd = (
            f"setsid nohup {shlex.quote(bin_)} "
            f"--master={shlex.quote(ser)} --baudrate {baud} "
            f"--non-interactive --no-state "
            f"--out={out1} --out={out2} --out={out3} "
            f"< /dev/null >> /tmp/rov_mavproxy.log 2>&1 & echo $!"
        )
        out, _, error = self.exec(cmd, timeout=10)
        if error:
            return False, error
        pid = out.strip()
        if not pid.isdigit():
            tail = self.get_mavproxy_log(lines=8)
            hint = tail.strip().splitlines()[-1][:120] if tail.strip() else "no log output"
            return False, f"MAVProxy failed to start — {hint}"
        return True, f"MAVProxy started (PID {pid})"

    def ensure_mavproxy(self):
        normalize_onboard_config()
        if self.is_mavproxy_running() and self.is_mavproxy_fc_connected() and self.is_mavproxy_tcp_ready():
            return True, "MAVProxy already running — Pix6 online"
        return self._start_mavproxy_fresh()

    def start_mavproxy(self):
        return self._start_mavproxy_fresh()

    def is_mavproxy_tcp_port_ready(self, port: int) -> bool:
        out, _, _ = self.exec(
            f"(ss -tln 2>/dev/null || netstat -tln 2>/dev/null) | grep -q ':{port} ' && echo ok"
        )
        return "ok" in out

    def is_mavproxy_tcp_ready(self):
        return (
            self.is_mavproxy_tcp_port_ready(MAVPROXY_TCP_PORT)
            and self.is_mavproxy_tcp_port_ready(MAVPROXY_ARM_TCP_PORT)
        )

    def wait_mavproxy_tcp_ready(self, timeout_sec: float = 20.0) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self.is_mavproxy_tcp_ready():
                return True
            time.sleep(1.0)
        return False

    _MAVPROXY_ALIVE_MARKERS = (
        "detected vehicle", "online system", "got command_ack",
        "vcc ", "ap:", "flight battery", "heartbeat", "fence present",
        "manual>", "received ", "saved ", "parameters",
    )

    def is_mavproxy_fc_connected(self):
        if not self.is_mavproxy_running() or self.mavproxy_recent_no_link():
            return False
        out, _, _ = self.exec("tail -n 30 /tmp/rov_mavproxy.log 2>/dev/null || true")
        lines = [ln.strip() for ln in out.strip().splitlines() if ln.strip()]
        if not lines:
            return False
        recent_text = "\n".join(lines[-15:]).lower()
        if "unloading module" in recent_text:
            return False
        trailing = sum(
            1 for ln in reversed(lines[-15:])
            if "no link" in ln.lower() or "link down" in ln.lower()
        )
        if trailing >= 3:
            return False
        return any(m in recent_text for m in self._MAVPROXY_ALIVE_MARKERS)

    def _device_exists(self, path: str) -> bool:
        out, _, _ = self.exec(f"test -e {shlex.quote(path)} && echo exists")
        return "exists" in out

    def mavproxy_serial_candidates(self):
        preferred = config.get("mavproxy_serial", "/dev/ttyACM0")
        ordered = []
        if self._device_exists(preferred):
            ordered.append(preferred)
        for dev in sorted(self.list_serial_candidates()):
            if dev not in ordered:
                ordered.append(dev)
        return ordered

    def serial_port_exists(self):
        return self._device_exists(config.get("mavproxy_serial", "/dev/ttyACM0"))

    def any_serial_port_exists(self):
        return bool(self.mavproxy_serial_candidates())

    def list_serial_candidates(self):
        """USB/UART device nodes that may be the Pixhawk (on the Pi)."""
        out, _, _ = self.exec(
            "ls -1 /dev/ttyACM* /dev/ttyUSB* /dev/serial/by-id/* 2>/dev/null || true"
        )
        seen = set()
        ordered = []
        for ln in out.strip().splitlines():
            path = ln.strip()
            if not path or path in seen:
                continue
            seen.add(path)
            ordered.append(path)
        return ordered

    def _usb_lsusb_summary(self) -> str:
        out, _, _ = self.exec("lsusb 2>/dev/null || true")
        lines = [ln.strip() for ln in out.strip().splitlines() if ln.strip()]
        if not lines:
            return "lsusb: no USB devices (or lsusb missing)"
        # Highlight common FC vendors: Hex/ProfiCNC 2dae, 3DR 26ac, Cube 1209:5740, STM 0483
        fc_markers = ("2dae:", "26ac:", "1209:5740", "0483:")
        fc_hits = [ln for ln in lines if any(m in ln.lower() for m in fc_markers)]
        if fc_hits:
            return "FC USB seen but no /dev/ttyACM* — driver or cable issue: " + fc_hits[0][:80]
        return f"{len(lines)} USB device(s), no Pixhawk-like ID — " + lines[0][:70]

    def _usb_recent_dmesg(self) -> str:
        out, _, _ = self.exec(
            "dmesg 2>/dev/null | grep -iE 'usb|ttyACM|ttyUSB|cdc_acm' | tail -5 || true"
        )
        lines = [ln.strip() for ln in out.strip().splitlines() if ln.strip()]
        return lines[-1][:100] if lines else ""

    def wait_for_mavproxy_fc(self, on_wait=None) -> tuple[bool, str]:
        time.sleep(2.0)
        deadline = time.time() + 45.0
        fc_wait_i = miss_running = 0
        ser = config.get("mavproxy_serial", "/dev/ttyACM0")
        while time.time() < deadline:
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
        return False, "MAVProxy running but Pix6 not detected — " + self.mavproxy_diagnosis()

    def mavproxy_recent_no_link(self):
        out, _, _ = self.exec("tail -n 5 /tmp/rov_mavproxy.log 2>/dev/null || true")
        lines = [ln.strip() for ln in out.strip().splitlines() if ln.strip()]
        if len(lines) < 3:
            return False
        return all("no link" in ln.lower() or "link down" in ln.lower() for ln in lines[-3:])

    def mavproxy_diagnosis(self):
        ser = config.get("mavproxy_serial", "/dev/ttyACM0")
        parts = []
        if not self.any_serial_port_exists():
            parts.append(f"{ser} not found")
            alts = self.list_serial_candidates()
            if alts:
                parts.append(f"try: {', '.join(alts)}")
            else:
                parts.append("no /dev/ttyACM* or /dev/ttyUSB* on Pi")
                parts.append(self._usb_lsusb_summary())
                dmesg_hint = self._usb_recent_dmesg()
                if dmesg_hint:
                    parts.append(f"dmesg: {dmesg_hint}")
                parts.append(
                    "Check: Pix6 USB → Pi (not laptop), data cable, powered on, "
                    "then run on Pi: ls -l /dev/ttyACM*"
                )
        elif not self.serial_port_exists():
            parts.append(f"{ser} not found")
            alts = self.list_serial_candidates()
            parts.append(f"available: {', '.join(alts)}" if alts else "no serial devices")
        else:
            parts.append(f"{ser} exists but MAVProxy reports no link")
        log_tail = self.get_mavproxy_log(lines=20)
        if log_tail and self.any_serial_port_exists():
            lower = log_tail.lower()
            if "not enough free disk space" in lower or "flight logs full" in lower:
                parts.append("Pi disk full — MAVProxy could not open tlog files")
            if "log_writer" in lower:
                parts.append("log_writer thread crashed — restart onboard")
            if "permission denied" in lower:
                parts.append(f"permission denied on {ser} — run: sudo usermod -aG dialout uruc")
            if "multiple access" in lower or "returned no data" in lower:
                parts.append("serial port busy")
            for ln in reversed(log_tail.strip().splitlines()):
                if ln.strip() and ln.strip().lower() not in ("no link", "link 1 down", "link down"):
                    parts.append(f"log: {ln.strip()[:100]}")
                    break
        df_out, _, _ = self.exec("df -h / 2>/dev/null | tail -1")
        if df_out.strip() and any(p in df_out.strip() for p in ("100%", "99%", "98%")):
            parts.append(f"disk nearly full: {df_out.strip()[:70]}")
        return " — ".join(parts)

    def stop_mavproxy(self):
        self._release_serial_port(config.get("mavproxy_serial", "/dev/ttyACM0"))

    def is_mavproxy_running(self):
        out, _, error = self.exec("pgrep -f 'mavproxy'")
        return not error and bool(out.strip())

    def get_onboard_status(self):
        st = self.supervisor_status()
        if not st:
            return {"mavproxy": False, "stab": False, "arm": False, "cam": False}
        return {
            "mavproxy": self.is_mavproxy_running() and self.is_mavproxy_fc_connected(),
            "stab": bool(st.get("stab", {}).get("alive")),
            "arm": bool(st.get("arm", {}).get("alive")),
            "cam": bool(st.get("cam", {}).get("alive")),
        }

    def get_mavproxy_log(self, lines=10):
        out, _, _ = self.exec(f"tail -n {lines} /tmp/rov_mavproxy.log 2>/dev/null || echo ''")
        return out

    def run_colmap(self):
        rov_path = config["pi_rov_path"]
        _, _, error = self.exec(f"cd {rov_path} && nohup {config['colmap_command']} > /tmp/rov_colmap.log 2>&1 &")
        return error is None, error or "started"

    def run_crabs(self):
        rov_path = config["pi_rov_path"]
        _, _, error = self.exec(f"cd {rov_path} && nohup {config['crabs_command']} > /tmp/rov_crabs.log 2>&1 &")
        return error is None, error or "started"

    def sync_onboard_files(self):
        with self._lock:
            if self._client is None:
                return False, "Not connected"
            try:
                sftp = self._client.open_sftp()
            except Exception as e:
                return False, f"SFTP channel failed: {e}"
        remote_onboard = f"{config['pi_rov_path']}/onboard"
        local_onboard = ROV_ROOT / "onboard"
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
