#!/usr/bin/env python3
"""Main UI and control launcher for the ROV project.

Ties together topside senders and onboard control programs:
- Launch screen: SSH onboard (stabilization.py, new_ar.py) + local topside (arm_sender, thrust_sender)
- Control screen: dual cameras, heading compass, telemetry, MOSFET, mode, Colmap/Crabs, system bar

Run: python main_control_ui.py
"""

import argparse
import importlib.util
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from tkinter import messagebox
import tkinter as tk
from tkinter import ttk
from tkinter import font as tkfont


HAVE_CV2 = importlib.util.find_spec("cv2") is not None
if HAVE_CV2:
    cv2 = importlib.import_module("cv2")
else:
    cv2 = None

HAVE_PIL = importlib.util.find_spec("PIL") is not None
if HAVE_PIL:
    PIL = importlib.import_module("PIL")
    Image = PIL.Image
    ImageTk = PIL.ImageTk
else:
    Image = None
    ImageTk = None
    HAVE_PIL = False


# Optional: prefer a pure-Python SSH path (paramiko) when available. This avoids
# relying on sshpass or platform ssh that may prompt on a TTY and cause the
# GUI-launcher subprocess to hang/timeout even though SSH works from an
# interactive terminal.
HAVE_PARAMIKO = importlib.util.find_spec("paramiko") is not None
if HAVE_PARAMIKO:
    paramiko = importlib.import_module("paramiko")
else:
    paramiko = None


TELEMETRY_PORT = 5007
TELEMETRY_TIMEOUT_SEC = 1.0
REMOTE_CONNECT_TIMEOUT_SEC = 10
REMOTE_LAUNCH_TIMEOUT_SEC = 45
UI_REFRESH_MS = 120
UI_MODE_FILENAME = "rov_ui_mode.json"


class UiModeBridge:
    """Writes the operator-selected mode for thrust_sender to consume."""

    def __init__(self, path):
        self.path = Path(path)

    def write(self, mode):
        payload = {"mode": mode, "time": time.time()}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle)

    def read(self):
        try:
            with open(self.path, "r", encoding="utf-8") as handle:
                return json.load(handle).get("mode", "Drive/Armed")
        except Exception:
            return "Drive/Armed"


class TelemetryReceiver:
    def __init__(self, port):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.latest_payload = {}
        self.last_update = 0.0

    def poll(self):
        updated = False
        while True:
            try:
                data, _addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError:
                break

            try:
                payload = json.loads(data.decode("utf-8"))
            except Exception:
                continue

            self.latest_payload = payload
            self.last_update = time.time()
            updated = True

        return self.latest_payload if updated else None

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


class ProcessController:
    def __init__(self, repo_root, args):
        self.repo_root = Path(repo_root).resolve()
        self.args = args
        self.log_dir = self.repo_root / "logs"
        self.log_dir.mkdir(exist_ok=True)

        self.local_processes = {}
        self.local_log_handles = {}
        self.remote_ready = False
        self.ssh_executable = None
        self.sshpass_executable = None

    def _local_script(self, relative_path):
        return self.repo_root / relative_path

    def _resolve_executable(self, candidates, extra_paths=None):
        for name in candidates:
            resolved = shutil.which(name)
            if resolved:
                return resolved

        if extra_paths:
            for path in extra_paths:
                if os.path.exists(path):
                    return path

        return None

    def _resolve_ssh_executable(self):
        if os.name == "nt":
            system_root = os.environ.get("SystemRoot", r"C:\Windows")
            system32 = os.path.join(system_root, "System32")
            open_ssh_dir = os.path.join(system32, "OpenSSH")
            extra_paths = [
                os.path.join(open_ssh_dir, "ssh.exe"),
                os.path.join(open_ssh_dir, "ssh"),
                os.path.join(system32, "ssh.exe"),
                os.path.join(system32, "ssh"),
                os.path.join(system_root, "Sysnative", "OpenSSH", "ssh.exe"),
                os.path.join(system_root, "Sysnative", "OpenSSH", "ssh"),
                r"C:\Program Files\OpenSSH\ssh.exe",
                r"C:\Program Files\OpenSSH\ssh",
                r"C:\Program Files\Git\usr\bin\ssh.exe",
                r"C:\Program Files\Git\bin\ssh.exe",
            ]
            return self._resolve_executable(["ssh", "ssh.exe"], extra_paths=extra_paths)

        return self._resolve_executable(["ssh", "ssh.exe"])

    def _ensure_ssh_available(self):
        self.ssh_executable = self._resolve_ssh_executable()
        if not self.ssh_executable:
            raise RuntimeError(
                "SSH was not found on PATH or in the common Windows OpenSSH locations. "
                "Install OpenSSH or update the onboard host settings."
            )
        return self.ssh_executable

    def _resolve_sshpass(self):
        if self.sshpass_executable is None:
            self.sshpass_executable = self._resolve_executable(["sshpass", "sshpass.exe"])
        return self.sshpass_executable

    def _build_ssh_command(self, remote_command):
        ssh_executable = self._ensure_ssh_available()
        ssh_args = [
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            f"ConnectTimeout={REMOTE_CONNECT_TIMEOUT_SEC}",
            "-o",
            "BatchMode=no",
            "-o",
            "PreferredAuthentications=password",
            "-o",
            "PubkeyAuthentication=no",
            "-o",
            "NumberOfPasswordPrompts=1",
        ]
        target = str(self.args.onboard_host)
        user = getattr(self.args, "onboard_user", None)
        if user:
            target = f"{str(user)}@{target}"

        password = getattr(self.args, "onboard_password", None)
        if isinstance(password, str) and password:
            sshpass_path = self._resolve_sshpass()
            if sshpass_path is not None:
                return [str(sshpass_path), "-p", password, ssh_executable, *ssh_args, target, str(remote_command)]

        return [ssh_executable, *ssh_args, target, str(remote_command)]

    def _run_remote_command(self, remote_command, timeout_sec=REMOTE_LAUNCH_TIMEOUT_SEC):
        # Prefer a paramiko-based SSH execution when we have a password but no
        # sshpass available. Many SSH clients prompt on a TTY for passwords and
        # will not read a password from stdin; that causes the subprocess to
        # block until the timeout even though interactive SSH works.
        password = getattr(self.args, "onboard_password", None)
        sshpass_present = self._resolve_sshpass() is not None
        if isinstance(password, str) and password and not sshpass_present and paramiko is not None:
            try:
                return self._run_remote_command_paramiko(remote_command, timeout_sec=timeout_sec)
            except Exception:
                pass

        command = self._build_ssh_command(remote_command)
        ssh_input = None
        if isinstance(password, str) and password and not sshpass_present:
            # As a fallback we attempt to write the password to stdin. This
            # will not work with all ssh builds (some read from /dev/tty), but
            # it's retained for environments where it is supported.
            ssh_input = f"{password}\n"

        try:
            return subprocess.run(
                command,
                capture_output=True,
                text=True,
                input=ssh_input,
                timeout=timeout_sec,
                env=os.environ.copy(),
            )
        except subprocess.TimeoutExpired as exc:
            stdout = (exc.stdout or "").strip()
            stderr = (exc.stderr or "").strip()
            details = stderr or stdout or "No output"
            raise RuntimeError(
                "SSH timed out while talking to the onboard computer. "
                "Make sure the Pi is powered on, reachable on the network, and that SSH is enabled. "
                f"Details: {details}"
            ) from exc
        except OSError as exc:
            raise RuntimeError(f"SSH error while talking to the onboard computer: {exc}") from exc

    def _run_remote_command_paramiko(self, remote_command, timeout_sec=REMOTE_LAUNCH_TIMEOUT_SEC):
        """Execute a remote command using Paramiko (pure-Python SSH client).

        Returns an object with attributes: returncode, stdout, stderr (to match
        subprocess.CompletedProcess-like usage in the rest of the code).
        """
        if paramiko is None:
            raise RuntimeError("Paramiko is not installed")

        host = str(self.args.onboard_host)
        user = getattr(self.args, "onboard_user", None)
        password = getattr(self.args, "onboard_password", None)

        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            client.connect(hostname=host, username=user or None, password=password or None, timeout=REMOTE_CONNECT_TIMEOUT_SEC)
            stdin, stdout_fh, stderr_fh = client.exec_command(remote_command, timeout=timeout_sec)
            stdout = stdout_fh.read().decode("utf-8", errors="replace")
            stderr = stderr_fh.read().decode("utf-8", errors="replace")
            # Paramiko provides an exit status on the channel
            try:
                returncode = stdout_fh.channel.recv_exit_status()
            except Exception:
                returncode = 0

            result = type("R", (), {})()
            result.returncode = returncode
            result.stdout = stdout
            result.stderr = stderr
            return result
        except paramiko.ssh_exception.AuthenticationException as exc:
            raise RuntimeError("SSH authentication failed: check username/password") from exc
        except (paramiko.SSHException, OSError, socket.timeout) as exc:
            raise RuntimeError(f"SSH (paramiko) error connecting to {host}: {exc}") from exc
        finally:
            try:
                client.close()
            except Exception:
                pass

    def _check_remote_setup(self):
        remote_root = str(self.args.onboard_root)
        check_command = (
            f'test -d "{remote_root}" && '
            f'test -f "{remote_root}/onboard/stabilization.py" && '
            f'test -f "{remote_root}/onboard/new_ar.py" && '
            'echo remote-ready'
        )
        result = self._run_remote_command(check_command, timeout_sec=REMOTE_LAUNCH_TIMEOUT_SEC)
        if result.returncode != 0:
            raise RuntimeError(
                "The onboard side is not set up yet or the remote root is wrong. "
                f"Expected files at {remote_root}/onboard/stabilization.py and "
                f"{remote_root}/onboard/new_ar.py. Copy the repo to the Pi and verify SSH access first."
            )

    def _launch_remote_program(self, program_name, remote_script_path, remote_log_path):
        remote_command = (
            f'cd "{self.args.onboard_root}" && '
            f'nohup python3 -u "{remote_script_path}" > "{remote_log_path}" 2>&1 < /dev/null & '
            f'echo "{program_name}:launched"'
        )
        result = self._run_remote_command(remote_command, timeout_sec=REMOTE_LAUNCH_TIMEOUT_SEC)
        if result.returncode != 0:
            raise RuntimeError(
                f"Could not launch {program_name} over SSH: "
                f"{result.stderr.strip() or result.stdout.strip()}"
            )
        return result

    def _build_onboard_hardware_stop_command(self):
        return (
            'pkill -f "onboard/stabilization.py" || true; '
            'pkill -f "onboard/new_ar.py" || true; '
            "sleep 0.25; "
            "python3 - <<'PY'\n"
            "import time\n"
            "try:\n"
            "    from smbus2 import SMBus\n"
            "    bus = SMBus(1)\n"
            "    addr = 0x40\n"
            "    for ch in range(16):\n"
            "        reg = 0x06 + 4 * ch\n"
            "        bus.write_i2c_block_data(addr, reg, [0, 0, 0, 0])\n"
            "    bus.close()\n"
            "    print('PCA9685 outputs cleared')\n"
            "except Exception as exc:\n"
            "    print(f'PCA9685 stop warning: {exc}')\n"
            "try:\n"
            "    import lgpio\n"
            "    handle = lgpio.gpiochip_open(0)\n"
            "    try:\n"
            "        lgpio.gpio_claim_output(handle, 17, 0)\n"
            "        lgpio.gpio_write(handle, 17, 0)\n"
            "    finally:\n"
            "        lgpio.gpiochip_close(handle)\n"
            "    print('MOSFET GPIO pulled low')\n"
            "except Exception as exc:\n"
            "    print(f'MOSFET stop warning: {exc}')\n"
            "open('/tmp/uru_mosfet_state', 'w', encoding='utf-8').write('0')\n"
            "PY"
        )

    def _stop_onboard_hardware(self):
        self._ensure_ssh_available()
        self._run_remote_command(
            self._build_onboard_hardware_stop_command(),
            timeout_sec=20,
        )

    def set_mosfet_state(self, enabled):
        state_value = 1 if enabled else 0
        remote_command = (
            "python3 - <<'PY'\n"
            "import lgpio\n"
            "handle = lgpio.gpiochip_open(0)\n"
            "try:\n"
            "    lgpio.gpio_claim_output(handle, 17, 0)\n"
            f"    lgpio.gpio_write(handle, 17, {state_value})\n"
            "finally:\n"
            "    lgpio.gpiochip_close(handle)\n"
            f"open('/tmp/uru_mosfet_state', 'w', encoding='utf-8').write('{state_value}')\n"
            "PY"
        )
        result = self._run_remote_command(remote_command, timeout_sec=REMOTE_LAUNCH_TIMEOUT_SEC)
        if result.returncode != 0:
            raise RuntimeError(
                f"Could not set MOSFET state over SSH: {result.stderr.strip() or result.stdout.strip()}"
            )
        return result

    def launch_remote_mission(self, mission_name, remote_command, log_path):
        wrapped = (
            f'nohup bash -lc {json.dumps(remote_command)} > {json.dumps(log_path)} 2>&1 < /dev/null & '
            f'echo "{mission_name}:launched"'
        )
        result = self._run_remote_command(wrapped, timeout_sec=REMOTE_LAUNCH_TIMEOUT_SEC)
        if result.returncode != 0:
            raise RuntimeError(
                f"Could not launch {mission_name}: {result.stderr.strip() or result.stdout.strip()}"
            )
        return result

    def stop_remote_mission(self, mission_name, process_pattern):
        stop_command = f'pkill -f {json.dumps(process_pattern)} || true; echo "{mission_name}:stopped"'
        return self._run_remote_command(stop_command, timeout_sec=20)

    def _launch_topside_programs_visible(self):
        results = {}
        for name, rel_path in [
            ("arm_sender", Path("topside") / "arm_sender.py"),
            ("thrust_sender", Path("topside") / "thrust_sender.py"),
        ]:
            script_path = self._local_script(rel_path)
            if not script_path.exists():
                raise FileNotFoundError(f"Topside script not found: {script_path}")

            log_path = self.log_dir / f"{name}.log"
            log_handle = open(log_path, "a", encoding="utf-8")
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            if name == "thrust_sender":
                env["ROV_TELEMETRY_UI_PORT"] = str(TELEMETRY_PORT)
                env["ROV_UI_MODE_FILE"] = str(self.log_dir / UI_MODE_FILENAME)

            launch_args = [sys.executable, str(script_path)]
            if name in {"arm_sender", "thrust_sender"}:
                launch_args.append(self.args.onboard_host)

            subprocess_kwargs = {
                "cwd": str(self.repo_root),
                "stdout": log_handle,
                "stderr": subprocess.STDOUT,
                "env": env,
            }
            if os.name == "nt":
                subprocess_kwargs["creationflags"] = getattr(subprocess, "CREATE_NEW_CONSOLE", 0)
            else:
                subprocess_kwargs["start_new_session"] = True

            process = subprocess.Popen(launch_args, **subprocess_kwargs)
            self.local_processes[name] = process
            self.local_log_handles[name] = log_handle
            results[name] = {"status": "started", "log": str(log_path)}

        return results

    def launch_topside_programs(self):
        return self._launch_topside_programs_visible()

    def _launch_onboard_programs_remote(self):
        self._ensure_ssh_available()
        self._check_remote_setup()

        remote_root = str(self.args.onboard_root)
        remote_logs = {
            "stabilization": "/tmp/uru_stabilization.log",
            "new_ar": "/tmp/uru_new_ar.log",
        }

        launch_plan = [
            ("stabilization", "onboard/stabilization.py", remote_logs["stabilization"]),
            ("new_ar", "onboard/new_ar.py", remote_logs["new_ar"]),
        ]

        for program_name, remote_script_path, remote_log_path in launch_plan:
            self._launch_remote_program(program_name, remote_script_path, remote_log_path)

        try:
            self.set_mosfet_state(False)
        except Exception:
            pass

        self.remote_ready = True
        return {
            "status": "started",
            "host": self.args.onboard_host,
            "user": self.args.onboard_user,
            "remote_root": remote_root,
            "logs": remote_logs,
        }

    def launch_onboard_programs(self):
        return self._launch_onboard_programs_remote()

    def stop_all(self):
        for name, process in list(self.local_processes.items()):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
            handle = self.local_log_handles.pop(name, None)
            if handle is not None:
                handle.close()
            self.local_processes.pop(name, None)

        if self.remote_ready:
            try:
                self._stop_onboard_hardware()
            except Exception:
                try:
                    self.set_mosfet_state(False)
                except Exception:
                    pass
            self.remote_ready = False

    def get_process_state(self, name):
        process = self.local_processes.get(name)
        if process is None:
            return "not_started"
        if process.poll() is None:
            return "running"
        return "stopped"


class ROVMainApp(tk.Tk):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.title("ROV Main Control UI")
        self.geometry("1280x860")
        self.minsize(1100, 780)
        self.configure(padx=16, pady=16)

        self.controller = ProcessController(self.args.repo_root, self.args)
        self.mode_bridge = UiModeBridge(Path(self.args.repo_root) / "logs" / UI_MODE_FILENAME)
        self.launch_frame = None
        self.control_frame = None

        self.topside_started = False
        self.onboard_started = False
        self._onboard_launch_in_progress = False
        self.mosfet_enabled = False
        self.control_mode = "Stabilization"
        self.colmap_running = False
        self.crabs_running = False
        self.colmap_button = None
        self.crabs_button = None
        self.colmap_cmd = os.getenv("ROV_COLMAP_CMD", "colmap automatic_reconstructor --workspace_path /home/uruc/colmap_ws")
        self.crabs_cmd = os.getenv("ROV_CRABS_CMD", "python3 /home/uruc/crabs/run_crabs.py")
        self.current_heading_deg = 0.0
        self.telemetry = {
            "depth_m": 0.0,
            "yaw_deg": 0.0,
            "pitch_deg": 0.0,
            "roll_deg": 0.0,
            "battery_v": 13.3,
            "battery_a": 0.0,
            "state": "NO_TELEMETRY",
        }
        self.telemetry_receiver = TelemetryReceiver(TELEMETRY_PORT)
        self.telemetry_payload = {}
        self.telemetry_online = False
        self.last_telemetry_time = 0.0
        self.camera_sources = [
            os.getenv("ROV_CAMERA_1_URL", "0"),
            os.getenv("ROV_CAMERA_2_URL", "1"),
        ]
        self.camera_streams = [None, None]
        self.camera_images = {}
        self.onboard_host_var = tk.StringVar(value=self.args.onboard_host)
        self.onboard_user_var = tk.StringVar(value=self.args.onboard_user)
        self.onboard_password_var = tk.StringVar(value=self.args.onboard_password)
        self.telemetry_status_var = tk.StringVar(value="Telemetry: waiting")
        self.launch_progress_var = tk.StringVar(value="")

        self.protocol("WM_DELETE_WINDOW", self._on_close)

        try:
            self.mode_bridge.write("Stabilization")
        except Exception:
            pass

        self._apply_dark_theme()
        self._build_ui()
        self._refresh_ui_loop()

    def _on_close(self):
        try:
            self.controller.stop_all()
        except Exception:
            pass
        try:
            self.telemetry_receiver.close()
        except Exception:
            pass
        for stream in self.camera_streams:
            if stream is not None:
                release = getattr(stream, "release", None)
                if callable(release):
                    release()
        self.destroy()

    def _apply_dark_theme(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        self.configure(bg="#020617")

        default_font = tkfont.nametofont("TkDefaultFont")
        default_font.configure(family="Segoe UI", size=10)
        text_font = tkfont.nametofont("TkTextFont")
        text_font.configure(family="Segoe UI", size=10)
        fixed_font = tkfont.nametofont("TkFixedFont")
        fixed_font.configure(family="Consolas", size=10)

        self.option_add("*Background", "#020617")
        self.option_add("*Foreground", "#f8fafc")
        self.option_add("*selectBackground", "#1d4ed8")
        self.option_add("*selectForeground", "#f8fafc")

        style.configure(".", background="#020617", foreground="#f8fafc", fieldbackground="#0f172a")
        style.configure("TFrame", background="#020617")
        style.configure("TLabelframe", background="#020617")
        style.configure("TLabelframe.Label", background="#020617", foreground="#f8fafc")
        style.configure("TLabel", background="#020617", foreground="#f8fafc")
        style.configure("TButton", background="#0f172a", foreground="#f8fafc", padding=(8, 6))
        style.map(
            "TButton",
            background=[("active", "#334155"), ("pressed", "#0f766e"), ("!disabled", "#0f172a")],
            foreground=[("disabled", "#64748b"), ("!disabled", "#f8fafc")],
        )
        style.configure("TEntry", fieldbackground="#0f172a", foreground="#f8fafc", background="#0f172a")
        style.configure("TRadiobutton", background="#020617", foreground="#f8fafc")
        style.configure("TCheckbutton", background="#020617", foreground="#f8fafc")
        style.configure("Card.TLabelframe", background="#0f172a", relief="flat")
        style.configure("Card.TLabelframe.Label", background="#0f172a", foreground="#94a3b8", font=("Segoe UI", 10, "bold"))
        style.configure("Hero.TLabel", background="#020617", foreground="#f8fafc", font=("Segoe UI", 26, "bold"))
        style.configure("Subtle.TLabel", background="#020617", foreground="#94a3b8", font=("Segoe UI", 10))
        style.configure("StatusOk.TLabel", background="#0f172a", foreground="#34d399", font=("Segoe UI", 10, "bold"))
        style.configure("StatusWarn.TLabel", background="#0f172a", foreground="#fbbf24", font=("Segoe UI", 10, "bold"))
        style.configure("StatusBad.TLabel", background="#0f172a", foreground="#f87171", font=("Segoe UI", 10, "bold"))
        style.configure("Primary.TButton", background="#1d4ed8", foreground="#f8fafc", padding=(12, 8))
        style.map(
            "Primary.TButton",
            background=[("active", "#2563eb"), ("pressed", "#1e40af"), ("!disabled", "#1d4ed8")],
            foreground=[("!disabled", "#f8fafc")],
        )
        style.configure("Success.TButton", background="#0f766e", foreground="#f8fafc", padding=(10, 6))
        style.map(
            "Success.TButton",
            background=[("active", "#14b8a6"), ("pressed", "#0d9488"), ("!disabled", "#0f766e")],
        )
        style.configure("Danger.TButton", background="#b91c1c", foreground="#f8fafc", padding=(10, 6))
        style.map(
            "Danger.TButton",
            background=[("active", "#dc2626"), ("pressed", "#991b1b"), ("!disabled", "#b91c1c")],
        )
        style.configure("Accent.TButton", background="#334155", foreground="#f8fafc", padding=(10, 6))
        style.map(
            "Accent.TButton",
            background=[("active", "#475569"), ("pressed", "#1e293b"), ("!disabled", "#334155")],
        )

    def _build_ui(self):
        self.launch_frame = ttk.Frame(self)
        self.launch_frame.pack(fill="both", expand=True)
        self.launch_frame.columnconfigure(0, weight=1)

        hero = ttk.Label(self.launch_frame, text="ROV Launch & Control", style="Hero.TLabel")
        hero.pack(pady=(8, 4))

        intro = ttk.Label(
            self.launch_frame,
            text=(
                "Start the topside sender programs and the onboard ROV control stack, "
                "then continue to the live control dashboard."
            ),
            style="Subtle.TLabel",
            wraplength=920,
            justify="center",
        )
        intro.pack(pady=(0, 14))

        host_frame = ttk.LabelFrame(self.launch_frame, text="SSH target", style="Card.TLabelframe")
        host_frame.pack(fill="x", padx=24, pady=(0, 12))
        host_frame.columnconfigure(1, weight=1)

        ttk.Label(host_frame, text="IP / hostname").grid(row=0, column=0, padx=14, pady=8, sticky="w")
        ttk.Entry(host_frame, textvariable=self.onboard_host_var).grid(row=0, column=1, padx=14, pady=8, sticky="ew")
        ttk.Label(host_frame, text="Username").grid(row=1, column=0, padx=14, pady=8, sticky="w")
        ttk.Entry(host_frame, textvariable=self.onboard_user_var).grid(row=1, column=1, padx=14, pady=8, sticky="ew")
        ttk.Label(host_frame, text="Password").grid(row=2, column=0, padx=14, pady=8, sticky="w")
        ttk.Entry(host_frame, textvariable=self.onboard_password_var, show="*").grid(
            row=2, column=1, padx=14, pady=8, sticky="ew"
        )

        button_frame = ttk.Frame(self.launch_frame)
        button_frame.pack(pady=8)

        self.onboard_button = ttk.Button(
            button_frame,
            text="Start onboard program",
            command=self._start_onboard_programs,
            style="Primary.TButton",
            width=26,
        )
        self.onboard_button.grid(row=0, column=0, padx=10, pady=8)

        self.topside_button = ttk.Button(
            button_frame,
            text="Start topside programs",
            command=self._start_topside_programs,
            style="Primary.TButton",
            width=26,
        )
        self.topside_button.grid(row=0, column=1, padx=10, pady=8)

        nav_frame = ttk.Frame(self.launch_frame)
        nav_frame.pack(pady=(10, 6))

        self.next_button = ttk.Button(
            nav_frame,
            text="Next → Live Control",
            command=self._show_control_screen,
            state="disabled",
            style="Success.TButton",
            width=22,
        )
        self.next_button.grid(row=0, column=0, padx=8)

        self.stop_all_button = ttk.Button(
            nav_frame,
            text="Stop all",
            command=self._stop_all,
            style="Danger.TButton",
            width=16,
        )
        self.stop_all_button.grid(row=0, column=1, padx=8)

        self.status_frame = ttk.LabelFrame(self.launch_frame, text="Launch status", style="Card.TLabelframe")
        self.status_frame.pack(fill="x", padx=24, pady=10)

        self.onboard_status_var = tk.StringVar(value="Onboard: not started")
        self.topside_status_var = tk.StringVar(value="Topside: not started")
        self.remote_status_var = tk.StringVar(value="Remote: idle")

        ttk.Label(self.status_frame, textvariable=self.onboard_status_var).pack(anchor="w", padx=14, pady=4)
        ttk.Label(self.status_frame, textvariable=self.topside_status_var).pack(anchor="w", padx=14, pady=4)
        ttk.Label(self.status_frame, textvariable=self.remote_status_var).pack(anchor="w", padx=14, pady=4)
        ttk.Label(self.status_frame, textvariable=self.launch_progress_var, style="Subtle.TLabel").pack(
            anchor="w", padx=14, pady=(2, 10)
        )

        self.control_frame = ttk.Frame(self)
        self.control_frame.pack(fill="both", expand=True)
        self.control_frame.pack_forget()
        self.control_frame.rowconfigure(1, weight=1)
        self.control_frame.columnconfigure(0, weight=1)

        self._build_control_screen()

    def _build_control_screen(self):
        title_bar = ttk.Frame(self.control_frame)
        title_bar.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        title_bar.columnconfigure(0, weight=1)

        ttk.Label(title_bar, text="ROV Live Control", font=("Segoe UI", 22, "bold")).grid(row=0, column=0, sticky="w")
        ttk.Label(title_bar, textvariable=self.telemetry_status_var, style="Subtle.TLabel").grid(
            row=1, column=0, sticky="w", pady=(2, 0)
        )
        ttk.Button(title_bar, text="← Back to launch", command=self._show_launch_screen, style="Accent.TButton").grid(
            row=0, column=1, rowspan=2, padx=(12, 0), sticky="e"
        )

        content = ttk.Frame(self.control_frame)
        content.grid(row=1, column=0, sticky="nsew")
        content.rowconfigure(1, weight=1)
        content.columnconfigure(0, weight=3)
        content.columnconfigure(1, weight=2)

        cameras = ttk.Frame(content)
        cameras.grid(row=0, column=0, columnspan=2, sticky="nsew", pady=(0, 10))
        cameras.columnconfigure(0, weight=1)
        cameras.columnconfigure(1, weight=1)
        cameras.rowconfigure(0, weight=1)

        self.camera_left = ttk.LabelFrame(cameras, text="Camera 1", style="Card.TLabelframe")
        self.camera_left.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self.camera_left.rowconfigure(0, weight=1)
        self.camera_left.columnconfigure(0, weight=1)
        self.camera_left_canvas = tk.Canvas(self.camera_left, bg="#101820", highlightthickness=0)
        self.camera_left_canvas.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)

        self.camera_right = ttk.LabelFrame(cameras, text="Camera 2", style="Card.TLabelframe")
        self.camera_right.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        self.camera_right.rowconfigure(0, weight=1)
        self.camera_right.columnconfigure(0, weight=1)
        self.camera_right_canvas = tk.Canvas(self.camera_right, bg="#101820", highlightthickness=0)
        self.camera_right_canvas.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)

        self.direction_frame = ttk.LabelFrame(content, text="Direction", style="Card.TLabelframe")
        self.direction_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 6), pady=(0, 10))
        self.direction_frame.rowconfigure(0, weight=1)
        self.direction_frame.columnconfigure(0, weight=1)
        self.direction_canvas = tk.Canvas(self.direction_frame, bg="#0f1720", highlightthickness=0)
        self.direction_canvas.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)

        self.telemetry_frame = ttk.LabelFrame(content, text="Telemetry", style="Card.TLabelframe")
        self.telemetry_frame.grid(row=1, column=1, sticky="nsew", padx=(6, 0), pady=(0, 10))
        self.telemetry_text = tk.StringVar(value="")
        ttk.Label(
            self.telemetry_frame,
            textvariable=self.telemetry_text,
            justify="left",
            font=("Consolas", 10),
            wraplength=420,
        ).pack(anchor="nw", padx=12, pady=12)

        controls = ttk.Frame(content)
        controls.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        controls.columnconfigure(3, weight=1)

        self.mosfet_button = ttk.Button(
            controls,
            text="MOSFET OFF",
            command=self._toggle_mosfet,
            style="Accent.TButton",
            width=22,
        )
        self.mosfet_button.grid(row=0, column=0, padx=(0, 10), sticky="w")
        self._update_mosfet_button()

        self.mode_var = tk.StringVar(value=self.control_mode)
        mode_frame = ttk.LabelFrame(controls, text="Mode", style="Card.TLabelframe")
        mode_frame.grid(row=0, column=1, padx=(0, 10), sticky="w")
        for idx, mode in enumerate(("Stabilization", "Drive/Armed", "Disarmed")):
            ttk.Radiobutton(mode_frame, text=mode, variable=self.mode_var, value=mode).grid(
                row=idx, column=0, sticky="w", padx=8, pady=2
            )
        self.mode_var.trace_add("write", lambda *_: self._set_mode(self.mode_var.get()))

        action_frame = ttk.LabelFrame(controls, text="Mission actions", style="Card.TLabelframe")
        action_frame.grid(row=0, column=2, sticky="w")
        self.colmap_button = ttk.Button(
            action_frame,
            text="Start Colmap",
            command=self._start_colmap,
            style="Accent.TButton",
            width=16,
        )
        self.colmap_button.pack(anchor="w", padx=8, pady=3)
        self.crabs_button = ttk.Button(
            action_frame,
            text="Start Crabs",
            command=self._start_crabs,
            style="Accent.TButton",
            width=16,
        )
        self.crabs_button.pack(anchor="w", padx=8, pady=3)

        bottom = ttk.LabelFrame(self.control_frame, text="System status", style="Card.TLabelframe")
        bottom.grid(row=2, column=0, sticky="ew")
        self.bottom_text = tk.StringVar(value="")
        ttk.Label(bottom, textvariable=self.bottom_text, justify="left", font=("Consolas", 10)).pack(
            anchor="w", padx=12, pady=10
        )

    def _show_launch_screen(self):
        self.launch_frame.pack(fill="both", expand=True)
        self.control_frame.pack_forget()

    def _show_control_screen(self):
        if not self.topside_started or not self.onboard_started:
            messagebox.showwarning("Not ready", "Start both the onboard and topside programs first.")
            return
        self.launch_frame.pack_forget()
        self.control_frame.pack(fill="both", expand=True)

    def _start_onboard_programs(self):
        if getattr(self, "_onboard_launch_in_progress", False):
            return

        host = self.onboard_host_var.get().strip()
        user = self.onboard_user_var.get().strip()
        password = self.onboard_password_var.get().strip()
        self.args.onboard_host = host or self.args.onboard_host
        self.args.onboard_user = user or self.args.onboard_user
        self.args.onboard_password = password or self.args.onboard_password
        self.controller.args.onboard_host = self.args.onboard_host
        self.controller.args.onboard_user = self.args.onboard_user
        self.controller.args.onboard_password = self.args.onboard_password

        self._onboard_launch_in_progress = True
        self.onboard_button.config(state="disabled")
        self.onboard_status_var.set("Onboard: launching over SSH...")
        self.launch_progress_var.set("Connecting to Pi and starting stabilization.py + new_ar.py")

        def worker():
            try:
                result = self.controller.launch_onboard_programs()
                self.after(0, lambda: self._on_onboard_launch_success(result))
            except Exception as exc:
                self.after(0, lambda: self._on_onboard_launch_failure(exc))

        threading.Thread(target=worker, daemon=True).start()

    def _on_onboard_launch_success(self, result):
        self._onboard_launch_in_progress = False
        self.onboard_button.config(state="normal")
        self.onboard_started = True
        self.onboard_status_var.set(
            f"Onboard: running on {result['host']} ({result['user']})"
        )
        self.remote_status_var.set(
            f"Remote root: {result['remote_root']}"
        )
        self.launch_progress_var.set("Onboard stack ready. Start topside programs if you have not already.")
        self.mosfet_enabled = False
        self._update_mosfet_button()
        self._refresh_next_state()

    def _on_onboard_launch_failure(self, exc):
        self._onboard_launch_in_progress = False
        self.onboard_button.config(state="normal")
        self.onboard_started = False
        self.onboard_status_var.set(f"Onboard: error — {exc}")
        self.launch_progress_var.set("")
        messagebox.showerror("Onboard launch failed", str(exc))
        self._refresh_next_state()

    def _start_topside_programs(self):
        try:
            self.mode_bridge.write(self.control_mode)
            result = self.controller.launch_topside_programs()
            self.topside_started = True
            self.topside_status_var.set("Topside: arm_sender.py + thrust_sender.py running")
            self.launch_progress_var.set(
                "Topside senders active. Use Next when both stacks are running."
            )
            self._refresh_next_state()
        except Exception as exc:
            self.topside_started = False
            self.topside_status_var.set(f"Topside: error — {exc}")
            self.launch_progress_var.set("")
            messagebox.showerror("Topside launch failed", str(exc))

    def _refresh_next_state(self):
        if self.topside_started and self.onboard_started:
            self.next_button.config(state="normal")
        else:
            self.next_button.config(state="disabled")

    def _stop_all(self):
        try:
            self.controller.stop_all()
        except Exception as exc:
            messagebox.showwarning("Stop all warning", f"Some stop steps failed: {exc}")

        self.onboard_started = False
        self.topside_started = False
        self.mosfet_enabled = False
        self.onboard_status_var.set("Onboard: not started")
        self.topside_status_var.set("Topside: not started")
        self.remote_status_var.set("Remote: idle")
        self.launch_progress_var.set("")
        self._update_mosfet_button()
        self._refresh_next_state()
        self._show_launch_screen()

    def _update_mosfet_button(self):
        if getattr(self, "mosfet_button", None) is None:
            return
        self.mosfet_button.config(
            text=f"MOSFET {'ON' if self.mosfet_enabled else 'OFF'}",
            style="Success.TButton" if self.mosfet_enabled else "Accent.TButton",
        )

    def _toggle_mosfet(self):
        if not self.onboard_started:
            messagebox.showwarning("Not ready", "Start the onboard programs first.")
            return

        target_enabled = not self.mosfet_enabled
        try:
            self.controller.set_mosfet_state(target_enabled)
            self.mosfet_enabled = target_enabled
        except Exception as exc:
            messagebox.showwarning("MOSFET toggle failed", f"Unable to change MOSFET state: {exc}")
            return

        self._update_mosfet_button()
        self.telemetry["state"] = "MOSFET ON" if self.mosfet_enabled else "OK"

    def _set_mode(self, mode):
        self.control_mode = mode
        try:
            self.mode_bridge.write(mode)
        except Exception as exc:
            messagebox.showwarning("Mode update failed", f"Could not write UI mode file: {exc}")
        if not self.colmap_running and not self.crabs_running:
            self.telemetry["state"] = mode

    def _toggle_mission(self, mission_name, running_attr, button_attr, start_cmd, stop_pattern):
        running = getattr(self, running_attr)
        button = getattr(self, button_attr, None)

        if not self.onboard_started:
            messagebox.showwarning("Not ready", "Start the onboard programs first.")
            return

        if running:
            def worker():
                try:
                    self.controller.stop_remote_mission(mission_name, stop_pattern)
                    self.after(0, lambda: self._on_mission_stopped(mission_name, running_attr, button_attr))
                except Exception as exc:
                    self.after(0, lambda: messagebox.showwarning(f"{mission_name} stop failed", str(exc)))

            if button is not None:
                button.config(state="disabled")
            threading.Thread(target=worker, daemon=True).start()
            return

        def worker():
            try:
                log_path = f"/tmp/uru_{mission_name.lower()}.log"
                self.controller.launch_remote_mission(mission_name, start_cmd, log_path)
                self.after(0, lambda: self._on_mission_started(mission_name, running_attr, button_attr))
            except Exception as exc:
                self.after(0, lambda: self._on_mission_failed(mission_name, button_attr, exc))

        if button is not None:
            button.config(state="disabled")
        threading.Thread(target=worker, daemon=True).start()

    def _on_mission_started(self, mission_name, running_attr, button_attr):
        setattr(self, running_attr, True)
        button = getattr(self, button_attr, None)
        if button is not None:
            button.config(state="normal", text=f"Stop {mission_name}")
        self.telemetry["state"] = f"{mission_name.upper()} RUNNING"

    def _on_mission_stopped(self, mission_name, running_attr, button_attr):
        setattr(self, running_attr, False)
        button = getattr(self, button_attr, None)
        if button is not None:
            button.config(state="normal", text=f"Start {mission_name}")
        self.telemetry["state"] = self.control_mode

    def _on_mission_failed(self, mission_name, button_attr, exc):
        button = getattr(self, button_attr, None)
        if button is not None:
            button.config(state="normal")
        messagebox.showwarning(f"{mission_name} launch failed", str(exc))

    def _start_colmap(self):
        self._toggle_mission("Colmap", "colmap_running", "colmap_button", self.colmap_cmd, "colmap")

    def _start_crabs(self):
        self._toggle_mission("Crabs", "crabs_running", "crabs_button", self.crabs_cmd, "run_crabs.py")

    def _refresh_ui_loop(self, *args):
        self._update_control_panels()
        self.after(UI_REFRESH_MS, self._refresh_ui_loop, None)

    def _as_float(self, value):
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    def _read_telemetry(self):
        payload = self.telemetry_receiver.poll()
        if payload is None:
            self.telemetry_online = (
                self.last_telemetry_time > 0.0 and (time.time() - self.last_telemetry_time) < TELEMETRY_TIMEOUT_SEC
            )
            return

        self.telemetry_payload = payload
        self.last_telemetry_time = time.time()
        self.telemetry_online = True

    def _update_control_panels(self):
        self._read_telemetry()

        payload = self.telemetry_payload if self.telemetry_online else {}
        if payload:
            if payload.get("state") is not None:
                self.telemetry["state"] = str(payload.get("state"))

            depth_m = payload.get("depth_m")
            if depth_m is not None:
                self.telemetry["depth_m"] = self._as_float(depth_m)

            hold_depth_m = payload.get("hold_depth_m")
            if hold_depth_m is not None:
                self.telemetry["hold_depth_m"] = self._as_float(hold_depth_m)

            yaw_deg = payload.get("yaw_deg")
            if yaw_deg is not None:
                self.telemetry["yaw_deg"] = self._as_float(yaw_deg)
                self.current_heading_deg = self._as_float(yaw_deg)

            pitch_deg = payload.get("pitch_deg")
            if pitch_deg is not None:
                self.telemetry["pitch_deg"] = self._as_float(pitch_deg)

            roll_deg = payload.get("roll_deg")
            if roll_deg is not None:
                self.telemetry["roll_deg"] = self._as_float(roll_deg)

            battery_v = payload.get("battery_v")
            if battery_v is not None:
                self.telemetry["battery_v"] = self._as_float(battery_v)

            battery_a = payload.get("battery_a")
            if battery_a is not None:
                self.telemetry["battery_a"] = self._as_float(battery_a)

        if not self.telemetry_online:
            self.telemetry["state"] = self.telemetry.get("state", "NO_TELEMETRY")

        self.telemetry_status_var.set(
            f"Telemetry: {'ONLINE' if self.telemetry_online else 'WAITING'}  |  "
            f"Mode: {self.control_mode}  |  MOSFET: {'ON' if self.mosfet_enabled else 'OFF'}"
        )

        self._draw_camera_view(self.camera_left_canvas, "Camera 1", 0)
        self._draw_camera_view(self.camera_right_canvas, "Camera 2", 1)
        self._draw_direction_overlay()
        self._update_telemetry_text()
        self._update_bottom_text()

    def _draw_camera_view(self, canvas, title, index):
        canvas.delete("all")
        width = max(1, canvas.winfo_width())
        height = max(1, canvas.winfo_height())
        canvas.create_rectangle(0, 0, width, height, fill="#101820", outline="#334155")

        source = self.camera_sources[index] if index < len(self.camera_sources) else ""
        status_text = f"Source: {source or 'disabled'}"
        canvas.create_text(
            width / 2,
            height / 2 - 20,
            text=title,
            fill="#f8fafc",
            font=("Segoe UI", 18, "bold"),
        )
        canvas.create_text(
            width / 2,
            height / 2 + 12,
            text="Live camera feed placeholder",
            fill="#94a3b8",
            font=("Segoe UI", 12),
        )
        canvas.create_text(
            width / 2,
            height / 2 + 38,
            text=status_text,
            fill="#cbd5e1",
            font=("Segoe UI", 10),
        )

        if HAVE_CV2 and HAVE_PIL and source:
            cap = self.camera_streams[index]
            if cap is None:
                try:
                    cap = cv2.VideoCapture(source)
                    self.camera_streams[index] = cap
                except Exception:
                    cap = None

            if cap is not None:
                try:
                    ok, frame = cap.read()
                    if ok and frame is not None:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        resized = cv2.resize(frame, (width, height))
                        photo = ImageTk.PhotoImage(Image.fromarray(resized))
                        self.camera_images[canvas] = photo
                        canvas.create_image(0, 0, anchor="nw", image=photo)
                        canvas.create_text(
                            width / 2,
                            height - 28,
                            text="Live feed active",
                            fill="#34d399",
                            font=("Segoe UI", 10, "bold"),
                        )
                        return
                except Exception:
                    pass

        canvas.create_line(40, 40, width - 40, height - 40, fill="#1d4ed8", width=2)
        canvas.create_line(40, height - 40, width - 40, 40, fill="#1d4ed8", width=2)

    def _draw_direction_overlay(self):
        canvas = self.direction_canvas
        canvas.delete("all")

        width = max(220, canvas.winfo_width())
        height = max(220, canvas.winfo_height())
        cx = width / 2
        cy = height / 2
        radius = min(width, height) * 0.36

        canvas.create_rectangle(0, 0, width, height, fill="#0f1720", outline="#334155")
        canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, outline="#3b82f6", width=2)

        for bearing, label in ((0, "N"), (90, "E"), (180, "S"), (270, "W")):
            angle_rad = math.radians(bearing - self.current_heading_deg - 90)
            tx = cx + math.cos(angle_rad) * (radius + 18)
            ty = cy + math.sin(angle_rad) * (radius + 18)
            canvas.create_text(tx, ty, text=label, fill="#64748b", font=("Segoe UI", 10, "bold"))

        heading_rad = math.radians(-self.current_heading_deg)
        nx = cx + math.sin(heading_rad) * (radius - 12)
        ny = cy - math.cos(heading_rad) * (radius - 12)
        canvas.create_line(cx, cy, nx, ny, fill="#f59e0b", width=4, arrow=tk.LAST, arrowshape=(14, 16, 6))

        pitch = self.telemetry.get("pitch_deg", 0.0)
        roll = self.telemetry.get("roll_deg", 0.0)
        canvas.create_text(
            cx,
            height - 42,
            text=f"Heading {self.current_heading_deg:.1f}°",
            fill="#f8fafc",
            font=("Segoe UI", 12, "bold"),
        )
        canvas.create_text(
            cx,
            height - 22,
            text=f"Pitch {pitch:+.1f}°   Roll {roll:+.1f}°",
            fill="#94a3b8",
            font=("Segoe UI", 10),
        )

    def _update_telemetry_text(self):
        payload = self.telemetry_payload if self.telemetry_online else {}
        battery_v = payload.get("battery_v")
        battery_a = payload.get("battery_a")
        if battery_v is not None and battery_a is not None:
            battery_line = f"Battery: {self._as_float(battery_v):.2f} V / {self._as_float(battery_a):.1f} A"
        else:
            battery_line = "Battery: awaiting sensor"

        text = (
            f"Link: {'ONLINE' if self.telemetry_online else 'WAITING'}\n"
            f"State: {self.telemetry['state']}\n"
            f"Depth: {self.telemetry['depth_m']:.2f} m\n"
            f"Yaw: {self.telemetry['yaw_deg']:.1f}°\n"
            f"Pitch: {self.telemetry['pitch_deg']:.1f}°\n"
            f"Roll: {self.telemetry['roll_deg']:.1f}°\n"
            f"{battery_line}\n"
            f"Mode: {self.control_mode}\n"
            f"MOSFET: {'ON' if self.mosfet_enabled else 'OFF'}\n"
            f"Colmap: {'RUNNING' if self.colmap_running else 'IDLE'}\n"
            f"Crabs: {'RUNNING' if self.crabs_running else 'IDLE'}"
        )
        if payload:
            if payload.get("hold_depth_m") is not None:
                text += f"\nHold depth: {payload.get('hold_depth_m')}"
            if payload.get("depth_correction") is not None:
                text += f"\nDepth correction: {payload.get('depth_correction')}"
            if payload.get("depth_source") is not None:
                text += f"\nDepth source: {payload.get('depth_source')}"
            if payload.get("pressure_hpa") is not None:
                text += f"\nPressure: {payload.get('pressure_hpa')} hPa"
            if payload.get("gain_percent") is not None:
                text += f"\nGain: {payload.get('gain_percent')}%"
            if payload.get("stabilize") is not None:
                text += f"\nStabilize active: {payload.get('stabilize')}"
        self.telemetry_text.set(text)

    def _update_bottom_text(self):
        arm_state = self.controller.get_process_state("arm_sender")
        thrust_state = self.controller.get_process_state("thrust_sender")
        self.bottom_text.set(
            "Status: "
            f"Onboard={'RUNNING' if self.onboard_started else 'STOPPED'} | "
            f"Topside arm={arm_state} thrust={thrust_state} | "
            f"Mode={self.control_mode} | "
            f"Telemetry={'ONLINE' if self.telemetry_online else 'OFFLINE'} | "
            f"Battery={self.telemetry['battery_v']:.2f} V | "
            f"Current={self.telemetry['battery_a']:.1f} A | "
            f"Depth={self.telemetry['depth_m']:.2f} m"
        )


def parse_args():
    parser = argparse.ArgumentParser(description="Launch the ROV UI and control stack")
    parser.add_argument("--repo-root", default=str(Path(__file__).resolve().parent))
    parser.add_argument("--onboard-host", default=os.getenv("ROV_HOST", "192.168.2.249"))
    parser.add_argument("--onboard-user", default=os.getenv("ROV_USER", "uruc"))
    parser.add_argument("--onboard-password", default=os.getenv("ROV_PASSWORD", "yahboom"))
    parser.add_argument("--onboard-root", default=os.getenv("ROV_ROOT", "/home/uruc/URUCDreadYachet"))
    return parser.parse_args()


def main():
    args = parse_args()
    app = ROVMainApp(args)
    app.mainloop()


if __name__ == "__main__":
    main()

