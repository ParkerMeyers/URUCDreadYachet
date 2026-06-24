#!/usr/bin/env python3
"""Main UI and control launcher for the ROV project.

This file ties together the existing topside and onboard scripts by:
- launching the topside sender programs locally
- launching the onboard control programs remotely over SSH
- exposing the outline-requested UI stages (launch and control)
- providing placeholders for the requested control features

The implementation is intentionally robust and dependency-light:
- Tkinter is used for the GUI so it works on most Python installs
- local process launching uses subprocess
- remote launching uses SSH if available
"""

import argparse
import importlib.util
import json
import os
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path
from tkinter import messagebox
import tkinter as tk
from tkinter import ttk


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


TELEMETRY_PORT = 5006
TELEMETRY_TIMEOUT_SEC = 1.0


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

    def _resolve_executable(self, candidates):
        for name in candidates:
            resolved = shutil.which(name)
            if resolved:
                return resolved

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
            for path in extra_paths:
                if os.path.exists(path):
                    return path

        return None

    def _ensure_ssh_available(self):
        self.ssh_executable = self._resolve_executable(["ssh", "ssh.exe"])
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
            "ConnectTimeout=5",
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

    def launch_topside_programs(self):
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

            process = subprocess.Popen(
                [sys.executable, str(script_path)],
                cwd=str(self.repo_root),
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                env=env,
            )
            self.local_processes[name] = process
            self.local_log_handles[name] = log_handle
            results[name] = {"status": "started", "log": str(log_path)}

        return results

    def launch_onboard_programs(self):
        self._ensure_ssh_available()

        remote_root = self.args.onboard_root
        remote_logs = {
            "stabilization": "/tmp/uru_stabilization.log",
            "new_ar": "/tmp/uru_new_ar.log",
        }

        remote_command = (
            f'cd "{remote_root}" && '
            f'nohup python3 "onboard/stabilization.py" > "{remote_logs["stabilization"]}" 2>&1 & '
            f'nohup python3 "onboard/new_ar.py" > "{remote_logs["new_ar"]}" 2>&1 & '
            'echo onboard-launched'
        )

        command = self._build_ssh_command(remote_command)

        password = getattr(self.args, "onboard_password", None)
        ssh_input = None
        if isinstance(password, str) and password and self._resolve_sshpass() is None:
            ssh_input = f"{password}\n"

        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            input=ssh_input,
            timeout=20,
            env=os.environ.copy(),
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"Could not launch onboard scripts over SSH: {result.stderr.strip() or result.stdout.strip()}"
            )

        self.remote_ready = True
        return {
            "status": "started",
            "host": self.args.onboard_host,
            "user": self.args.onboard_user,
            "remote_root": remote_root,
            "logs": remote_logs,
        }

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
            self._ensure_ssh_available()
            stop_command = (
                'pkill -f "onboard/stabilization.py" || true; '
                'pkill -f "onboard/new_ar.py" || true'
            )
            command = self._build_ssh_command(stop_command)
            subprocess.run(command, capture_output=True, text=True, timeout=15)
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
        self.launch_frame = None
        self.control_frame = None

        self.topside_started = False
        self.onboard_started = False
        self.mosfet_enabled = False
        self.control_mode = "Stabilization"
        self.colmap_running = False
        self.crabs_running = False
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

        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_ui()
        self._refresh_ui_loop()

    def _on_close(self):
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

    def _build_ui(self):
        self.launch_frame = ttk.Frame(self)
        self.launch_frame.pack(fill="both", expand=True)

        title = ttk.Label(
            self.launch_frame,
            text="ROV Launch & Control",
            font=("Segoe UI", 22, "bold"),
        )
        title.pack(pady=(0, 12))

        intro = ttk.Label(
            self.launch_frame,
            text=(
                "Start the topside control programs and the onboard ROV control programs, "
                "then move to the live control screen."
            ),
            wraplength=900,
            justify="center",
        )
        intro.pack(pady=(0, 18))

        host_frame = ttk.LabelFrame(self.launch_frame, text="SSH target")
        host_frame.pack(fill="x", padx=18, pady=(0, 10))
        ttk.Label(host_frame, text="IP/hostname to SSH into:").grid(
            row=0, column=0, padx=12, pady=8, sticky="w"
        )
        ttk.Entry(host_frame, textvariable=self.onboard_host_var, width=28).grid(
            row=0, column=1, padx=12, pady=8, sticky="ew"
        )
        ttk.Label(host_frame, text="Username:").grid(
            row=1, column=0, padx=12, pady=8, sticky="w"
        )
        ttk.Entry(host_frame, textvariable=self.onboard_user_var, width=28).grid(
            row=1, column=1, padx=12, pady=8, sticky="ew"
        )
        ttk.Label(host_frame, text="Password:").grid(
            row=2, column=0, padx=12, pady=8, sticky="w"
        )
        ttk.Entry(host_frame, textvariable=self.onboard_password_var, width=28, show="*").grid(
            row=2, column=1, padx=12, pady=8, sticky="ew"
        )
        host_frame.columnconfigure(1, weight=1)

        button_frame = ttk.Frame(self.launch_frame)
        button_frame.pack(pady=6)

        self.onboard_button = ttk.Button(
            button_frame,
            text="Start onboard program",
            command=self._start_onboard_programs,
            width=24,
        )
        self.onboard_button.grid(row=0, column=0, padx=12, pady=8)

        self.topside_button = ttk.Button(
            button_frame,
            text="Start art topside program",
            command=self._start_topside_programs,
            width=24,
        )
        self.topside_button.grid(row=0, column=1, padx=12, pady=8)

        self.next_button = ttk.Button(
            self.launch_frame,
            text="Next",
            command=self._show_control_screen,
            state="disabled",
            width=18,
        )
        self.next_button.pack(pady=(16, 8))

        self.stop_all_button = ttk.Button(
            self.launch_frame,
            text="Stop all",
            command=self._stop_all,
            width=18,
        )
        self.stop_all_button.pack(pady=(0, 20))

        self.status_frame = ttk.LabelFrame(self.launch_frame, text="Launch status")
        self.status_frame.pack(fill="x", padx=18, pady=8)

        self.onboard_status_var = tk.StringVar(value="Not started")
        self.topside_status_var = tk.StringVar(value="Not started")
        self.remote_status_var = tk.StringVar(value="No remote launch")

        ttk.Label(self.status_frame, textvariable=self.onboard_status_var).grid(
            row=0, column=0, padx=12, pady=8, sticky="w"
        )
        ttk.Label(self.status_frame, textvariable=self.topside_status_var).grid(
            row=1, column=0, padx=12, pady=8, sticky="w"
        )
        ttk.Label(self.status_frame, textvariable=self.remote_status_var).grid(
            row=2, column=0, padx=12, pady=8, sticky="w"
        )

        self.control_frame = ttk.Frame(self)
        self.control_frame.pack(fill="both", expand=True)
        self.control_frame.pack_forget()

        self._build_control_screen()

    def _build_control_screen(self):
        title_bar = ttk.Frame(self.control_frame)
        title_bar.pack(fill="x", pady=(0, 10))

        ttk.Label(
            title_bar,
            text="ROV Live Control",
            font=("Segoe UI", 20, "bold"),
        ).pack(side="left")

        ttk.Button(title_bar, text="Back", command=self._show_launch_screen).pack(side="right")

        content = ttk.Frame(self.control_frame)
        content.pack(fill="both", expand=True)

        top_row = ttk.Frame(content)
        top_row.pack(fill="x", pady=(0, 12))

        self.camera_left = ttk.LabelFrame(top_row, text="Camera 1")
        self.camera_left.pack(side="left", fill="both", expand=True, padx=(0, 8))
        self.camera_left_canvas = tk.Canvas(self.camera_left, width=480, height=270, bg="#101820")
        self.camera_left_canvas.pack(fill="both", expand=True)

        self.camera_right = ttk.LabelFrame(top_row, text="Camera 2")
        self.camera_right.pack(side="right", fill="both", expand=True, padx=(8, 0))
        self.camera_right_canvas = tk.Canvas(self.camera_right, width=480, height=270, bg="#101820")
        self.camera_right_canvas.pack(fill="both", expand=True)

        middle_row = ttk.Frame(content)
        middle_row.pack(fill="x", pady=(0, 12))

        self.direction_frame = ttk.LabelFrame(middle_row, text="Direction overlay")
        self.direction_frame.pack(side="left", fill="both", expand=True, padx=(0, 8))
        self.direction_canvas = tk.Canvas(self.direction_frame, width=360, height=240, bg="#0f1720")
        self.direction_canvas.pack(fill="both", expand=True)

        self.telemetry_frame = ttk.LabelFrame(middle_row, text="Telemetry overlay")
        self.telemetry_frame.pack(side="right", fill="both", expand=True, padx=(8, 0))
        self.telemetry_text = tk.StringVar(value="")
        ttk.Label(
            self.telemetry_frame,
            textvariable=self.telemetry_text,
            justify="left",
            font=("Consolas", 11),
            wraplength=420,
        ).pack(anchor="w", padx=10, pady=10)

        controls = ttk.Frame(content)
        controls.pack(fill="x", pady=(0, 12))

        self.mosfet_button = ttk.Button(
            controls,
            text="Toggle MOSFET (OFF)",
            command=self._toggle_mosfet,
            width=24,
        )
        self.mosfet_button.pack(side="left", padx=(0, 8))

        self.mode_var = tk.StringVar(value=self.control_mode)
        mode_frame = ttk.LabelFrame(controls, text="Mode")
        mode_frame.pack(side="left", padx=(0, 8))
        ttk.Radiobutton(mode_frame, text="Stabilization", variable=self.mode_var, value="Stabilization").pack(anchor="w")
        ttk.Radiobutton(mode_frame, text="Drive/Armed", variable=self.mode_var, value="Drive/Armed").pack(anchor="w")
        ttk.Radiobutton(mode_frame, text="Disarmed", variable=self.mode_var, value="Disarmed").pack(anchor="w")
        self.mode_var.trace_add("write", lambda *_: self._set_mode(self.mode_var.get()))

        action_frame = ttk.LabelFrame(controls, text="Mission actions")
        action_frame.pack(side="left", padx=(0, 8))
        ttk.Button(action_frame, text="Start Colmap", command=self._start_colmap).pack(anchor="w", padx=6, pady=2)
        ttk.Button(action_frame, text="Start Crabs", command=self._start_crabs).pack(anchor="w", padx=6, pady=2)

        bottom = ttk.LabelFrame(content, text="System info")
        bottom.pack(fill="x")
        self.bottom_text = tk.StringVar(value="")
        ttk.Label(bottom, textvariable=self.bottom_text, justify="left", font=("Consolas", 11)).pack(anchor="w", padx=10, pady=10)

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
        try:
            host = self.onboard_host_var.get().strip()
            user = self.onboard_user_var.get().strip()
            password = self.onboard_password_var.get().strip()
            self.args.onboard_host = host or self.args.onboard_host
            self.args.onboard_user = user or self.args.onboard_user
            self.args.onboard_password = password or self.args.onboard_password
            self.controller.args.onboard_host = self.args.onboard_host
            self.controller.args.onboard_user = self.args.onboard_user
            self.controller.args.onboard_password = self.args.onboard_password
            result = self.controller.launch_onboard_programs()
            self.onboard_started = True
            self.onboard_status_var.set(
                f"Onboard launched on {result['host']} ({result['user']})"
            )
            self.remote_status_var.set(
                f"Remote root: {result['remote_root']}"
            )
            self._refresh_next_state()
        except Exception as exc:
            self.onboard_started = False
            self.onboard_status_var.set(f"Error: {exc}")
            messagebox.showerror("Onboard launch failed", str(exc))

    def _start_topside_programs(self):
        try:
            result = self.controller.launch_topside_programs()
            self.topside_started = True
            self.topside_status_var.set(
                "Topside launched: arm_sender.py + thrust_sender.py"
            )
            for name, info in result.items():
                self.topside_status_var.set(
                    f"Topside launched: {name} -> {info['log']}"
                )
            self._refresh_next_state()
        except Exception as exc:
            self.topside_started = False
            self.topside_status_var.set(f"Error: {exc}")
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
            messagebox.showerror("Stop failed", str(exc))
        self.topside_started = False
        self.onboard_started = False
        self.onboard_status_var.set("Not started")
        self.topside_status_var.set("Not started")
        self.remote_status_var.set("No remote launch")
        self._refresh_next_state()

    def _toggle_mosfet(self):
        self.mosfet_enabled = not self.mosfet_enabled
        self.mosfet_button.config(text=f"Toggle MOSFET ({'ON' if self.mosfet_enabled else 'OFF'})")
        self.telemetry["state"] = "MOSFET ON" if self.mosfet_enabled else "OK"

    def _set_mode(self, mode):
        self.control_mode = mode
        self.telemetry["state"] = mode

    def _start_colmap(self):
        self.colmap_running = not self.colmap_running
        self.telemetry["state"] = "COLMAP RUNNING" if self.colmap_running else self.control_mode

    def _start_crabs(self):
        self.crabs_running = not self.crabs_running
        self.telemetry["state"] = "CRABS RUNNING" if self.crabs_running else self.control_mode

    def _refresh_ui_loop(self, *args):
        self._update_control_panels()
        self.after(250, self._refresh_ui_loop, None)

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
        self.direction_canvas.delete("all")
        self.direction_canvas.create_rectangle(0, 0, 360, 240, fill="#0f1720", outline="#334155")
        self.direction_canvas.create_text(180, 25, text="Direction overlay", fill="#f8fafc", font=("Segoe UI", 14, "bold"))
        self.direction_canvas.create_oval(120, 70, 240, 190, outline="#3b82f6", width=2)
        self.direction_canvas.create_line(180, 130, 180, 80, fill="#f59e0b", width=3)
        self.direction_canvas.create_line(180, 130, 230, 105, fill="#f59e0b", width=3)
        self.direction_canvas.create_line(180, 130, 130, 105, fill="#f59e0b", width=3)
        angle = self.current_heading_deg
        self.direction_canvas.create_text(180, 220, text=f"Heading {angle:.1f}°", fill="#f8fafc", font=("Segoe UI", 12, "bold"))

    def _update_telemetry_text(self):
        payload = self.telemetry_payload if self.telemetry_online else {}
        text = (
            f"Telemetry: {'ONLINE' if self.telemetry_online else 'WAITING'}\n"
            f"State: {self.telemetry['state']}\n"
            f"Depth: {self.telemetry['depth_m']:.2f} m\n"
            f"Yaw: {self.telemetry['yaw_deg']:.1f}°\n"
            f"Pitch: {self.telemetry['pitch_deg']:.1f}°\n"
            f"Roll: {self.telemetry['roll_deg']:.1f}°\n"
            f"Battery: {self.telemetry['battery_v']:.2f} V / {self.telemetry['battery_a']:.1f} A\n"
            f"Mode: {self.control_mode}\n"
            f"MOSFET: {'ON' if self.mosfet_enabled else 'OFF'}\n"
            f"Colmap: {'RUNNING' if self.colmap_running else 'IDLE'}\n"
            f"Crabs: {'RUNNING' if self.crabs_running else 'IDLE'}"
        )
        if payload:
            text += f"\nHold depth: {payload.get('hold_depth_m', 'n/a')}"
            text += f"\nDepth correction: {payload.get('depth_correction', 'n/a')}"
            text += f"\nDepth source: {payload.get('depth_source', 'n/a')}"
        self.telemetry_text.set(text)

    def _update_bottom_text(self):
        self.bottom_text.set(
            "Status: "
            f"Onboard={'RUNNING' if self.onboard_started else 'STOPPED'} | "
            f"Topside={'RUNNING' if self.topside_started else 'STOPPED'} | "
            f"Mode={self.control_mode} | "
            f"Battery={self.telemetry['battery_v']:.2f} V | "
            f"Current={self.telemetry['battery_a']:.1f} A"
        )


def parse_args():
    parser = argparse.ArgumentParser(description="Launch the ROV UI and control stack")
    parser.add_argument("--repo-root", default=str(Path(__file__).resolve().parent))
    parser.add_argument("--onboard-host", default=os.getenv("ROV_HOST", "10.42.0.181"))
    parser.add_argument("--onboard-user", default=os.getenv("ROV_USER", "uruc"))
    parser.add_argument("--onboard-password", default=os.getenv("ROV_PASSWORD", "yahboom"))
    parser.add_argument("--onboard-root", default=os.getenv("ROV_ROOT", "/home/pi/URUCDreadYachet"))
    return parser.parse_args()


def main():
    args = parse_args()
    app = ROVMainApp(args)
    app.mainloop()


if __name__ == "__main__":
    main()

