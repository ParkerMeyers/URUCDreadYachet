#!/usr/bin/env python3
"""
Onboard process supervisor — reliable start / stop / wait / status on the Pi.

Uses the same launch style as manual testing:
  cd ~/URUCDreadYachet && python3 onboard/stabilization.py

Processes are placed in their own session (setsid) so they survive SSH disconnect.
Status uses PID files + log readiness markers instead of fragile pgrep over SSH.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

SERVICES: dict[str, dict] = {
    "stab": {
        "script": "onboard/stabilization.py",
        "log": "/tmp/rov_stab.log",
        "pidfile": "/tmp/rov_stab.pid",
        "ready_re": r"(Receiver running\.|Pix6 RC output ready)",
    },
    "arm": {
        "script": "onboard/new_ar.py",
        "log": "/tmp/rov_arm.log",
        "pidfile": "/tmp/rov_arm.pid",
        "ready_re": r"\[arm\] Listening on UDP|\[arm\] rx=",
    },
    "cam": {
        "script": "onboard/camera_stream.py",
        "log": "/tmp/rov_cam.log",
        "pidfile": "/tmp/rov_cam.pid",
        "ready_re": r"\[cam\] Dual-camera MJPEG server running",
    },
}


def _read_pid(pidfile: str) -> int | None:
    try:
        raw = Path(pidfile).read_text(encoding="utf-8").strip()
        return int(raw) if raw.isdigit() else None
    except OSError:
        return None


def _write_pid(pidfile: str, pid: int) -> None:
    Path(pidfile).write_text(str(pid), encoding="utf-8")


def _clear_pid(pidfile: str) -> None:
    try:
        Path(pidfile).unlink()
    except OSError:
        pass


def _pid_alive(pid: int | None) -> bool:
    if not pid:
        return False
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def _log_text(log_path: str) -> str:
    try:
        return Path(log_path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""


def _log_tail(log_path: str, lines: int = 8) -> str:
    text = _log_text(log_path)
    if not text:
        return ""
    return "\n".join(text.splitlines()[-lines:])


def _is_ready(svc: dict) -> bool:
    pattern = svc.get("ready_re")
    if not pattern:
        return True
    return bool(re.search(pattern, _log_text(svc["log"])))


def _pkill_script(script: str) -> None:
    name = Path(script).name
    subprocess.run(
        ["pkill", "-f", f"python3.*{name}"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def _stop_service(svc: dict) -> None:
    pid = _read_pid(svc["pidfile"])
    if pid and _pid_alive(pid):
        try:
            os.killpg(os.getpgid(pid), signal.SIGTERM)
        except OSError:
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                pass
        for _ in range(30):
            if not _pid_alive(pid):
                break
            time.sleep(0.1)
    _pkill_script(svc["script"])
    _clear_pid(svc["pidfile"])


def _release_video_devices_from_args(extra_args: str) -> None:
    """Free V4L2 nodes before launching camera_stream (PipeWire, stale grabbers)."""
    for dev in re.findall(r"/dev/video\d+", extra_args):
        try:
            proc = subprocess.run(
                ["fuser", dev],
                capture_output=True,
                text=True,
                check=False,
            )
            if proc.returncode == 0 and proc.stdout.strip():
                subprocess.run(
                    ["fuser", "-k", dev],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False,
                )
        except OSError:
            pass
    time.sleep(0.5)


def _start_service(svc: dict, extra_args: str = "") -> int:
    _stop_service(svc)
    if svc.get("script", "").endswith("camera_stream.py"):
        _release_video_devices_from_args(extra_args)
    log_path = Path(svc["log"])
    log_path.write_text("", encoding="utf-8")

    cmd = ["python3", svc["script"]]
    if extra_args.strip():
        cmd.extend(extra_args.split())

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"

    with open(log_path, "ab", buffering=0) as logf:
        proc = subprocess.Popen(
            ["python3", "-u", *cmd[1:]],
            cwd=str(ROOT),
            stdin=subprocess.DEVNULL,
            stdout=logf,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=env,
        )

    _write_pid(svc["pidfile"], proc.pid)
    return proc.pid


def _status_service(name: str, svc: dict) -> dict:
    pid = _read_pid(svc["pidfile"])
    alive = _pid_alive(pid)
    ready = alive and _is_ready(svc)
    return {
        "name": name,
        "pid": pid,
        "alive": alive,
        "ready": ready,
        "log_tail": _log_tail(svc["log"]),
    }


def cmd_start(name: str, extra_args: str = "") -> dict:
    svc = SERVICES[name]
    pid = _start_service(svc, extra_args)
    return {"ok": True, "name": name, "pid": pid}


def cmd_stop(name: str) -> dict:
    _stop_service(SERVICES[name])
    return {"ok": True, "name": name}


def cmd_stop_all() -> dict:
    for name in SERVICES:
        _stop_service(SERVICES[name])
    return {"ok": True, "stopped": list(SERVICES.keys())}


def cmd_status() -> dict:
    return {name: _status_service(name, svc) for name, svc in SERVICES.items()}


def cmd_wait(name: str, timeout: float) -> dict:
    svc = SERVICES[name]
    deadline = time.time() + timeout
    saw_alive = False

    while time.time() < deadline:
        st = _status_service(name, svc)
        if st["alive"]:
            saw_alive = True
            if st["ready"]:
                return {"ok": True, **st}
        elif saw_alive:
            return {
                "ok": False,
                "error": "process exited",
                **st,
            }
        time.sleep(1.0)

    st = _status_service(name, svc)
    err = "timeout waiting for ready" if st["alive"] else "process not running"
    return {"ok": False, "error": err, **st}


def main() -> None:
    parser = argparse.ArgumentParser(description="Onboard process supervisor")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_start = sub.add_parser("start")
    p_start.add_argument("name", choices=SERVICES.keys())
    p_start.add_argument("--extra-args", default="")

    p_stop = sub.add_parser("stop")
    p_stop.add_argument("name", choices=[*SERVICES.keys(), "all"])

    p_wait = sub.add_parser("wait")
    p_wait.add_argument("name", choices=SERVICES.keys())
    p_wait.add_argument("--timeout", type=float, default=45.0)

    sub.add_parser("status")

    args = parser.parse_args()

    if args.cmd == "start":
        result = cmd_start(args.name, args.extra_args)
    elif args.cmd == "stop":
        result = cmd_stop_all() if args.name == "all" else cmd_stop(args.name)
    elif args.cmd == "wait":
        result = cmd_wait(args.name, args.timeout)
    else:
        result = cmd_status()

    print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
