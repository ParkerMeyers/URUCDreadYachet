#!/usr/bin/env python3
"""Set up the ROV stack after a reset or fresh clone.

Run on the topside laptop:
    python install.py --topside

Run on the Raspberry Pi (onboard):
    python3 install.py --onboard

Auto-detect (Pi vs laptop):
    python install.py
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent
LOGS_DIR = REPO_ROOT / "logs"
TOPSIDE_REQUIREMENTS = REPO_ROOT / "requirements.txt"
ONBOARD_REQUIREMENTS = REPO_ROOT / "requirements-onboard.txt"
ENV_EXAMPLE = REPO_ROOT / "rov.env.example"

DEFAULT_ONBOARD_VENV = "venv"
DEFAULT_TOPSIDE_VENV = ".venv"


def log(msg):
    print(msg, flush=True)


def run(cmd, **kwargs):
    log(f"  $ {' '.join(str(part) for part in cmd)}")
    subprocess.run(cmd, check=True, **kwargs)


def is_probably_pi():
    machine = platform.machine().lower()
    if machine.startswith(("arm", "aarch64")):
        return True
    return (REPO_ROOT / "onboard" / "stabilization.py").exists() and os.name != "nt"


def venv_python(venv_dir: Path) -> Path:
    if os.name == "nt":
        return venv_dir / "Scripts" / "python.exe"
    return venv_dir / "bin" / "python3"


def ensure_venv(venv_dir: Path, recreate: bool) -> Path:
    py = venv_python(venv_dir)
    if recreate and venv_dir.exists():
        log(f"Removing existing venv: {venv_dir}")
        shutil.rmtree(venv_dir)

    if not py.exists():
        log(f"Creating virtual environment: {venv_dir}")
        run([sys.executable, "-m", "venv", str(venv_dir)])
    else:
        log(f"Using existing virtual environment: {venv_dir}")

    if not py.exists():
        raise RuntimeError(f"venv python not found at {py}")

    run([str(py), "-m", "pip", "install", "--upgrade", "pip"])
    return py


def pip_install(py: Path, requirements: Path):
    if not requirements.exists():
        raise FileNotFoundError(f"Missing requirements file: {requirements}")
    run([str(py), "-m", "pip", "install", "-r", str(requirements)])


def ensure_logs():
    LOGS_DIR.mkdir(exist_ok=True)
    mode_file = LOGS_DIR / "rov_ui_mode.json"
    if not mode_file.exists():
        mode_file.write_text(
            json.dumps({"mode": "Disarmed", "time": 0}, indent=2) + "\n",
            encoding="utf-8",
        )
        log(f"Created {mode_file}")


def write_env_example():
    if ENV_EXAMPLE.exists():
        return
    ENV_EXAMPLE.write_text(
        """# Copy to rov.env and adjust, or set these in your shell / UI fields.
# Topside laptop
ROV_HOST=192.168.2.249
ROV_USER=uruc
ROV_PASSWORD=yahboom
ROV_ROOT=/home/uruc/URUCDreadYachet
ROV_VENV=venv

# Arm serial (Windows: COM3, Linux: /dev/ttyACM0)
ROV_ARM_SERIAL=COM3

# Camera RTP/UDP ports (see topside/ROV_Cameras.sh)
ROV_CAMERA_1_URL=rov-udp:5600
ROV_CAMERA_2_URL=rov-udp:5601

# Optional mission commands on the Pi
# ROV_COLMAP_CMD=colmap automatic_reconstructor --workspace_path /home/uruc/colmap_ws
# ROV_CRABS_CMD=python3 /home/uruc/crabs/run_crabs.py
""",
        encoding="utf-8",
    )
    log(f"Created {ENV_EXAMPLE}")


def chmod_scripts():
    cameras = REPO_ROOT / "topside" / "ROV_Cameras.sh"
    if cameras.exists() and os.name != "nt":
        cameras.chmod(cameras.stat().st_mode | 0o111)
        log(f"Made executable: {cameras}")


def verify_imports(py: Path, modules):
    failed = []
    for mod in modules:
        result = subprocess.run(
            [str(py), "-c", f"import {mod}"],
            capture_output=True,
            text=True,
        )
        if result.returncode == 0:
            log(f"  OK  {mod}")
        else:
            failed.append(mod)
            log(f"  FAIL {mod}")
    return failed


def check_tool(name):
    path = shutil.which(name)
    if path:
        log(f"  OK  {name} ({path})")
        return True
    log(f"  --  {name} not found (optional or install separately)")
    return False


def install_topside(recreate_venv: bool):
    log("\n=== Topside / UI setup ===")
    py = ensure_venv(REPO_ROOT / DEFAULT_TOPSIDE_VENV, recreate_venv)
    pip_install(py, TOPSIDE_REQUIREMENTS)
    ensure_logs()
    write_env_example()
    chmod_scripts()

    log("\nVerifying Python imports:")
    failed = verify_imports(
        py,
        [
            "paramiko",
            "pygame",
            "serial",
            "cv2",
            "PIL",
            "numpy",
            "pynput",
        ],
    )

    log("\nChecking system tools:")
    check_tool("gst-launch-1.0")

    log("\nTopside next steps:")
    log(f"  1. Activate venv: {activate_hint(REPO_ROOT / DEFAULT_TOPSIDE_VENV)}")
    log("  2. Copy rov.env.example to rov.env and set ROV_HOST / serial port")
    log("  3. Install GStreamer if cameras stay blank (see topside/ROV_Cameras.sh)")
    log("  4. Run: python main_control_ui.py")
    log("     Or:  run_ui.bat  (Windows) / ./run_ui.sh  (Linux/macOS)")
    if failed:
        log(f"\nWarning: some imports failed: {', '.join(failed)}")


def install_onboard(recreate_venv: bool):
    log("\n=== Onboard (Raspberry Pi) setup ===")
    if not ONBOARD_REQUIREMENTS.exists():
        raise FileNotFoundError(ONBOARD_REQUIREMENTS)

    py = ensure_venv(REPO_ROOT / DEFAULT_ONBOARD_VENV, recreate_venv)
    pip_install(py, ONBOARD_REQUIREMENTS)
    chmod_scripts()

    log("\nVerifying Python imports:")
    failed = verify_imports(
        py,
        [
            "smbus2",
            "pymavlink",
            "lgpio",
            "board",
            "adafruit_bno055",
            "adafruit_servokit",
        ],
    )

    log("\nOnboard next steps:")
    log(f"  1. venv python: {venv_python(REPO_ROOT / DEFAULT_ONBOARD_VENV)}")
    log("  2. Enable I2C / SSH on the Pi if not already done")
    log("  3. Start stack from the topside UI (uses venv/bin/python3 over SSH)")
    log("  4. Manual test:")
    log(f"     source {DEFAULT_ONBOARD_VENV}/bin/activate")
    log("     python onboard/stabilization.py")
    if failed:
        log(f"\nWarning: some imports failed: {', '.join(failed)}")


def activate_hint(venv_dir: Path) -> str:
    if os.name == "nt":
        return str(venv_dir / "Scripts" / "activate")
    return f"source {venv_dir}/bin/activate"


def print_system_deps(role: str):
    log("\n=== Optional system packages (run manually with sudo) ===")
    if role in ("topside", "both"):
        log("Topside (Linux) camera decode:")
        log("  sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-{base,good,bad,ugly}")
        log("Windows: install GStreamer from https://gstreamer.freedesktop.org/download/")
    if role in ("onboard", "both"):
        log("Onboard Pi:")
        log("  sudo apt install python3-venv python3-dev i2c-tools libgpiod2")
        log("  sudo raspi-config  # enable I2C, SSH")
        log("  sudo usermod -aG i2c,gpio $USER  # then log out/in")


def parse_args():
    parser = argparse.ArgumentParser(description="Install ROV topside and/or onboard dependencies")
    role = parser.add_mutually_exclusive_group()
    role.add_argument("--topside", action="store_true", help="Set up the control laptop / UI")
    role.add_argument("--onboard", action="store_true", help="Set up the Raspberry Pi onboard venv")
    role.add_argument(
        "--both",
        action="store_true",
        help="Run topside and onboard setup (full repo on one machine)",
    )
    parser.add_argument(
        "--recreate-venv",
        action="store_true",
        help="Delete and recreate the target virtual environment(s)",
    )
    parser.add_argument(
        "--system-deps",
        action="store_true",
        help="Print suggested apt/system packages (does not run sudo)",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    if args.both:
        selected = "both"
    elif args.topside:
        selected = "topside"
    elif args.onboard:
        selected = "onboard"
    elif is_probably_pi():
        selected = "onboard"
    else:
        selected = "topside"

    log(f"ROV install | repo: {REPO_ROOT}")
    log(f"Role: {selected}")

    if args.system_deps:
        print_system_deps(selected)

    if selected in ("topside", "both"):
        install_topside(args.recreate_venv)
    if selected in ("onboard", "both"):
        install_onboard(args.recreate_venv)

    if not args.system_deps:
        print_system_deps(selected)

    log("\nDone.")


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as exc:
        log(f"\nInstall failed (exit {exc.returncode}).")
        sys.exit(exc.returncode)
    except Exception as exc:
        log(f"\nInstall failed: {exc}")
        sys.exit(1)
