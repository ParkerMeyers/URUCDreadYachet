"""MOSFET servo power rail — UDP to Pi mosfet_service.py."""
from __future__ import annotations

import json
import socket

from topside.config import config, load_config_file


def is_enabled() -> bool:
    load_config_file()
    val = config.get("mosfet_enabled", True)
    if isinstance(val, str):
        return val.strip().lower() in ("1", "true", "yes", "on")
    return bool(val)


def send_command(state: bool) -> tuple[bool, str]:
    if not is_enabled():
        return False, "MOSFET disabled in config"
    load_config_file()
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            payload = json.dumps({"cmd": "mosfet", "state": bool(state)}).encode("utf-8")
            port = int(config["mosfet_control_port"])
            sock.sendto(payload, (config["pi_ip"], port))
        finally:
            sock.close()
        return True, f"sent → {config['pi_ip']}:{port}"
    except Exception as e:
        return False, str(e)
