#!/usr/bin/env python3
"""Shared UDP/TCP port map — single reference for debugging comms."""
from __future__ import annotations

# Topside → Pi thrust commands (JSON)
UDP_THRUST = 5005

# Pi arm CSV from arm_sender + thrust telemetry to topside (different hosts)
UDP_ARM_CSV = 5006
UDP_TELEMETRY = 5006

# Pi arm JSON control (web UI presets, manual AUX)
UDP_ARM_CONTROL = 5009

# Pi → topside arm BNO055 telemetry
UDP_ARM_TELEMETRY = 5008

# MAVProxy tcpin — one TCP client per port
MAVPROXY_TCP_STAB = 5762
MAVPROXY_TCP_ARM = 5763

MAVPROXY_ONBOARD_STAB = f"tcpin:127.0.0.1:{MAVPROXY_TCP_STAB}"
MAVPROXY_ONBOARD_ARM = f"tcpin:127.0.0.1:{MAVPROXY_TCP_ARM}"

# Camera MJPEG HTTP
HTTP_CAM_ARM = 8160
HTTP_CAM_FORWARD = 8161
