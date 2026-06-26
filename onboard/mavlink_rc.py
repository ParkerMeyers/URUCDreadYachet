#!/usr/bin/env python3
"""MAVLink helpers for onboard companion scripts."""
from __future__ import annotations

import os
import threading
import time

# Select the v20 dialect before pymavlink loads message definitions.
os.environ.setdefault("MAVLINK20", "1")

RC_IGNORE = 65535

# MAVProxy tcpin accepts ONE TCP client per port — use separate ports:
#   stabilization.py → 5762   new_ar.py → 5763
# See ports.py for the full port map.
try:
    from ports import MAVPROXY_ONBOARD_ARM, MAVPROXY_ONBOARD_STAB
except ImportError:
    from onboard.ports import MAVPROXY_ONBOARD_ARM, MAVPROXY_ONBOARD_STAB

MAVLINK_ONBOARD = MAVPROXY_ONBOARD_STAB.replace("tcpin:", "tcp:")
MAVLINK_ONBOARD_ARM = MAVPROXY_ONBOARD_ARM.replace("tcpin:", "tcp:")


def normalize_mavlink_url(url: str) -> str:
    """
    Normalize legacy udp:HOST:PORT URLs to udpin for MAVProxy UDP outputs.

    Prefer MAVLINK_ONBOARD (tcp) for new deployments — UDP only supports one
    listener per port.
    """
    if url.startswith(("udpin:", "udpout:", "tcp:", "serial:", "/dev/")):
        return url
    if url.startswith("udp:"):
        parts = url.split(":")
        if len(parts) == 3 and parts[2].isdigit():
            return f"udpin:0.0.0.0:{parts[2]}"
    return url


def _targets(master) -> tuple[int, int]:
    return (
        int(getattr(master, "target_system", None) or 1),
        int(getattr(master, "target_component", None) or 1),
    )


def _pad_channels(channels, *, ignore: int = RC_IGNORE) -> list[int]:
    ch = [int(v) for v in channels]
    if len(ch) > 18:
        ch = ch[:18]
    while len(ch) < 18:
        ch.append(ignore)
    return ch


def send_rc_channels_override(master, channels, *, ignore: int = RC_IGNORE) -> None:
    """Send RC_CHANNELS_OVERRIDE for up to 18 channels (MAVLink 2 encoding)."""
    ts, tc = _targets(master)
    ch = _pad_channels(channels, ignore=ignore)
    # Use the connection's mavlink encoder so sequence/CRC match the open link.
    master.mav.rc_channels_override_send(
        ts,
        tc,
        ch[0],
        ch[1],
        ch[2],
        ch[3],
        ch[4],
        ch[5],
        ch[6],
        ch[7],
        ch[8],
        ch[9],
        ch[10],
        ch[11],
        ch[12],
        ch[13],
        ch[14],
        ch[15],
        ch[16],
        ch[17],
    )


def connect_mavlink(url: str | None = None, *, source_system: int = 255, timeout: float = 45.0):
    """Open a MAVLink connection to MAVProxy; retry until TCP/UDP endpoint is up."""
    from pymavlink import mavutil

    url = url or MAVLINK_ONBOARD
    conn_url = normalize_mavlink_url(url)
    if conn_url != url:
        print(f"[mavlink] Using {conn_url} (from {url})")
    print(f"[mavlink] Connecting to {conn_url} ...")

    deadline = time.time() + timeout
    last_err: Exception | None = None
    while time.time() < deadline:
        try:
            master = mavutil.mavlink_connection(conn_url, source_system=source_system)
            try:
                master.mav.set_protocol(mavutil.mavlink.MAVLINK_V2)
            except Exception:
                pass
            return master
        except (ConnectionRefusedError, OSError) as e:
            last_err = e
            if "Connection refused" not in str(e) and not isinstance(e, ConnectionRefusedError):
                raise
            time.sleep(1.0)
        except Exception as e:
            if "Connection refused" in str(e):
                last_err = e
                time.sleep(1.0)
            else:
                raise

    raise ConnectionRefusedError(
        f"MAVLink endpoint {conn_url} not available after {int(timeout)}s: {last_err}"
    )


def wait_for_heartbeat(master, timeout: float = 20.0):
    """Wait for a vehicle HEARTBEAT, pinging as GCS while polling."""
    from pymavlink import mavutil

    stop = threading.Event()

    def _gcs_ping():
        while not stop.is_set():
            try:
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
            except Exception:
                pass
            if stop.wait(1.0):
                break

    threading.Thread(target=_gcs_ping, daemon=True, name="mavlink-hb-ping").start()
    try:
        return master.wait_heartbeat(timeout=timeout)
    finally:
        stop.set()
