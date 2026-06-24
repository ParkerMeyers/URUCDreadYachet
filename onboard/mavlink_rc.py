#!/usr/bin/env python3
"""RC_CHANNELS_OVERRIDE helper — works with MAVLink 1 and 2 pymavlink bindings."""
from __future__ import annotations

import os
import time

# Select the v20 dialect before pymavlink loads message definitions.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink.dialects.v20 import ardupilotmega as mav_v20

RC_IGNORE = 65535

_ENCODER = mav_v20.MAVLink(None, srcSystem=255, srcComponent=190)


def normalize_mavlink_listen_url(url: str) -> str:
    """
    Convert udp:HOST:PORT → udpin:0.0.0.0:PORT for MAVProxy companion links.

    MAVProxy --out=udp:127.0.0.1:PORT *sends to* PORT.  pymavlink must listen
    on that port (udpin).  A bare udp: URL often binds an ephemeral port and
    never receives the vehicle heartbeats MAVProxy forwards.
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
    """
    Send RC_CHANNELS_OVERRIDE for up to 18 channels.

    Always encodes with the MAVLink 2 dialect.  Pi pymavlink often exposes a
    MAVLink 1 binding (8 channels only) on master.mav — calling
    rc_channels_override_send(..., *18) raises TypeError.  Packing via v20
    avoids that and supports AUX channels 9-16 used by the arm controller.
    """
    ts, tc = _targets(master)
    ch = _pad_channels(channels, ignore=ignore)
    msg = _ENCODER.rc_channels_override_encode(ts, tc, *ch)
    master.write(msg.pack(_ENCODER))


def connect_mavlink(url: str, *, source_system: int = 255):
    """Open a MAVLink UDP/serial connection; prefer MAVLink 2 on the wire."""
    from pymavlink import mavutil

    listen_url = normalize_mavlink_listen_url(url)
    if listen_url != url:
        print(f"[mavlink] Listening on {listen_url} (from {url})")
    master = mavutil.mavlink_connection(listen_url, source_system=source_system)
    try:
        master.mav.set_protocol(mavutil.mavlink.MAVLINK_V2)
    except Exception:
        pass
    return master


def wait_for_heartbeat(master, timeout: float = 20.0):
    """
    Wait for a vehicle HEARTBEAT, sending GCS heartbeats while polling.

    MAVProxy only forwards FC traffic after the onboard listener is bound and
    sometimes after it sees inbound packets from the client.
    """
    from pymavlink import mavutil

    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
        except Exception:
            pass
        remaining = max(0.1, deadline - time.time())
        msg = master.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=min(1.0, remaining),
        )
        if msg is not None:
            src = msg.get_srcSystem()
            # Ignore our own GCS heartbeats if they loop back.
            if src and src != source_system(master):
                return msg
    return None


def source_system(master) -> int:
    return int(getattr(master, "source_system", None) or 255)
