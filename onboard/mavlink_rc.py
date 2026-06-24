#!/usr/bin/env python3
"""RC_CHANNELS_OVERRIDE helper — works with MAVLink 1 and 2 pymavlink bindings."""
from __future__ import annotations

import os

# Select the v20 dialect before pymavlink loads message definitions.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink.dialects.v20 import ardupilotmega as mav_v20

RC_IGNORE = 65535

_ENCODER = mav_v20.MAVLink(None, srcSystem=255, srcComponent=190)


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

    master = mavutil.mavlink_connection(url, source_system=source_system)
    try:
        master.mav.set_protocol(mavutil.mavlink.MAVLINK_V2)
    except Exception:
        pass
    return master
