"""Shared runtime handles (Flask app, SocketIO) — set once at startup."""
from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from flask import Flask
    from flask_socketio import SocketIO

app: Flask | None = None
socketio: SocketIO | None = None
pi_ctrl_sock = None  # UDP socket for thrust commands
