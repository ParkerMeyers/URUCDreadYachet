#!/usr/bin/env python3
"""ROV dual H.264 RTP/UDP camera feeds (ports 5600 and 5601).

The robot sends RTP to the topside laptop IP on these ports (see onboard/camera_streamer.sh).
The UI listens locally with udpsrc — you do not point udpsrc at the robot IP.
"""

import os
import shutil
import subprocess
import time

# Same ports as ROV_Cameras.sh
ROV_CAMERA_PORTS = (5600, 5601)

# Common Windows install path when not on PATH
WINDOWS_GSTREAMER_CANDIDATES = [
    r"C:\Program Files\gstreamer\1.0\msvc_x86_64\bin\gst-launch-1.0.exe",
    r"C:\gstreamer\1.0\msvc_x86_64\bin\gst-launch-1.0.exe",
]


def resolve_gst_launch_executable():
    found = shutil.which("gst-launch-1.0")
    if found:
        return found
    for candidate in WINDOWS_GSTREAMER_CANDIDATES:
        if os.path.exists(candidate):
            return candidate
    return None


def parse_rov_udp_source(source):
    """Parse rov-udp:5600 or rov-udp:5600@192.168.2.249 (legacy hint; listen is still local)."""
    if not isinstance(source, str) or not source.startswith("rov-udp:"):
        return None, None
    rest = source.split(":", 1)[1]
    if "@" in rest:
        port_text, _host = rest.split("@", 1)
    else:
        port_text = rest
    try:
        return int(port_text), None
    except ValueError:
        return None, None


def h264_rtp_pipeline(port, width=640, height=480):
    """GStreamer pipeline string for OpenCV appsink (ROV_Cameras.sh equivalent)."""
    return (
        f'udpsrc port={port} caps="application/x-rtp, media=video, clock-rate=90000, encoding-name=H264" '
        f"! rtph264depay ! avdec_h264 ! videoconvert ! "
        f"video/x-raw,format=BGR,width={width},height={height} ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


def gst_subprocess_args(port, width=640, height=480, gst_executable=None):
    """gst-launch-1.0 argv for raw BGR frames on stdout (fallback path)."""
    gst_executable = gst_executable or resolve_gst_launch_executable() or "gst-launch-1.0"
    caps = "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264"
    return [
        gst_executable,
        "-q",
        f"udpsrc port={port} caps={caps}",
        "!",
        "rtph264depay",
        "!",
        "avdec_h264",
        "!",
        "videoconvert",
        "!",
        f"video/x-raw,format=BGR,width={width},height={height}",
        "!",
        "fdsink",
        "fd=1",
    ]


def default_camera_sources():
    return [
        os.getenv("ROV_CAMERA_1_URL", f"rov-udp:{ROV_CAMERA_PORTS[0]}"),
        os.getenv("ROV_CAMERA_2_URL", f"rov-udp:{ROV_CAMERA_PORTS[1]}"),
    ]


def camera_display_label(source):
    port, _host = parse_rov_udp_source(source)
    if port is not None:
        return f"listen UDP :{port} (from robot)"
    return str(source)


def _parse_rov_udp_port(source):
    port, _host = parse_rov_udp_source(source)
    return port


class RovCameraStream:
    """Reads frames from an ROV RTP feed or a generic OpenCV source."""

    def __init__(self, source, cv2=None, width=640, height=480):
        self.source = source
        self.cv2 = cv2
        self.width = width
        self.height = height
        self.label = camera_display_label(source)
        self._cap = None
        self._gst_proc = None
        self._gst_executable = resolve_gst_launch_executable()
        self._frame_bytes = width * height * 3
        self._last_open_attempt = 0.0
        self._open()

    def _open(self):
        self.release()
        self._last_open_attempt = time.time()

        port = _parse_rov_udp_port(self.source)
        if port is not None and self.cv2 is not None:
            pipeline = h264_rtp_pipeline(port, self.width, self.height)
            try:
                cap = self.cv2.VideoCapture(pipeline, self.cv2.CAP_GSTREAMER)
                if cap.isOpened():
                    self._cap = cap
                    self.label = f"listen UDP :{port} (OpenCV GStreamer)"
                    return
                cap.release()
            except Exception:
                pass

            if self._gst_executable:
                try:
                    self._gst_proc = subprocess.Popen(
                        gst_subprocess_args(
                            port, self.width, self.height, self._gst_executable
                        ),
                        stdout=subprocess.PIPE,
                        stderr=subprocess.DEVNULL,
                    )
                    self.label = f"listen UDP :{port} (gst-launch)"
                    return
                except Exception:
                    self._gst_proc = None

        if self.cv2 is not None and self.source:
            try:
                open_arg = self.source
                if isinstance(self.source, str) and self.source.startswith("gstreamer:"):
                    open_arg = self.source.split(":", 1)[1]
                cap = self.cv2.VideoCapture(open_arg)
                if cap.isOpened():
                    self._cap = cap
                    return
                cap.release()
            except Exception:
                pass

    def read(self):
        if self._cap is not None:
            try:
                ok, frame = self._cap.read()
                if ok and frame is not None:
                    return True, frame
            except Exception:
                pass
            return False, None

        if self._gst_proc is not None and self._gst_proc.poll() is None:
            try:
                import numpy as np

                raw = self._gst_proc.stdout.read(self._frame_bytes)
                if raw and len(raw) == self._frame_bytes:
                    frame = np.frombuffer(raw, dtype=np.uint8).reshape(
                        (self.height, self.width, 3)
                    )
                    return True, frame.copy()
            except Exception:
                pass
            return False, None

        if time.time() - self._last_open_attempt > 3.0:
            self._open()
        return False, None

    def release(self):
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

        if self._gst_proc is not None:
            try:
                self._gst_proc.terminate()
                self._gst_proc.wait(timeout=2)
            except Exception:
                try:
                    self._gst_proc.kill()
                except Exception:
                    pass
            self._gst_proc = None


def gst_launch_available():
    return resolve_gst_launch_executable() is not None
