#!/usr/bin/env python3
"""
Camera streaming server — ONBOARD  (runs on the Raspberry Pi)
=============================================================
Serves two USB camera feeds as MJPEG HTTP streams.

  Camera 0 → http://0.0.0.0:8160/
  Camera 1 → http://0.0.0.0:8161/

The Flask topside (rov_ui.py) proxies forward at /camera/1 and arm at /camera/2.

ROV wiring (do not swap):
  /dev/video0 → Pi cam0 / port 8160 → arm
  /dev/video2 → Pi cam1 / port 8161 → forward

Install dependency on the Pi:
  pip install opencv-python-headless

Usage:
  python3 camera_stream.py
  python3 camera_stream.py --cam0 /dev/video0 --cam1 /dev/video2 --fps 15 --quality 70
"""

import argparse
import gc
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

try:
    import cv2
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False
    print("[cam] WARNING: opencv-python-headless not installed.")
    print("[cam]   Install with:  pip install opencv-python-headless")

BOUNDARY = b"frame"
_placeholder_lock = threading.Lock()
_placeholder_frame: bytes | None = None

# V4L2_CAP_VIDEO_CAPTURE from linux/videodev2.h
_V4L2_CAP_VIDEO_CAPTURE = 0x00000001


def _log(msg: str) -> None:
    print(msg, flush=True)


def _is_usb_webcam(device: str) -> bool:
    """USB UVC camera only (excludes Pi internal pispbe / hevc nodes)."""
    name = Path(device).name
    dev_node = Path(f"/sys/class/video4linux/{name}/device")
    try:
        real = dev_node.resolve()
    except OSError:
        return False
    return "/usb" in str(real).lower()


def _is_capture_node(device: str) -> bool:
    """True if sysfs marks this node as a video-capture device (not metadata)."""
    name = Path(device).name
    caps_path = Path(f"/sys/class/video4linux/{name}/device_caps")
    if not caps_path.is_file():
        return _is_primary_capture_node(device)
    try:
        caps = int(caps_path.read_text(encoding="utf-8").strip(), 16)
        return bool(caps & _V4L2_CAP_VIDEO_CAPTURE)
    except (OSError, ValueError):
        return True


def _is_primary_capture_node(device: str) -> bool:
    """
    UVC webcams expose /dev/videoN (capture, index=0) and /dev/videoN+1 (metadata).
    Prefer nodes with index 0 — matches v4l2-ctl --list-devices grouping.
    """
    name = Path(device).name
    index_path = Path(f"/sys/class/video4linux/{name}/index")
    if index_path.is_file():
        try:
            return int(index_path.read_text(encoding="utf-8").strip()) == 0
        except (OSError, ValueError):
            pass
    # Odd-numbered nodes are usually metadata when caps sysfs is missing.
    if name[5:].isdigit():
        return int(name[5:]) % 2 == 0
    return True


def _device_to_v4l2_index(device: str) -> int:
    """Map /dev/videoN → N for OpenCV (Pi builds reject path strings)."""
    name = Path(device).name
    if name.startswith("video") and name[5:].isdigit():
        return int(name[5:])
    return int(device)


def _device_busy_pids(device: str) -> str:
    try:
        proc = subprocess.run(
            ["fuser", device],
            capture_output=True,
            text=True,
            check=False,
        )
        if proc.returncode == 0:
            return proc.stdout.strip()
    except OSError:
        pass
    return ""


def release_video_device(device: str, *, force: bool = False) -> None:
    """Log and optionally SIGKILL processes holding a V4L2 node."""
    pids = _device_busy_pids(device)
    if not pids:
        return
    _log(f"[cam] {device} busy — PIDs: {pids}")
    if force:
        subprocess.run(
            ["fuser", "-k", device],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        time.sleep(0.4)


def list_capture_devices(*, usb_only: bool = False) -> list[str]:
    """Return /dev/videoN paths that support video capture, in numeric order."""
    base = Path("/sys/class/video4linux")
    if not base.is_dir():
        return []
    devices: list[tuple[int, str]] = []
    for entry in base.glob("video*"):
        if not entry.name[5:].isdigit():
            continue
        dev = f"/dev/{entry.name}"
        if usb_only and not _is_usb_webcam(dev):
            continue
        if not _is_capture_node(dev):
            continue
        if usb_only and not _is_primary_capture_node(dev):
            continue
        devices.append((int(entry.name[5:]), dev))
    devices.sort(key=lambda item: item[0])
    return [dev for _, dev in devices]


def resolve_camera_device(requested: str, used: set[str]) -> str:
    """
    Pick a capture-capable V4L2 node.

    UVC webcams expose two nodes per camera (video + metadata).  Opening the
    metadata node produces OpenCV's "can't be used to capture" warning.
    """
    req = (requested or "").strip()
    if req and req.lower() != "auto":
        if not _is_capture_node(req):
            idx = Path(req).name
            alt = f"/dev/video{int(idx[5:]) - 1}" if idx[5:].isdigit() else ""
            if alt and _is_capture_node(alt) and alt not in used:
                _log(f"[cam] {req} is a metadata node — using {alt} instead")
                return alt
        return req

    for dev in list_capture_devices(usb_only=True):
        if dev not in used:
            return dev
    return req or "/dev/video0"


def _fourcc(name: str) -> int:
    return cv2.VideoWriter_fourcc(*name)


# UVC webcams on Pi usually need MJPEG for reliable capture at 640×480.
_CAPTURE_PROFILES: tuple[tuple[str, int, int], ...] = (
    ("MJPG", 640, 480),
    ("MJPG", 320, 240),
    ("YUYV", 640, 480),
    ("YUYV", 320, 240),
)

# Per-device open lock — different USB cameras can open in parallel on Pi.
_open_locks: dict[str, threading.Lock] = {}
_open_locks_guard = threading.Lock()


def _device_open_lock(device: str) -> threading.Lock:
    with _open_locks_guard:
        if device not in _open_locks:
            _open_locks[device] = threading.Lock()
        return _open_locks[device]


def _capture_profiles(width: int, height: int) -> list[tuple[str, int, int]]:
    seen: set[tuple[str, int, int]] = set()
    ordered: list[tuple[str, int, int]] = []
    for fmt, w, h in (("MJPG", width, height), *_CAPTURE_PROFILES):
        key = (fmt, w, h)
        if key not in seen:
            seen.add(key)
            ordered.append(key)
    return ordered


def _v4l2_prepare(device: str, width: int, height: int, fourcc: str = "MJPG") -> None:
    """Set format via v4l2-ctl before OpenCV opens (more reliable on Pi 5)."""
    subprocess.run(
        [
            "v4l2-ctl", f"--device={device}",
            f"--set-fmt-video=width={width},height={height},pixelformat={fourcc}",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def _v4l2_prepare_all(devices: list[str], width: int, height: int) -> None:
    """Pre-configure every camera in parallel (separate USB buses)."""
    threads = [
        threading.Thread(
            target=_v4l2_prepare,
            args=(dev, width, height, "MJPG"),
            daemon=True,
            name=f"v4l2-{Path(dev).name}",
        )
        for dev in devices
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=3.0)


def _release_capture(cap: "cv2.VideoCapture | None") -> None:
    if cap is None:
        return
    try:
        cap.release()
    except Exception:
        pass
    gc.collect()


def _configure_capture(
    cap: "cv2.VideoCapture", device: str, width: int, height: int, fps: int,
) -> tuple[int, int, str]:
    """
    Try common UVC format profiles.  Returns (actual_w, actual_h, fourcc_used).
    MJPEG first — uncompressed YUYV at 640×480 often fails with two cameras.
    """
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for fourcc_name, w, h in _capture_profiles(width, height):
        _v4l2_prepare(device, w, h, fourcc_name)
        cap.set(cv2.CAP_PROP_FOURCC, _fourcc(fourcc_name))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        cap.set(cv2.CAP_PROP_FPS, fps)
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # Warm up — first frames after format change are often empty on UVC.
        for _ in range(2):
            cap.grab()
        ret, frame = cap.read()
        if ret and frame is not None and frame.size > 0:
            return actual_w, actual_h, fourcc_name
    return width, height, "?"


def _build_placeholder(width: int, height: int) -> bytes | None:
    """Render a simple dark 'NO SIGNAL' JPEG used while a camera is unavailable."""
    if not HAVE_CV2:
        return None
    try:
        import numpy as np
        img = np.zeros((height, width, 3), dtype=np.uint8)
        text = "NO SIGNAL"
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = max(0.8, min(width, height) / 300)
        thick = max(1, int(scale * 2))
        (tw, th), _ = cv2.getTextSize(text, font, scale, thick)
        org = ((width - tw) // 2, (height + th) // 2)
        cv2.putText(img, text, org, font, scale, (55, 55, 55), thick, cv2.LINE_AA)
        _, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 50])
        return jpeg.tobytes()
    except Exception:
        return None


class CameraStream:
    """
    Captures JPEG frames from one V4L2 device in a background thread.
    Automatically retries with exponential back-off if the device is
    unavailable or a read fails.
    """

    def __init__(self, device: str, width: int = 640, height: int = 480,
                 fps: int = 15, quality: int = 75):
        self.device  = device
        self.width   = width
        self.height  = height
        self.fps     = fps
        self.quality = quality
        self._lock   = threading.Lock()
        self._frame: bytes | None = None
        self._running = False
        self._ready   = threading.Event()

    def wait_ready(self, timeout: float = 15.0) -> bool:
        """Block until the first frame is captured (or timeout)."""
        return self._ready.wait(timeout)

    def start(self) -> "CameraStream":
        self._running = True
        t = threading.Thread(
            target=self._capture_loop,
            daemon=True,
            name=f"cam-{self.device}",
        )
        t.start()
        return self

    def _open_capture(self) -> "cv2.VideoCapture | None":
        """
        Open via numeric V4L2 index — Pi OpenCV builds reject /dev/videoN paths
        ("can't be used to capture by name") but accept cv2.VideoCapture(N, CAP_V4L2).
        """
        with _device_open_lock(self.device):
            idx = _device_to_v4l2_index(self.device)
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if not cap.isOpened():
                pids = _device_busy_pids(self.device)
                if pids:
                    _log(f"[cam] {self.device} (index {idx}): open failed — "
                         f"device busy (PIDs {pids})")
                _release_capture(cap)
                return None
            try:
                actual_w, actual_h, fmt = _configure_capture(
                    cap, self.device, self.width, self.height, self.fps,
                )
                if fmt == "?":
                    _release_capture(cap)
                    return None
                self.width = actual_w
                self.height = actual_h
                _log(f"[cam] {self.device}: opened {actual_w}×{actual_h} fmt={fmt}")
            except Exception as exc:
                _log(f"[cam] {self.device}: configure failed ({exc})")
                _release_capture(cap)
                return None
            return cap

    def _capture_loop(self) -> None:
        cap = None
        retry_delay = 1.0

        while self._running:
            # (Re-)open the camera device
            if cap is None or not cap.isOpened():
                cap = self._open_capture()
                if cap is not None and cap.isOpened():
                    retry_delay = 1.0
                else:
                    _release_capture(cap)
                    cap = None
                    _log(f"[cam] {self.device}: not available — "
                         f"retry in {retry_delay:.1f}s")
                    time.sleep(retry_delay)
                    retry_delay = min(retry_delay * 1.5, 5.0)
                    continue

            # Grab and encode a frame (skip corrupt/partial MJPEG frames)
            ret, frame = cap.read()
            if not ret or frame is None or frame.size == 0:
                for _ in range(2):
                    ret, frame = cap.read()
                    if ret and frame is not None and frame.size > 0:
                        break
            if not ret or frame is None or frame.size == 0:
                _log(f"[cam] {self.device}: read failed — reopening")
                _release_capture(cap)
                cap = None
                time.sleep(0.5)
                continue

            ok, jpeg = cv2.imencode(
                ".jpg", frame,
                [cv2.IMWRITE_JPEG_QUALITY, self.quality],
            )
            if ok:
                with self._lock:
                    self._frame = jpeg.tobytes()
                self._ready.set()

            time.sleep(1.0 / max(1, self.fps))

    def get_frame(self) -> bytes | None:
        with self._lock:
            return self._frame

    def stop(self) -> None:
        self._running = False


class _MJPEGHandler(BaseHTTPRequestHandler):
    """
    Serves a continuous MJPEG stream (and single snapshots) from a
    bound CameraStream.  One instance per client connection; multiple
    clients are handled concurrently by ThreadingHTTPServer.
    """

    _stream: CameraStream

    def do_GET(self) -> None:
        if self.path in ("/", "/stream", "/video"):
            self._serve_stream()
        elif self.path == "/snapshot":
            self._serve_snapshot()
        else:
            self.send_error(404)

    def do_HEAD(self) -> None:
        """Respond to HEAD so the Flask proxy can sniff the Content-Type."""
        self.send_response(200)
        self.send_header(
            "Content-Type",
            f"multipart/x-mixed-replace; boundary={BOUNDARY.decode()}",
        )
        self.end_headers()

    def _serve_stream(self) -> None:
        self.send_response(200)
        self.send_header(
            "Content-Type",
            f"multipart/x-mixed-replace; boundary={BOUNDARY.decode()}",
        )
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma",        "no-cache")
        self.end_headers()

        interval = 1.0 / max(1, self.__class__._stream.fps)

        while True:
            frame = self.__class__._stream.get_frame()
            if frame is None:
                frame = _placeholder_frame
            if frame is None:
                time.sleep(0.1)
                continue
            try:
                self.wfile.write(
                    b"--" + BOUNDARY + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n"
                    + frame + b"\r\n"
                )
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                break
            time.sleep(interval)

    def _serve_snapshot(self) -> None:
        frame = self.__class__._stream.get_frame() or _placeholder_frame
        if frame is None:
            self.send_error(503, "No frame available")
            return
        self.send_response(200)
        self.send_header("Content-Type",   "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.end_headers()
        self.wfile.write(frame)

    def log_message(self, *_) -> None:
        pass  # silence per-request access log noise


def _make_handler(stream: CameraStream) -> type:
    """Dynamically create a handler subclass bound to a specific stream."""
    return type("MJPEGHandler", (_MJPEGHandler,), {"_stream": stream})


def _serve_camera(stream: CameraStream, port: int) -> None:
    handler = _make_handler(stream)
    server  = ThreadingHTTPServer(("0.0.0.0", port), handler)
    server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    print(f"[cam] http://0.0.0.0:{port}  ←  {stream.device}", flush=True)
    server.serve_forever()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="ROV dual-camera MJPEG streaming server",
    )
    parser.add_argument(
        "--cam0",    default="/dev/video0",
        help="Camera 0 V4L2 device (default /dev/video0)",
    )
    parser.add_argument(
        "--cam1",    default="/dev/video2",
        help="Camera 1 V4L2 device (default /dev/video2)",
    )
    parser.add_argument(
        "--port0",   type=int, default=8160,
        help="HTTP port for camera 0 (default 8160)",
    )
    parser.add_argument(
        "--port1",   type=int, default=8161,
        help="HTTP port for camera 1 (default 8161)",
    )
    parser.add_argument(
        "--width",   type=int, default=640,
        help="Frame width  (default 640)",
    )
    parser.add_argument(
        "--height",  type=int, default=480,
        help="Frame height (default 480)",
    )
    parser.add_argument(
        "--fps",     type=int, default=15,
        help="Target FPS   (default 15)",
    )
    parser.add_argument(
        "--quality", type=int, default=75,
        help="JPEG quality 1–100 (default 75)",
    )
    args = parser.parse_args()

    if not HAVE_CV2:
        _log("[cam] FATAL: opencv-python-headless not installed.")
        _log("[cam]   pip install opencv-python-headless")
        _log("[cam]   Streams will serve placeholder frames only.")

    global _placeholder_frame
    _placeholder_frame = _build_placeholder(args.width, args.height)

    usb_cams = list_capture_devices(usb_only=True)
    if usb_cams:
        _log(f"[cam] USB webcams: {', '.join(usb_cams)}")
    else:
        _log("[cam] WARNING: no USB capture devices found")
    platform_count = len(list_capture_devices()) - len(usb_cams)
    if platform_count > 0:
        _log(f"[cam] ({platform_count} Pi ISP/codec nodes ignored — not webcams)")

    used: set[str] = set()
    cam0_dev = resolve_camera_device(args.cam0, used)
    used.add(cam0_dev)
    cam1_dev = resolve_camera_device(args.cam1, used)

    for dev in (cam0_dev, cam1_dev):
        release_video_device(dev, force=True)

    stream0 = CameraStream(
        cam0_dev, args.width, args.height, args.fps, args.quality,
    )
    stream1 = CameraStream(
        cam1_dev, args.width, args.height, args.fps, args.quality,
    )

    if HAVE_CV2:
        _v4l2_prepare_all([cam0_dev, cam1_dev], args.width, args.height)
        _log("[cam] Starting both capture threads in parallel")
        stream0.start()
        stream1.start()

    t0 = threading.Thread(
        target=_serve_camera, args=(stream0, args.port0),
        daemon=True, name="srv-cam0",
    )
    t1 = threading.Thread(
        target=_serve_camera, args=(stream1, args.port1),
        daemon=True, name="srv-cam1",
    )

    _log("[cam] Dual-camera MJPEG server running")
    _log(f"[cam]   Cam 0 ({cam0_dev}): http://0.0.0.0:{args.port0}")
    _log(f"[cam]   Cam 1 ({cam1_dev}): http://0.0.0.0:{args.port1}")
    _log(f"[cam]   {args.width}×{args.height}  @{args.fps}fps  quality={args.quality}")

    t0.start()
    t1.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[cam] Stopping.")
        stream0.stop()
        stream1.stop()


if __name__ == "__main__":
    main()
