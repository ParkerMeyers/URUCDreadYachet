#!/usr/bin/env python3
"""
Camera streaming server — ONBOARD  (runs on the Raspberry Pi)
=============================================================
Serves two USB camera feeds as MJPEG HTTP streams.

  Camera 0 → http://0.0.0.0:8160/
  Camera 1 → http://0.0.0.0:8161/

The Flask topside (rov_ui.py) proxies these at /camera/1 and /camera/2.

USB cameras on Raspberry Pi typically enumerate as:
  /dev/video0  — USB camera 0 (video)
  /dev/video2  — USB camera 1 (video)
  (Each UVC camera creates two V4L2 nodes: one video, one metadata)

Install dependency on the Pi:
  pip install opencv-python-headless

Usage:
  python3 camera_stream.py
  python3 camera_stream.py --cam0 /dev/video0 --cam1 /dev/video2 --fps 15 --quality 70
"""

import argparse
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

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

    def start(self) -> "CameraStream":
        self._running = True
        t = threading.Thread(
            target=self._capture_loop,
            daemon=True,
            name=f"cam-{self.device}",
        )
        t.start()
        return self

    def _capture_loop(self) -> None:
        cap = None
        retry_delay = 5.0

        while self._running:
            # (Re-)open the camera device
            if cap is None or not cap.isOpened():
                cap = cv2.VideoCapture(self.device)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                    cap.set(cv2.CAP_PROP_FPS,          self.fps)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
                    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    actual_f = int(cap.get(cv2.CAP_PROP_FPS))
                    print(f"[cam] {self.device}: opened "
                          f"({actual_w}×{actual_h} @{actual_f}fps)")
                    retry_delay = 5.0
                else:
                    if cap:
                        cap.release()
                    cap = None
                    print(f"[cam] {self.device}: not available — "
                          f"retry in {retry_delay:.0f}s")
                    time.sleep(retry_delay)
                    retry_delay = min(retry_delay * 1.5, 30.0)
                    continue

            # Grab and encode a frame
            ret, frame = cap.read()
            if not ret:
                print(f"[cam] {self.device}: read failed — reopening")
                cap.release()
                cap = None
                time.sleep(1.0)
                continue

            ok, jpeg = cv2.imencode(
                ".jpg", frame,
                [cv2.IMWRITE_JPEG_QUALITY, self.quality],
            )
            if ok:
                with self._lock:
                    self._frame = jpeg.tobytes()

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
    print(f"[cam] http://0.0.0.0:{port}  ←  {stream.device}")
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
        print("[cam] FATAL: opencv-python-headless not installed.")
        print("[cam]   pip install opencv-python-headless")
        print("[cam]   Streams will serve placeholder frames only.")

    global _placeholder_frame
    _placeholder_frame = _build_placeholder(args.width, args.height)

    stream0 = CameraStream(
        args.cam0, args.width, args.height, args.fps, args.quality,
    )
    stream1 = CameraStream(
        args.cam1, args.width, args.height, args.fps, args.quality,
    )

    if HAVE_CV2:
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
    t0.start()
    t1.start()

    print(f"[cam] Dual-camera MJPEG server running")
    print(f"[cam]   Cam 0 ({args.cam0}): http://0.0.0.0:{args.port0}")
    print(f"[cam]   Cam 1 ({args.cam1}): http://0.0.0.0:{args.port1}")
    print(f"[cam]   {args.width}×{args.height}  @{args.fps}fps  quality={args.quality}")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[cam] Stopping.")
        stream0.stop()
        stream1.stop()


if __name__ == "__main__":
    main()
