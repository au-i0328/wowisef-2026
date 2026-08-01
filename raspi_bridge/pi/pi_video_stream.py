#!/usr/bin/env python3
"""
Pi Zero 2 W - USB Webcam Video Streamer
=======================================
Reads frames from a USB webcam (V4L2 /dev/video0) and exposes:
  GET /stream    -> MJPEG multipart stream (used by mac_dashboard.py)
  GET /snapshot  -> single JPEG
  GET /mp4       -> DOES NOT exist; recording is driven by the laptop's
                    mac_dashboard.py, which writes mp4 from the MJPEG stream
                    it already pulls. Keeping the streamer MJPEG-only avoids
                    re-encoding on the Pi.

If you want the Pi to write mp4 instead, enable --ffmpeg (see README).

Run on the Pi:
  python3 pi_video_stream.py                         # default /dev/video0
  python3 pi_video_stream.py --device /dev/video1   # different USB device
  python3 pi_video_stream.py --test                  # serve a synthetic frame
"""

import argparse
import logging
import os
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional, Tuple

import cv2
import numpy as np

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("video")

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8080
PART_BOUNDARY = "123456789000000000000987654321"


# ---------------------------- Source ----------------------------
class FrameSource:
    """Thread-safe source. Pulls frames from a real V4L2 camera or a synthetic
    loop, sized once on the first frame."""

    def __init__(self, device: Optional[str], width: int, height: int,
                 fps: float, test_mode: bool):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.test_mode = test_mode
        self._cap: Optional[cv2.VideoCapture] = None
        self._lock = threading.Lock()
        self._last_jpeg: Optional[bytes] = None
        self._last_shape: Optional[Tuple[int, int]] = None
        self._t0 = 0.0
        self._frame_n = 0
        self._opened = False

    def open(self) -> bool:
        """Try to open the real device. On failure, return False; the
        driver's outer retry loop will handle backoff / fallback."""
        if self.test_mode:
            self._t0 = time.time()
            self._opened = True
            log.info("Video source: synthetic test pattern")
            return True
        # Try V4L2 first, default fallback
        for backend in (cv2.CAP_V4L2, cv2.CAP_ANY):
            try:
                cap = cv2.VideoCapture(self.device, backend)
            except Exception:
                continue
            if cap.isOpened():
                self._cap = cap
                self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if self.width:  self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
                if self.height: self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                if self.fps:    self._cap.set(cv2.CAP_PROP_FPS,          self.fps)
                self._opened = True
                log.info("Video source: %s (backend=%s) %dx%d @ %.1f fps",
                         self.device,
                         "V4L2" if backend == cv2.CAP_V4L2 else "ANY",
                         int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                         int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                         self._cap.get(cv2.CAP_PROP_FPS))
                return True
            cap.release()
        log.warning("Video source: failed to open %s", self.device)
        return False

    def close(self):
        if self._cap:
            self._cap.release()
            self._cap = None

    def _read_one(self) -> Tuple[Optional[bytes], Optional[Tuple[int, int]]]:
        """Returns (jpeg_bytes, (w, h)) or (None, None) on failure."""
        if self.test_mode:
            return self._synthetic_frame()
        if not self._cap:
            return None, None
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return None, None
        h, w = frame.shape[:2]
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return None, None
        return buf.tobytes(), (w, h)

    def _synthetic_frame(self) -> Tuple[bytes, Tuple[int, int]]:
        w, h = self.width or 640, self.height or 480
        t = time.time() - self._t0
        img = np.zeros((h, w, 3), dtype=np.uint8)
        # Moving gradient + timestamp
        for y in range(h):
            img[y, :, 0] = (y + int(t * 80)) & 0xFF
            img[y, :, 1] = (y + int(t * 40)) & 0xFF
            img[y, :, 2] = (y * 2 + int(t * 30)) & 0xFF
        cv2.putText(img, f"TEST PATTERN  t={t:.1f}s",
                    (16, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (255, 255, 255), 2, cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes(), (w, h)

    def latest_jpeg(self) -> Tuple[Optional[bytes], Optional[Tuple[int, int]]]:
        with self._lock:
            # Always grab a fresh frame so the cached one isn't stale
            data, shape = self._read_one()
            if data is not None:
                self._last_jpeg = data
                self._last_shape = shape
                self._frame_n += 1
            return self._last_jpeg, self._last_shape


# ---------------------------- HTTP handler ----------------------------
class StreamHandler(BaseHTTPRequestHandler):
    server_version = "PiVideoStreamer/1.0"
    source: FrameSource = None  # type: ignore[assignment]

    def log_message(self, fmt, *args):
        log.info("%s - %s", self.address_string(), fmt % args)

    def _send_common(self, status, ctype, length, ctype_extra=""):
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        if length is not None:
            self.send_header("Content-Length", str(length))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Connection", "close")
        if ctype_extra:
            self.send_header("X-Extra", ctype_extra)
        self.end_headers()

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self._send_index()
        elif self.path.startswith("/stream"):
            self._send_mjpeg()
        elif self.path.startswith("/snapshot"):
            self._send_snapshot()
        elif self.path.startswith("/health"):
            self._send_health()
        else:
            self.send_error(404)

    def _send_index(self):
        body = (
            "<html><body style='background:#111;color:#eee;font-family:system-ui;text-align:center'>"
            "<h2>Climbing Robot - Pi Video</h2>"
            "<img src='/stream' style='max-width:90%;border:2px solid #333'/>"
            "<p><a style='color:#6cf' href='/snapshot'>snapshot</a></p></body></html>"
        ).encode()
        self._send_common(200, "text/html", len(body))
        self.wfile.write(body)

    def _send_health(self):
        msg = b"OK"
        self._send_common(200, "text/plain", len(msg))
        self.wfile.write(msg)

    def _send_snapshot(self):
        data, _ = self.source.latest_jpeg()
        if not data:
            self.send_error(503, "no frame")
            return
        self._send_common(200, "image/jpeg", len(data))
        self.wfile.write(data)

    def _send_mjpeg(self):
        boundary = PART_BOUNDARY.encode()
        self.send_response(200)
        self.send_header("Content-Type",
                         f"multipart/x-mixed-replace;boundary={boundary.decode()}")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Connection", "close")
        self.end_headers()
        try:
            while True:
                data, _ = self.source.latest_jpeg()
                if data:
                    self.wfile.write(b"\r\n--" + boundary + b"\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode())
                    self.wfile.write(data)
                    self.wfile.flush()
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError):
            pass


# ---------------------------- Server ----------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", default=DEFAULT_PORT, type=int)
    ap.add_argument("--device", default="/dev/video0",
                    help="V4L2 device path. Ignored when --test is set.")
    ap.add_argument("--width",  default=640, type=int)
    ap.add_argument("--height", default=480, type=int)
    ap.add_argument("--fps",    default=20.0, type=float)
    ap.add_argument("--test", action="store_true",
                    help="Serve a synthetic test pattern (no webcam needed).")
    ap.add_argument("--auto-test", action="store_true",
                    help="Fall back to --test automatically if no camera is "
                         "found within --wait-device seconds.")
    ap.add_argument("--wait-device", default="60", type=float,
                    help="Seconds to wait for the webcam before falling back "
                         "or giving up. Use -1 to wait forever (default 60).")
    args = ap.parse_args()

    # /etc/default/climbingrobot can force mock mode at startup.
    env_force_test = os.environ.get("TEST_MODE", "0") == "1"
    test_mode = args.test or env_force_test
    fallback = args.auto_test and not test_mode

    if env_force_test and not args.test:
        log.warning("TEST_MODE=1 in environment — forcing synthetic video")

    if not test_mode:
        deadline = None if args.wait_device < 0 else time.time() + args.wait_device
        log.info("Looking for video device %s (--wait-device=%s)",
                 args.device, args.wait_device)
        while True:
            src = FrameSource(args.device, args.width, args.height, args.fps,
                              test_mode=False)
            if src.open():
                break
            src.close()
            now = time.time()
            if deadline is not None and now >= deadline:
                if fallback:
                    log.warning("No camera after %.0fs — switching to "
                                "--test mode", args.wait_device)
                    test_mode = True
                    break
                log.error(
                    "No camera device found and auto-test disabled. "
                    "Pass --test or --auto-test for a synthetic stream.",
                )
                sys.exit(1)
            time.sleep(2.0)
    if test_mode:
        src = FrameSource(args.device, args.width, args.height, args.fps,
                          test_mode=True)
        if not src.open():
            log.error("Failed to initialise synthetic source (impossible?)")
            sys.exit(1)

    StreamHandler.source = src
    httpd = ThreadingHTTPServer((args.host, args.port), StreamHandler)

    def _shutdown(*_):
        log.info("Shutting down...")
        try:
            httpd.shutdown()
        finally:
            src.close()
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    mode = "MOCK" if src.test_mode else f"device {src.device}"
    log.info("Serving video on http://%s:%d  (mode=%s; /stream /snapshot /health)",
             args.host, args.port, mode)
    try:
        httpd.serve_forever()
    finally:
        src.close()


if __name__ == "__main__":
    main()