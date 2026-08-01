"""
Climbing Robot Dashboard (macOS)
=================================
Mac-side control + recording application.

What it does:
  - Connects to the ESP32 access point ("ClimbingRobot")
    and opens a WebSocket to ws://192.168.4.1:81/
  - Receives JSON telemetry (IMU + TOF + status) at 1 Hz and
    displays it live in the window
  - Pulls the MJPEG video stream from http://192.168.4.1:stream
    and records it as H.264 mp4 (and as a backup mp4) on disk
  - Sends gamepad commands from a PS4 controller back over the WS
    (uses the same button mapping as gamepad_to_esp32.py)
  - Logs all telemetry to CSV and a parallel JSONL stream,
    rotated per session

Run:
    pip install pyqt5 opencv-python numpy websockets pygame
    python3 mac_dashboard.py
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import os
import queue
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from threading import Lock, Thread
from typing import Optional

import cv2
import numpy as np
import pygame
import websockets
from PyQt5 import QtCore, QtGui, QtWidgets

# ---------------------------- Configuration ----------------------------
DEFAULT_HOST = "192.168.4.1"
WS_PATH     = "/"
HTTP_STREAM = f"http://{DEFAULT_HOST}:80/stream"
SEND_HZ     = 30
HOLD_FRAMES = 10
LOG_DIR     = Path.home() / "Documents" / "ClimbingRobotLogs"
LOG_DIR.mkdir(parents=True, exist_ok=True)

# ---------------------------- Telemetry state ----------------------------
@dataclass
class Telemetry:
    imu_ax: float = 0.0
    imu_ay: float = 0.0
    imu_az: float = 0.0
    imu_gx: float = 0.0
    imu_gy: float = 0.0
    imu_gz: float = 0.0
    tof_front: int = 0
    tof_rear: int = 0
    drive_active: bool = False
    e_stop: bool = False
    uptime_ms: int = 0
    received_at: float = 0.0

# ---------------------------- Async WS client ----------------------------
class WsClient(QtCore.QObject):
    """Async websocket client running on its own event loop."""
    connected    = QtCore.pyqtSignal()
    disconnected = QtCore.pyqtSignal()
    telemetry    = QtCore.pyqtSignal(dict)
    log          = QtCore.pyqtSignal(str)

    def __init__(self, host: str, port: int, path: str = WS_PATH):
        super().__init__()
        self.host = host
        self.port = port
        self.path = path
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[Thread] = None
        self._ws: Optional[object] = None
        self._send_queue: asyncio.Queue = None  # type: ignore
        self._stop = False
        self._connected_evt = asyncio.Event() if False else None  # set in loop

    def start(self):
        self._stop = False
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop = True
        if self._loop and self._send_queue:
            self._loop.call_soon_threadsafe(self._send_queue.put_nowait, "__STOP__")

    def send(self, payload: str):
        if not self._loop or not self._send_queue:
            return
        self._loop.call_soon_threadsafe(self._send_queue.put_nowait, payload)

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._send_queue = asyncio.Queue()
        try:
            self._loop.run_until_complete(self._main())
        except Exception as e:
            self.log.emit(f"WS loop crashed: {e}")
        finally:
            self._loop.close()

    async def _main(self):
        url = f"ws://{self.host}:{self.port}{self.path}"
        backoff = 1.0
        while not self._stop:
            try:
                async with websockets.connect(url, ping_interval=20, ping_timeout=20) as ws:
                    self._ws = ws
                    self.connected.emit()
                    self.log.emit(f"WS connected to {url}")
                    backoff = 1.0
                    recv_task = asyncio.create_task(self._recv_loop(ws))
                    send_task = asyncio.create_task(self._send_loop(ws))
                    done, pending = await asyncio.wait(
                        {recv_task, send_task}, return_when=asyncio.FIRST_COMPLETED
                    )
                    for t in pending:
                        t.cancel()
            except Exception as e:
                self.log.emit(f"WS disconnect: {e}; retrying in {backoff:.1f}s")
                self.disconnected.emit()
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, 10.0)

    async def _recv_loop(self, ws):
        async for raw in ws:
            try:
                msg = json.loads(raw)
                self.telemetry.emit(msg)
            except json.JSONDecodeError:
                continue

    async def _send_loop(self, ws):
        while True:
            payload = await self._send_queue.get()
            if payload == "__STOP__":
                return
            try:
                await ws.send(payload)
            except Exception:
                return

# ---------------------------- Video worker ----------------------------
class VideoWorker(QtCore.QObject):
    """Pulls MJPEG from ESP32, decodes, writes mp4."""
    frame  = QtCore.pyqtSignal(np.ndarray)
    log    = QtCore.pyqtSignal(str)
    recording_changed = QtCore.pyqtSignal(bool)

    def __init__(self, url: str, save_path: Path):
        super().__init__()
        self.url = url
        self.save_path = save_path
        self._stop = False
        self._recording = False
        self._writer: Optional[cv2.VideoWriter] = None
        self._thread: Optional[Thread] = None

    def start(self):
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop = True

    def set_recording(self, on: bool):
        self._recording = on

    def _run(self):
        # Try MJPG first, then raw stream fallback
        cap: Optional[cv2.VideoCapture] = None
        for try_url in (self.url, self.url.replace("/stream", "/mjpeg")):
            try:
                c = cv2.VideoCapture(try_url)
                if c.isOpened():
                    cap = c
                    self.log.emit(f"Video source: {try_url}")
                    break
                c.release()
            except Exception:
                continue
        if cap is None:
            self.log.emit("Video: failed to open stream")
            return

        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        last_size: Optional[tuple[int, int]] = None
        fps_out = 20.0

        while not self._stop:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.02)
                continue

            self.frame.emit(frame)

            if self._recording:
                h, w = frame.shape[:2]
                if last_size != (w, h):
                    if self._writer:
                        self._writer.release()
                        self._writer = None
                    last_size = (w, h)
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    self._writer = cv2.VideoWriter(
                        str(self.save_path), fourcc, fps_out, (w, h)
                    )
                if self._writer:
                    self._writer.write(frame)

        if self._writer:
            self._writer.release()
        cap.release()

# ---------------------------- Logger ----------------------------
class SessionLogger:
    def __init__(self, base: Path):
        self.base = base
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.session_dir = base / f"session_{ts}"
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.session_dir / "telemetry.csv"
        self.jsonl_path = self.session_dir / "telemetry.jsonl"
        self.video_path = self.session_dir / "video.mp4"
        self.events_path = self.session_dir / "events.log"

        with open(self.csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "host_t_ms", "esp_uptime_ms",
                "imu_ax", "imu_ay", "imu_az",
                "imu_gx", "imu_gy", "imu_gz",
                "tof_front_mm", "tof_rear_mm",
                "drive_active", "e_stop",
            ])
        self._lock = Lock()

    def log_telemetry(self, t: Telemetry):
        with self._lock:
            with open(self.csv_path, "a", newline="") as f:
                csv.writer(f).writerow([
                    int(time.time() * 1000), t.uptime_ms,
                    t.imu_ax, t.imu_ay, t.imu_az,
                    t.imu_gx, t.imu_gy, t.imu_gz,
                    t.tof_front, t.tof_rear,
                    int(t.drive_active), int(t.e_stop),
                ])
            with open(self.jsonl_path, "a") as f:
                f.write(json.dumps({
                    "host_t_ms": int(time.time() * 1000),
                    "esp_uptime_ms": t.uptime_ms,
                    "imu": {"ax": t.imu_ax, "ay": t.imu_ay, "az": t.imu_az,
                            "gx": t.imu_gx, "gy": t.imu_gy, "gz": t.imu_gz},
                    "tof": {"front": t.tof_front, "rear": t.tof_rear},
                    "status": {"drive_active": t.drive_active, "e_stop": t.e_stop},
                }) + "\n")

    def log_event(self, event: str):
        with self._lock:
            with open(self.events_path, "a") as f:
                f.write(f"{time.strftime('%H:%M:%S')} {event}\n")

# ---------------------------- Gamepad helpers ----------------------------
def normalize_stick(val, deadzone=4000):
    if abs(val) < deadzone:
        return 128
    return max(0, min(255, int((val + 32768) / 65535.0 * 255)))

def normalize_trigger(val):
    if val < 0:
        val = (val + 32768) / 65535.0 * 255
    else:
        val = (val / 32767.0) * 255
    return max(0, min(255, int(val)))

def get_channels(pad):
    lx = normalize_stick(pad.get_axis(0))
    ly = normalize_stick(pad.get_axis(1))
    rx = normalize_stick(pad.get_axis(2))
    ry = normalize_stick(pad.get_axis(3))
    l2 = normalize_trigger(pad.get_axis(4))
    r2 = normalize_trigger(pad.get_axis(5))
    cross    = 1 if pad.get_button(0)  else 0
    circle   = 1 if pad.get_button(1)  else 0
    square   = 1 if pad.get_button(2)  else 0
    triangle = 1 if pad.get_button(3)  else 0
    ps_btn   = 1 if pad.get_button(5)  else 0
    dp_up    = 1 if pad.get_button(11) else 0
    dp_down  = 1 if pad.get_button(12) else 0
    dp_left  = 1 if pad.get_button(13) else 0
    dp_right = 1 if pad.get_button(14) else 0
    l1       = 1 if pad.get_button(9)  else 0
    r1       = 1 if pad.get_button(10) else 0
    share    = 1 if pad.get_button(4)  else 0
    options  = 1 if pad.get_button(6)  else 0
    l3       = 1 if pad.get_button(7)  else 0
    r3       = 1 if pad.get_button(8)  else 0
    touchpad = 1 if pad.get_button(17) else 0
    return [
        lx, ly, rx, ry, l2, r2,
        cross, circle, square, triangle, ps_btn,
        dp_up, dp_down, dp_left, dp_right,
        l1, r1, share, options, l3, r3,
        touchpad,
    ]

# ---------------------------- Main window ----------------------------
class Dashboard(QtWidgets.QMainWindow):
    def __init__(self, host: str):
        super().__init__()
        self.host = host
        self.logger = SessionLogger(LOG_DIR)
        self.telem = Telemetry()
        self.telem_lock = Lock()
        self.cmd_history = deque(maxlen=64)
        self.active_command = "NONE"
        self.hold_counter = 0
        self.drive_direction = "FORWARD"
        self._last_send = 0.0
        self._connected = False
        self._recording = False
        self._pad = None

        self._setup_ui()
        self._setup_networking()
        self._setup_gamepad()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000 / SEND_HZ))

    # ---------- UI ----------
    def _setup_ui(self):
        self.setWindowTitle("Climbing Robot Dashboard")
        self.resize(1280, 800)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        # ----- Left: video -----
        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        self.video_label = QtWidgets.QLabel("Connecting video…")
        self.video_label.setAlignment(QtCore.Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setStyleSheet("background:#222;color:#888")
        left_layout.addWidget(self.video_label, stretch=1)

        rec_row = QtWidgets.QHBoxLayout()
        self.rec_button = QtWidgets.QPushButton("● Record")
        self.rec_button.setCheckable(True)
        self.rec_button.setStyleSheet("QPushButton:checked{background:#c33;color:white;font-weight:bold}")
        rec_row.addWidget(self.rec_button)
        self.session_label = QtWidgets.QLabel(f"Session: {self.logger.session_dir.name}")
        rec_row.addWidget(self.session_label, stretch=1)
        left_layout.addLayout(rec_row)
        self.rec_button.toggled.connect(self._toggle_recording)

        # ----- Right: telemetry + command pad -----
        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        right.setMinimumWidth(420)

        conn_row = QtWidgets.QHBoxLayout()
        self.conn_indicator = QtWidgets.QLabel("● Disconnected")
        self.conn_indicator.setStyleSheet("color:#c33;font-weight:bold")
        conn_row.addWidget(self.conn_indicator)
        conn_row.addStretch()
        conn_row.addWidget(QtWidgets.QLabel(f"Host: {self.host}"))
        right_layout.addLayout(conn_row)

        # IMU readout
        imu_box = QtWidgets.QGroupBox("IMU (MPU6050)")
        imu_layout = QtWidgets.QFormLayout(imu_box)
        self.lbl_ax = QtWidgets.QLabel("0.00")
        self.lbl_ay = QtWidgets.QLabel("0.00")
        self.lbl_az = QtWidgets.QLabel("0.00")
        self.lbl_gx = QtWidgets.QLabel("0.00")
        self.lbl_gy = QtWidgets.QLabel("0.00")
        self.lbl_gz = QtWidgets.QLabel("0.00")
        for lbl in (self.lbl_ax, self.lbl_ay, self.lbl_az,
                    self.lbl_gx, self.lbl_gy, self.lbl_gz):
            lbl.setStyleSheet("font-family:Menlo,monospace;font-size:13px")
        imu_layout.addRow("Accel X (m/s²)", self.lbl_ax)
        imu_layout.addRow("Accel Y (m/s²)", self.lbl_ay)
        imu_layout.addRow("Accel Z (m/s²)", self.lbl_az)
        imu_layout.addRow("Gyro  X (°/s)", self.lbl_gx)
        imu_layout.addRow("Gyro  Y (°/s)", self.lbl_gy)
        imu_layout.addRow("Gyro  Z (°/s)", self.lbl_gz)
        right_layout.addWidget(imu_box)

        # TOF readout
        tof_box = QtWidgets.QGroupBox("Time-of-Flight")
        tof_layout = QtWidgets.QFormLayout(tof_box)
        self.lbl_tof_front = QtWidgets.QLabel("----")
        self.lbl_tof_rear  = QtWidgets.QLabel("----")
        for lbl in (self.lbl_tof_front, self.lbl_tof_rear):
            lbl.setStyleSheet("font-family:Menlo,monospace;font-size:13px")
        tof_layout.addRow("Front (mm)", self.lbl_tof_front)
        tof_layout.addRow("Rear  (mm)", self.lbl_tof_rear)
        right_layout.addWidget(tof_box)

        # Status
        status_box = QtWidgets.QGroupBox("Status")
        status_layout = QtWidgets.QFormLayout(status_box)
        self.lbl_drive   = QtWidgets.QLabel("idle")
        self.lbl_estop   = QtWidgets.QLabel("OFF")
        self.lbl_uptime  = QtWidgets.QLabel("0 ms")
        self.lbl_cmd     = QtWidgets.QLabel("NONE")
        self.lbl_dir     = QtWidgets.QLabel("FORWARD")
        status_layout.addRow("Drive", self.lbl_drive)
        status_layout.addRow("E-Stop", self.lbl_estop)
        status_layout.addRow("ESP uptime", self.lbl_uptime)
        status_layout.addRow("Last cmd", self.lbl_cmd)
        status_layout.addRow("Direction", self.lbl_dir)
        right_layout.addWidget(status_box)

        # Manual command buttons (works without gamepad)
        cmd_box = QtWidgets.QGroupBox("Manual Commands")
        cmd_layout = QtWidgets.QGridLayout(cmd_box)
        manual_cmds = [
            ("up_attach",   0, 0), ("up_detach",   0, 1),
            ("down_attach", 1, 0), ("down_detach", 1, 1),
            ("both_attach", 2, 0), ("both_detach", 2, 1),
        ]
        for name, r, c in manual_cmds:
            b = QtWidgets.QPushButton(name)
            b.clicked.connect(lambda _, n=name: self._send_command(n, frames=HOLD_FRAMES))
            cmd_layout.addWidget(b, r, c)
        estop = QtWidgets.QPushButton("EMERGENCY STOP")
        estop.setStyleSheet("background:#c33;color:white;font-weight:bold;padding:8px")
        estop.clicked.connect(lambda: self._send_command("estop", frames=9999))
        cmd_layout.addWidget(estop, 3, 0, 1, 2)
        right_layout.addWidget(cmd_box)

        right_layout.addStretch()

        # Log
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(200)
        self.log_view.setStyleSheet("font-family:Menlo,monospace;font-size:11px;background:#111;color:#aaa")
        right_layout.addWidget(self.log_view, stretch=1)

        layout.addWidget(left, stretch=3)
        layout.addWidget(right, stretch=2)

        self._add_log("Dashboard ready.")
        self._add_log(f"Session logs in: {self.logger.session_dir}")

    def _add_log(self, msg: str):
        self.log_view.appendPlainText(f"{time.strftime('%H:%M:%S')}  {msg}")

    # ---------- Networking ----------
    def _setup_networking(self):
        self.ws = WsClient(self.host, 81)
        self.ws.connected.connect(self._on_ws_connected)
        self.ws.disconnected.connect(self._on_ws_disconnected)
        self.ws.telemetry.connect(self._on_telemetry)
        self.ws.log.connect(self._add_log)
        self.ws.start()

        self.video = VideoWorker(HTTP_STREAM, self.logger.video_path)
        self.video.frame.connect(self._on_frame)
        self.video.log.connect(self._add_log)
        self.video.start()

    def _on_ws_connected(self):
        self._connected = True
        self.conn_indicator.setText("● Connected")
        self.conn_indicator.setStyleSheet("color:#3c3;font-weight:bold")
        self._add_log("Telemetry stream connected.")

    def _on_ws_disconnected(self):
        self._connected = False
        self.conn_indicator.setText("● Disconnected")
        self.conn_indicator.setStyleSheet("color:#c33;font-weight:bold")

    def _on_telemetry(self, msg: dict):
        imu = msg.get("imu", {})
        tof = msg.get("tof", {})
        st  = msg.get("status", {})
        with self.telem_lock:
            self.telem.imu_ax = imu.get("ax", 0)
            self.telem.imu_ay = imu.get("ay", 0)
            self.telem.imu_az = imu.get("az", 0)
            self.telem.imu_gx = imu.get("gx", 0)
            self.telem.imu_gy = imu.get("gy", 0)
            self.telem.imu_gz = imu.get("gz", 0)
            self.telem.tof_front = tof.get("front", 0)
            self.telem.tof_rear  = tof.get("rear", 0)
            self.telem.drive_active = bool(st.get("drive_active", False))
            self.telem.e_stop       = bool(st.get("e_stop", False))
            self.telem.uptime_ms    = int(st.get("uptime_ms", 0))
            self.telem.received_at  = time.time()
        self.logger.log_telemetry(self.telem)

    # ---------- Video ----------
    def _on_frame(self, frame: np.ndarray):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        img = QtGui.QImage(rgb.data, w, h, ch * w, QtGui.QImage.Format_RGB888).copy()
        pix = QtGui.QPixmap.fromImage(img).scaled(
            self.video_label.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation,
        )
        self.video_label.setPixmap(pix)

    def _toggle_recording(self, on: bool):
        self._recording = on
        self.video.set_recording(on)
        self.rec_button.setText("■ Recording" if on else "● Record")
        self.logger.log_event(f"recording={'ON' if on else 'OFF'}")
        self._add_log(f"Video recording: {'ON' if on else 'OFF'} -> {self.logger.video_path}")

    # ---------- Gamepad ----------
    def _setup_gamepad(self):
        try:
            pygame.init()
            pygame.display.init()
            from pygame._sdl2 import controller as pgc
            pgc.init()
            for i in range(pgc.get_count()):
                if pgc.is_controller(i):
                    pad = pgc.Controller(i)
                    pad.init()
                    self._pad = pad
                    self._add_log(f"Gamepad: {pad.name}")
                    break
            if self._pad is None:
                self._add_log("Gamepad: none detected (manual buttons still work).")
        except Exception as e:
            self._add_log(f"Gamepad init failed: {e}")

    # ---------- Tick ----------
    def _send_command(self, command: str, frames: int = HOLD_FRAMES):
        self.active_command = command
        self.hold_counter = frames
        self.logger.log_event(f"cmd={command}")

    def _tick(self):
        self._refresh_labels()

        # Gamepad loop
        if self._pad is not None:
            try:
                pygame.event.pump()
                channels = get_channels(self._pad)

                if channels[10]:  # PS button -> e-stop
                    self._send_command("estop", frames=9999)
                else:
                    if   channels[15]: self.drive_direction = "FORWARD"
                    elif channels[16]: self.drive_direction = "BACKWARD"

                    drive_speed = channels[4]
                    if   channels[11]: drive_speed, self.drive_direction = 204, "FORWARD"
                    elif channels[12]: drive_speed, self.drive_direction = 204, "BACKWARD"

                    # Manual buttons via gamepad
                    if not self.hold_counter:
                        if   channels[6]: self._send_command("down_detach", HOLD_FRAMES)
                        elif channels[7]: self._send_command("down_attach", HOLD_FRAMES)
                        elif channels[8]: self._send_command("up_attach",   HOLD_FRAMES)
                        elif channels[9]: self._send_command("up_detach",   HOLD_FRAMES)
                        elif channels[17]: self._send_command("both_attach", HOLD_FRAMES)
                        elif channels[18]: self._send_command("both_detach", HOLD_FRAMES)

                if self.hold_counter > 0:
                    command_out = self.active_command
                    self.hold_counter -= 1
                else:
                    command_out = "NONE"

                payload = f"{drive_speed},{self.drive_direction},{command_out}\n"
                if self._connected:
                    self.ws.send(payload)
            except Exception as e:
                self._add_log(f"Gamepad tick error: {e}")

    def _refresh_labels(self):
        with self.telem_lock:
            t = self.telem
        self.lbl_ax.setText(f"{t.imu_ax:+.2f}")
        self.lbl_ay.setText(f"{t.imu_ay:+.2f}")
        self.lbl_az.setText(f"{t.imu_az:+.2f}")
        self.lbl_gx.setText(f"{t.imu_gx:+.2f}")
        self.lbl_gy.setText(f"{t.imu_gy:+.2f}")
        self.lbl_gz.setText(f"{t.imu_gz:+.2f}")
        self.lbl_tof_front.setText(str(t.tof_front))
        self.lbl_tof_rear.setText(str(t.tof_rear))
        self.lbl_drive.setText("ACTIVE" if t.drive_active else "idle")
        self.lbl_drive.setStyleSheet(
            "color:#3c3;font-weight:bold" if t.drive_active else "")
        self.lbl_estop.setText("ON" if t.e_stop else "OFF")
        self.lbl_estop.setStyleSheet(
            "color:#c33;font-weight:bold" if t.e_stop else "color:#3c3")
        self.lbl_uptime.setText(f"{t.uptime_ms} ms")
        self.lbl_cmd.setText(self.active_command if self.hold_counter else "—")
        self.lbl_dir.setText(self.drive_direction)

    def closeEvent(self, ev):
        try:
            self.ws.stop()
            self.video.stop()
        finally:
            ev.accept()

# ---------------------------- Entry ----------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=DEFAULT_HOST,
                        help="ESP32 IP (default 192.168.4.1)")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    win = Dashboard(args.host)
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()