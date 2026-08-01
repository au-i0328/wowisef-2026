"""
Climbing Robot Dashboard v2 (Pi bridge edition)
================================================
Mac-side dashboard for the Arduino + Pi Zero 2 W bridge.

Reads from the Pi's WebSocket bus (ws://192.168.4.1:81/bus) and shows:
  - live MJPEG video pulled from the Pi /stream (port 8080)
  - IMU + TOF + bar pose + drive speed + drive direction
  - "Last ACK" command (whichever the Arduino echoed back last - not
    whatever the gamepad client requested, per the v2 spec)
  - recording + CSV/JSONL logging

The dashboard is read-only: control comes from the separate
gamepad_to_pi.py. Manual buttons in the UI still send friendly
{"kind":"command","payload":"..."} frames for ad-hoc testing.

Run:
    pip install pyqt5 opencv-python numpy websockets
    python3 mac_dashboard.py
    python3 mac_dashboard.py --host 192.168.4.1
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from threading import Lock, Thread
from typing import Optional

import cv2
import numpy as np
import websockets
from PyQt5 import QtCore, QtGui, QtWidgets

# ---------------------------- Configuration ----------------------------
DEFAULT_HOST = "pi.local"
WS_PATH      = "/bus"
HTTP_STREAM  = f"http://{DEFAULT_HOST}:8080/stream"
LOG_DIR      = Path.home() / "Documents" / "ClimbingRobotLogs"
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
    tof_up: int = 0
    tof_down: int = 0
    drive_speed: int = 0
    direction: str = "FORWARD"
    bar_pose: str = "parallel"
    last_ack: str = "NONE"
    received_at: float = 0.0


# ---------------------------- Async WS client (subscribe-only) ----------------------------
class WsClient(QtCore.QObject):
    connected    = QtCore.pyqtSignal()
    disconnected = QtCore.pyqtSignal()
    telemetry    = QtCore.pyqtSignal(dict)
    ack_received = QtCore.pyqtSignal(str)
    log          = QtCore.pyqtSignal(str)

    def __init__(self, host: str, port: int = 81, path: str = WS_PATH):
        super().__init__()
        self.host = host
        self.port = port
        self.path = path
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[Thread] = None
        self._ws: Optional[object] = None
        self._send_queue: Optional[asyncio.Queue] = None
        self._stop = False
        self._connected = False

    def start(self):
        self._stop = False
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop = True
        if self._loop and self._send_queue:
            try:
                self._loop.call_soon_threadsafe(self._send_queue.put_nowait, "__STOP__")
            except Exception:
                pass

    def send(self, payload: str):
        if not self._loop or not self._send_queue:
            return
        try:
            self._loop.call_soon_threadsafe(self._send_queue.put_nowait, payload)
        except Exception:
            pass

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
                    self._connected = True
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
                self._connected = False
                self.disconnected.emit()
                self.log.emit(f"WS disconnect: {e}; retrying in {backoff:.1f}s")
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, 10.0)

    async def _recv_loop(self, ws):
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            kind = msg.get("kind", "telemetry")
            if kind == "ack":
                self.ack_received.emit(str(msg.get("cmd", "")))
            elif kind == "telemetry":
                self.telemetry.emit(msg)

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
    """Pulls MJPEG from the Pi, decodes, optionally writes mp4."""
    frame  = QtCore.pyqtSignal(np.ndarray)
    log    = QtCore.pyqtSignal(str)

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
        cap: Optional[cv2.VideoCapture] = None
        for try_url in (self.url,):
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
        self.csv_path   = self.session_dir / "telemetry.csv"
        self.jsonl_path = self.session_dir / "telemetry.jsonl"
        self.video_path = self.session_dir / "video.mp4"
        self.events_path = self.session_dir / "events.log"
        with open(self.csv_path, "w", newline="") as f:
            csv.writer(f).writerow([
                "host_t_ms",
                "imu_ax", "imu_ay", "imu_az",
                "imu_gx", "imu_gy", "imu_gz",
                "tof_up_mm", "tof_down_mm",
                "speed", "direction", "bar_pose", "last_ack",
            ])
        self._lock = Lock()

    def log_telemetry(self, t: Telemetry):
        with self._lock:
            with open(self.csv_path, "a", newline="") as f:
                csv.writer(f).writerow([
                    int(time.time() * 1000),
                    t.imu_ax, t.imu_ay, t.imu_az,
                    t.imu_gx, t.imu_gy, t.imu_gz,
                    t.tof_up, t.tof_down,
                    t.drive_speed, t.direction, t.bar_pose, t.last_ack,
                ])
            with open(self.jsonl_path, "a") as f:
                f.write(json.dumps({
                    "host_t_ms": int(time.time() * 1000),
                    "imu": {"ax": t.imu_ax, "ay": t.imu_ay, "az": t.imu_az,
                            "gx": t.imu_gx, "gy": t.imu_gy, "gz": t.imu_gz},
                    "tof": {"front": t.tof_up, "rear": t.tof_down},
                    "status": {
                        "speed": t.drive_speed,
                        "direction": t.direction,
                        "bar_pose": t.bar_pose,
                        "last_ack": t.last_ack,
                    },
                }) + "\n")

    def log_event(self, event: str):
        with self._lock:
            with open(self.events_path, "a") as f:
                f.write(f"{time.strftime('%H:%M:%S')} {event}\n")


# ---------------------------- Main window ----------------------------
class Dashboard(QtWidgets.QMainWindow):
    def __init__(self, host: str):
        super().__init__()
        self.host = host
        self.logger = SessionLogger(LOG_DIR)
        self.telem = Telemetry()
        self.telem_lock = Lock()
        self._connected = False
        self._recording = False

        self._setup_ui()
        self._setup_networking()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh_labels)
        self.timer.start(100)  # 10 Hz UI refresh

    # ---------- UI ----------
    def _setup_ui(self):
        self.setWindowTitle("Climbing Robot Dashboard (Pi Bridge)")
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
        self.rec_button.setStyleSheet(
            "QPushButton:checked{background:#c33;color:white;font-weight:bold}"
        )
        rec_row.addWidget(self.rec_button)
        self.session_label = QtWidgets.QLabel(f"Session: {self.logger.session_dir.name}")
        rec_row.addWidget(self.session_label, stretch=1)
        left_layout.addLayout(rec_row)
        self.rec_button.toggled.connect(self._toggle_recording)

        # ----- Right: telemetry + manual commands -----
        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        right.setMinimumWidth(440)

        conn_row = QtWidgets.QHBoxLayout()
        self.conn_indicator = QtWidgets.QLabel("● Disconnected")
        self.conn_indicator.setStyleSheet("color:#c33;font-weight:bold")
        conn_row.addWidget(self.conn_indicator)
        conn_row.addStretch()
        conn_row.addWidget(QtWidgets.QLabel(f"Host: {self.host}"))
        right_layout.addLayout(conn_row)

        # IMU
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

        # TOF
        tof_box = QtWidgets.QGroupBox("Time-of-Flight")
        tof_layout = QtWidgets.QFormLayout(tof_box)
        self.lbl_tof_up = QtWidgets.QLabel("----")
        self.lbl_tof_down  = QtWidgets.QLabel("----")
        for lbl in (self.lbl_tof_up, self.lbl_tof_down):
            lbl.setStyleSheet("font-family:Menlo,monospace;font-size:13px")
        tof_layout.addRow("Front (mm)", self.lbl_tof_up)
        tof_layout.addRow("Rear  (mm)", self.lbl_tof_down)
        right_layout.addWidget(tof_box)

        # Status (now with 4 new fields vs. v1)
        status_box = QtWidgets.QGroupBox("Status")
        status_layout = QtWidgets.QFormLayout(status_box)
        self.lbl_speed   = QtWidgets.QLabel("0")
        self.lbl_dir     = QtWidgets.QLabel("FORWARD")
        self.lbl_pose    = QtWidgets.QLabel("parallel")
        self.lbl_ack     = QtWidgets.QLabel("—")
        self.lbl_active  = QtWidgets.QLabel("idle")
        for lbl in (self.lbl_speed, self.lbl_dir, self.lbl_pose, self.lbl_ack, self.lbl_active):
            lbl.setStyleSheet("font-family:Menlo,monospace;font-size:13px")
        status_layout.addRow("Drive speed", self.lbl_speed)
        status_layout.addRow("Direction",   self.lbl_dir)
        status_layout.addRow("Bar pose",    self.lbl_pose)
        status_layout.addRow("Last ACK",    self.lbl_ack)
        status_layout.addRow("State",       self.lbl_active)
        right_layout.addWidget(status_box)

        # Manual commands (kept; routes through WS bus)
        cmd_box = QtWidgets.QGroupBox("Manual Commands (test)")
        cmd_layout = QtWidgets.QGridLayout(cmd_box)
        manual_cmds = [
            ("up_attach",   0, 0), ("up_detach",   0, 1),
            ("down_attach", 1, 0), ("down_detach", 1, 1),
            ("both_attach", 2, 0), ("both_detach", 2, 1),
        ]
        for name, r, c in manual_cmds:
            b = QtWidgets.QPushButton(name)
            b.clicked.connect(lambda _, n=name: self._send_manual(n))
            cmd_layout.addWidget(b, r, c)
        estop = QtWidgets.QPushButton("EMERGENCY STOP")
        estop.setStyleSheet("background:#c33;color:white;font-weight:bold;padding:6px")
        estop.clicked.connect(lambda: self._send_manual("estop"))
        cmd_layout.addWidget(estop, 3, 0, 1, 2)
        right_layout.addWidget(cmd_box)

        right_layout.addStretch()

        # Log
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(200)
        self.log_view.setStyleSheet(
            "font-family:Menlo,monospace;font-size:11px;background:#111;color:#aaa"
        )
        right_layout.addWidget(self.log_view, stretch=1)

        layout.addWidget(left, stretch=3)
        layout.addWidget(right, stretch=2)

        self._add_log("Dashboard ready.")
        self._add_log(f"Session logs in: {self.logger.session_dir}")

    def _add_log(self, msg: str):
        self.log_view.appendPlainText(f"{time.strftime('%H:%M:%S')}  {msg}")

    # ---------- Networking ----------
    def _setup_networking(self):
        self.ws = WsClient(self.host)
        self.ws.connected.connect(self._on_ws_connected)
        self.ws.disconnected.connect(self._on_ws_disconnected)
        self.ws.telemetry.connect(self._on_telemetry)
        self.ws.ack_received.connect(self._on_ack)
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
        with self.telem_lock:
            self.telem.imu_ax      = imu.get("ax", 0)
            self.telem.imu_ay      = imu.get("ay", 0)
            self.telem.imu_az      = imu.get("az", 0)
            self.telem.imu_gx      = imu.get("gx", 0)
            self.telem.imu_gy      = imu.get("gy", 0)
            self.telem.imu_gz      = imu.get("gz", 0)
            self.telem.tof_up   = tof.get("front", 0)
            self.telem.tof_down    = tof.get("rear",  0)
            self.telem.drive_speed = int(msg.get("speed", 0))
            self.telem.direction   = str(msg.get("dir", "FORWARD"))
            self.telem.bar_pose    = str(msg.get("pose", "parallel"))
            self.telem.last_ack    = str(msg.get("ack", "NONE"))
            self.telem.received_at = time.time()
        self.logger.log_telemetry(self.telem)

    def _on_ack(self, cmd: str):
        with self.telem_lock:
            self.telem.last_ack = cmd
        self.logger.log_event(f"ack={cmd}")

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

    # ---------- Manual commands ----------
    def _send_manual(self, cmd_name: str):
        """Send a single-shot command. The Arduino will exec + ACK."""
        if cmd_name == "estop":
            payload = json.dumps({"kind": "command", "payload": "0,FORWARD,estop"})
        else:
            payload = json.dumps({
                "kind": "command",
                "payload": f"0,FORWARD,{cmd_name}",
            })
        self.ws.send(payload)
        self._add_log(f"Manual cmd sent: {cmd_name}")
        self.logger.log_event(f"manual_cmd={cmd_name}")

    # ---------- Refresh ----------
    def _refresh_labels(self):
        with self.telem_lock:
            t = self.telem
        self.lbl_ax.setText(f"{t.imu_ax:+.2f}")
        self.lbl_ay.setText(f"{t.imu_ay:+.2f}")
        self.lbl_az.setText(f"{t.imu_az:+.2f}")
        self.lbl_gx.setText(f"{t.imu_gx:+.2f}")
        self.lbl_gy.setText(f"{t.imu_gy:+.2f}")
        self.lbl_gz.setText(f"{t.imu_gz:+.2f}")
        self.lbl_tof_up.setText(str(t.tof_up))
        self.lbl_tof_down.setText(str(t.tof_down))
        self.lbl_speed.setText(str(t.drive_speed))
        self.lbl_dir.setText(t.direction)
        self.lbl_pose.setText(t.bar_pose)
        self.lbl_ack.setText(t.last_ack)
        self.lbl_active.setText("ACTIVE" if t.drive_speed > 0 else "idle")
        self.lbl_active.setStyleSheet(
            "color:#3c3;font-weight:bold" if t.drive_speed > 0 else ""
        )

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
                        help="Pi AP IP (default 192.168.4.1)")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    win = Dashboard(args.host)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
