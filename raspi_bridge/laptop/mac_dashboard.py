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

# TOF safety thresholds, in millimetres. A reading outside [min, max] raises
# the top-right warning banner. Defaults cover the normal operating range for
# the climbing robot: closer than 30 mm means the gripper has already hit
# the bar, farther than 300 mm means the sensor has lost the bar. Override
# at the command line:
#   python3 mac_dashboard.py --tof-up-min 40 --tof-down-max 280
TOF_UP_MIN_MM   = 30
TOF_UP_MAX_MM   = 300
TOF_DOWN_MIN_MM = 30
TOF_DOWN_MAX_MM = 300


# ---------------------------- Telemetry state ----------------------------
@dataclass
class Telemetry:
    tof_up: int = 0
    tof_down: int = 0
    drive_speed: int = 0
    direction: str = "FORWARD"
    bar_pose: str = "parallel"
    last_ack: str = "NONE"
    received_at: float = 0.0
    latched: bool = False
    latch_reason: str = ""


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
                "tof_up_mm", "tof_down_mm",
                "speed", "direction", "bar_pose", "last_ack", "latched",
            ])
        self._lock = Lock()

    def log_telemetry(self, t: Telemetry):
        with self._lock:
            with open(self.csv_path, "a", newline="") as f:
                csv.writer(f).writerow([
                    int(time.time() * 1000),
                    t.tof_up, t.tof_down,
                    t.drive_speed, t.direction, t.bar_pose, t.last_ack,
                    int(t.latched),
                ])
            with open(self.jsonl_path, "a") as f:
                f.write(json.dumps({
                    "host_t_ms": int(time.time() * 1000),
                    "tof": {"up": t.tof_up, "down": t.tof_down},
                    "status": {
                        "speed": t.drive_speed,
                        "direction": t.direction,
                        "bar_pose": t.bar_pose,
                        "last_ack": t.last_ack,
                        "latched": t.latched,
                    },
                }) + "\n")

    def log_event(self, event: str):
        with self._lock:
            with open(self.events_path, "a") as f:
                f.write(f"{time.strftime('%H:%M:%S')} {event}\n")


# ---------------------------- Main window ----------------------------
class Dashboard(QtWidgets.QMainWindow):
    def __init__(self, host: str,
                 tof_up_min: int = TOF_UP_MIN_MM,
                 tof_up_max: int = TOF_UP_MAX_MM,
                 tof_down_min: int = TOF_DOWN_MIN_MM,
                 tof_down_max: int = TOF_DOWN_MAX_MM):
        super().__init__()
        self.host = host
        self.tof_up_min   = tof_up_min
        self.tof_up_max   = tof_up_max
        self.tof_down_min = tof_down_min
        self.tof_down_max = tof_down_max
        self.logger = SessionLogger(LOG_DIR)
        self.telem = Telemetry()
        self.telem_lock = Lock()
        self._connected = False
        self._recording = False
        # Tracks which sensor is currently in violation so we only emit a
        # log entry on the rising and falling edges.
        self._warning_state: dict[str, str] = {"up": "ok", "down": "ok"}
        # The Arduino ships 0 mm before its first valid TOF sample; we treat
        # those as "no data yet" rather than a violation. Once we've seen
        # any non-zero reading from a sensor, 0 means a real out-of-range
        # event and the warning machinery kicks in normally.
        self._tof_seen: dict[str, bool] = {"up": False, "down": False}
        # Quiet period after the operator clicks "Run both_attach": the
        # servos move for ~800 ms (delay_to_pose on the Arduino) and the
        # TOF readings swing through out-of-range values as the gripper
        # re-positions. Suppress warning-state churn during that window so
        # the banner doesn't flicker between ok and bad mid-recovery.
        self._recovery_quiet_until: float = 0.0
        # When the operator clicks "Test warning", we want the dashboard
        # to stay in the warning state until they click 'Run both_attach',
        # even if real telemetry reports in-range values (because the
        # physical robot hasn't actually moved). Suppress threshold checks
        # entirely while this flag is set.
        self._test_warn_active: bool = False

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

        # ----- Top-right warning banner (overlays everything) -----
        self.warning_banner = QtWidgets.QFrame(central)
        self.warning_banner.setObjectName("warning_banner")
        self.warning_banner.setStyleSheet(
            "#warning_banner{background:#c33;color:white;border:2px solid #fff;"
            "border-radius:6px;padding:8px 12px;font-weight:bold}"
            "#warning_banner QLabel{color:white;background:transparent;"
            "border:none;font-family:Menlo,monospace;font-size:12px}"
        )
        self.warning_banner.setVisible(False)
        banner_layout = QtWidgets.QVBoxLayout(self.warning_banner)
        banner_layout.setContentsMargins(0, 0, 0, 0)
        self.warning_label = QtWidgets.QLabel("", self.warning_banner)
        self.warning_label.setAlignment(QtCore.Qt.AlignCenter)
        banner_layout.addWidget(self.warning_label)
        # Position in the top-right corner with a small margin. The banner
        # stays put when the window is resized via resizeEvent().
        self._banner_margin = 12
        self._reposition_banner()

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

        # TOF
        tof_box = QtWidgets.QGroupBox("Time-of-Flight")
        tof_layout = QtWidgets.QFormLayout(tof_box)
        self.lbl_tof_up = QtWidgets.QLabel("----")
        self.lbl_tof_down  = QtWidgets.QLabel("----")
        for lbl in (self.lbl_tof_up, self.lbl_tof_down):
            lbl.setStyleSheet("font-family:Menlo,monospace;font-size:13px")
        tof_layout.addRow("Up   (mm)", self.lbl_tof_up)
        tof_layout.addRow("Down (mm)", self.lbl_tof_down)
        # Show the active thresholds so the operator can see what the
        # banner will fire at without opening the source.
        self.lbl_tof_up_limits = QtWidgets.QLabel(
            f"warns outside [{self.tof_up_min}, {self.tof_up_max}] mm"
        )
        self.lbl_tof_down_limits = QtWidgets.QLabel(
            f"warns outside [{self.tof_down_min}, {self.tof_down_max}] mm"
        )
        for lbl in (self.lbl_tof_up_limits, self.lbl_tof_down_limits):
            lbl.setStyleSheet("color:#888;font-size:10px")
        tof_layout.addRow("", self.lbl_tof_up_limits)
        tof_layout.addRow("", self.lbl_tof_down_limits)
        right_layout.addWidget(tof_box)

        # Recovery prompt: shown only while at least one sensor is in
        # violation. One-click path back to a safe state via both_attach.
        self.warn_prompt = QtWidgets.QFrame(right)
        self.warn_prompt.setObjectName("warn_prompt")
        self.warn_prompt.setStyleSheet(
            "#warn_prompt{background:#fff3cd;color:#333;border:2px solid #c33;"
            "border-radius:6px;padding:10px}"
            "#warn_prompt QLabel{background:transparent;border:none;"
            "color:#333;font-family:Menlo,monospace;font-size:12px}"
            "#warn_prompt QPushButton{background:#3a3;color:white;font-weight:bold;"
            "padding:8px;border:none;border-radius:4px}"
            "#warn_prompt QPushButton:hover{background:#4c4}"
            "#warn_prompt QPushButton:pressed{background:#282}"
        )
        self.warn_prompt.setVisible(False)
        prompt_layout = QtWidgets.QVBoxLayout(self.warn_prompt)
        prompt_layout.setContentsMargins(0, 0, 0, 0)
        prompt_layout.setSpacing(6)
        self.warn_prompt_label = QtWidgets.QLabel(
            "⚠  TOF out of range — Arduino has stopped.\n"
            "Run both_attach to recover."
        )
        self.warn_prompt_label.setAlignment(QtCore.Qt.AlignCenter)
        prompt_layout.addWidget(self.warn_prompt_label)
        self.warn_prompt_button = QtWidgets.QPushButton("Run both_attach")
        self.warn_prompt_button.setCursor(QtCore.Qt.PointingHandCursor)
        self.warn_prompt_button.clicked.connect(self._on_run_both_attach)
        prompt_layout.addWidget(self.warn_prompt_button)
        right_layout.addWidget(self.warn_prompt)

        # Status (now with 4 new fields vs. v1)
        status_box = QtWidgets.QGroupBox("Status")
        status_layout = QtWidgets.QFormLayout(status_box)
        self.lbl_speed   = QtWidgets.QLabel("0")
        self.lbl_dir     = QtWidgets.QLabel("FORWARD")
        self.lbl_pose    = QtWidgets.QLabel("parallel")
        self.lbl_ack     = QtWidgets.QLabel("—")
        self.lbl_active  = QtWidgets.QLabel("idle")
        self.lbl_latched = QtWidgets.QLabel("released")
        for lbl in (self.lbl_speed, self.lbl_dir, self.lbl_pose, self.lbl_ack,
                    self.lbl_active, self.lbl_latched):
            lbl.setStyleSheet("font-family:Menlo,monospace;font-size:13px")
        status_layout.addRow("Drive speed", self.lbl_speed)
        status_layout.addRow("Direction",   self.lbl_dir)
        status_layout.addRow("Bar pose",    self.lbl_pose)
        status_layout.addRow("Last ACK",    self.lbl_ack)
        status_layout.addRow("State",       self.lbl_active)
        status_layout.addRow("Safety",      self.lbl_latched)
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
        # Self-test: triggers the full WARN pipeline (dashboard -> bridge
        # -> Arduino latch) without touching the physical TOF sensors.
        # Useful for verifying the recovery flow on a fresh setup.
        self.test_warn_button = QtWidgets.QPushButton("Test warning (WARN:up)")
        self.test_warn_button.setStyleSheet(
            "background:#e8a000;color:#222;font-weight:bold;padding:6px"
        )
        self.test_warn_button.setToolTip(
            "Sends WARN:up to the bridge. The Arduino should latch, the\n"
            "banner + prompt should appear, and clicking 'Run both_attach'\n"
            "should clear the latch and return the dashboard to normal."
        )
        self.test_warn_button.clicked.connect(self._on_test_warning)
        cmd_layout.addWidget(self.test_warn_button, 4, 0, 1, 2)
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

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        # Keep the warning banner pinned to the top-right corner.
        self._reposition_banner()

    def _reposition_banner(self):
        bw = self.warning_banner.sizeHint().width()
        bh = self.warning_banner.sizeHint().height()
        m = self._banner_margin
        self.warning_banner.setGeometry(
            self.width() - bw - m, m, bw, bh
        )

    def _set_warning(self, sensor: str, state: str, value: int):
        """sensor: 'up' or 'down'; state: 'ok' / 'low' / 'high'."""
        prev = self._warning_state.get(sensor, "ok")
        if state == prev:
            return
        self._warning_state[sensor] = state
        self._refresh_banner()
        self._refresh_warn_prompt()
        # Log edge transitions only -- never per-frame.
        if state == "ok":
            self._add_log(f"TOF {sensor} OK ({value} mm)")
            self.logger.log_event(f"tof_{sensor}_ok={value}")
        else:
            self._add_log(
                f"TOF {sensor} {state.upper()} ({value} mm) "
                f"outside [{self.tof_up_min if sensor == 'up' else self.tof_down_min}, "
                f"{self.tof_up_max if sensor == 'up' else self.tof_down_max}] mm"
            )
            self.logger.log_event(f"tof_{sensor}_{state}={value}")
            # Tell the Arduino to latch. The Pi bridge forwards this as a
            # raw "WARN:<sensor>\n" line.
            self.ws.send(f"WARN:{sensor}")
            self._add_log(f"Sent WARN:{sensor} to Arduino (latch engaged)")

    def _refresh_banner(self):
        states = self._warning_state
        violators = [s for s in ("up", "down") if states[s] != "ok"]
        if not violators:
            self.warning_banner.setVisible(False)
            self.warning_label.setText("")
            return
        # If the Arduino told us why it's latched, prefer that reason so
        # the operator sees "EMERGENCY STOP" for estops and the sensor
        # details for genuine TOF violations.
        reason = (self.telem.latch_reason
                  if hasattr(self, "telem") else "")
        if reason == "estop":
            text = "EMERGENCY STOP"
        else:
            parts = [f"TOF {s.upper()} {states[s].upper()}" for s in violators]
            text = "  •  ".join(parts)
        self.warning_label.setText("  ⚠  " + text + "  ⚠  ")
        self.warning_banner.setVisible(True)
        self.warning_banner.raise_()
        # Re-position in case the label grew and the sizeHint changed.
        self._reposition_banner()

    def _refresh_warn_prompt(self):
        states = self._warning_state
        violators = [s for s in ("up", "down") if states[s] != "ok"]
        self.warn_prompt.setVisible(bool(violators))
        if not violators:
            return
        sensor_list = ", ".join(s.upper() for s in violators)
        # If the Arduino told us why it's latched, surface that reason so
        # the operator knows whether it's a TOF violation or an estop.
        reason = (self.telem.latch_reason
                  if hasattr(self, "telem") else "")
        if reason in ("up", "down"):
            head = f"⚠  TOF {reason.upper()} out of range — Arduino has stopped.\n"
        elif reason == "estop":
            head = "⚠  EMERGENCY STOP — Arduino has stopped.\n"
        elif violators:
            head = f"⚠  TOF {sensor_list} out of range — Arduino has stopped.\n"
        else:
            head = "⚠  Arduino has stopped.\n"
        self.warn_prompt_label.setText(
            head + "Run both_attach to recover."
        )

    def _on_run_both_attach(self):
        # This is the recovery path: send both_attach through the WS bus,
        # which (a) executes the bar-pose command on the Arduino, and (b)
        # is the first non-NONE command the latched Arduino will see, so
        # it also clears the latch.
        self._recovery_quiet_until = time.monotonic() + 1.5  # cover delay_to_pose
        # Releasing the test-warning latch: real telemetry takes over
        # threshold checks again on the next frame.
        self._test_warn_active = False
        self._send_manual("both_attach")
        self._add_log("Recovery: requested both_attach; latch should clear "
                      "on the next Arduino status frame.")
        self.logger.log_event("recovery_both_attach_requested")

    def _on_test_warning(self):
        # Self-test for the full WARN pipeline. Drives the dashboard's
        # warning state machine into a forced-low state for the up
        # sensor. The rising edge inside _set_warning sends WARN:up to
        # the bridge, the Arduino latches for real, and the next status
        # JSON confirms with latched=true.
        # If the warning is already active, do nothing -- clicking the
        # button repeatedly would just spam the bridge.
        if self._warning_state.get("up", "ok") != "ok":
            self._add_log("Test warning: already active; click 'Run "
                          "both_attach' to clear first.")
            return
        # Use a fabricated value one below the configured minimum so the
        # log entry on the rising edge reads as a real low reading.
        simulated = self.tof_up_min - 1
        # Mark the sensor as "seen" so the bootstrap-suppression path
        # doesn't neutralize our synthetic value.
        self._tof_seen["up"] = True
        # Freeze the threshold check until the operator recovers --
        # otherwise the next real telemetry frame could immediately undo
        # the synthetic warning state.
        self._test_warn_active = True
        self._set_warning("up", "low", simulated)
        self._add_log("Test warning: forced TOF up into 'low' state; "
                      "WARN:up sent to bridge. Click 'Run both_attach' "
                      "to recover.")
        self.logger.log_event("test_warning_triggered")

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
        tof = msg.get("tof", {})
        with self.telem_lock:
            self.telem.tof_up   = tof.get("up", 0)
            self.telem.tof_down = tof.get("down", 0)
            self.telem.drive_speed = int(msg.get("speed", 0))
            self.telem.direction   = str(msg.get("dir", "FORWARD"))
            self.telem.bar_pose    = str(msg.get("pose", "parallel"))
            self.telem.last_ack    = str(msg.get("ack", "NONE"))
            self.telem.received_at = time.time()
            self.telem.latched     = bool(msg.get("latched", False))
            self.telem.latch_reason = str(msg.get("latch_reason", "")) if self.telem.latched else ""
        self.logger.log_telemetry(self.telem)
        if not self.telem.latched:
            # When the Arduino confirms the latch has cleared (e.g. after
            # the user clicks "Run both_attach"), force our local warning
            # state to OK so the banner / prompt collapse immediately even
            # if the TOF reading hasn't yet settled back into range.
            for sensor in ("up", "down"):
                if self._warning_state.get(sensor, "ok") != "ok":
                    self._set_warning(sensor, "ok",
                                      self.telem.tof_up if sensor == "up" else self.telem.tof_down)
        else:
            # Arduino-driven latch (e.g. estop, or a WARN: that the
            # dashboard missed). Make sure the UI shows the warning
            # state, but don't echo WARN: back to the bridge -- the
            # Arduino already knows.
            reason = self.telem.latch_reason or "unknown"
            if self._warning_state.get("up", "ok") == "ok":
                self._warning_state["up"] = "low"
                self._refresh_banner()
                self._refresh_warn_prompt()
                self._add_log(f"Arduino latched (reason: {reason}). "
                              "Click 'Run both_attach' to recover.")
                self.logger.log_event(f"arduino_latched={reason}")
        # Check TOF thresholds outside the lock so we don't hold it while
        # fiddling with widgets. Edge-triggered: only logs on the transition.
        self._check_tof_thresholds(self.telem.tof_up, self.telem.tof_down)

    def _check_tof_thresholds(self, up_mm: int, down_mm: int):
        # While a test warning is in flight, freeze the dashboard's
        # warning state. The operator has explicitly asked the dashboard
        # to show the warning; we shouldn't undo that just because real
        # telemetry happens to be in range.
        if self._test_warn_active:
            return
        # During the recovery grace window, leave the warning state alone.
        # The servos are mid-motion and TOF readings swing through bogus
        # values; we'd otherwise flicker the banner through ok/low/ok/low.
        if time.monotonic() < self._recovery_quiet_until:
            return
        # Mark sensors as "have we ever seen a non-zero reading?" Once true
        # we trust that 0 is a real low reading rather than the Arduino's
        # "not yet sampled" placeholder.
        if up_mm > 0:
            self._tof_seen["up"] = True
        if down_mm > 0:
            self._tof_seen["down"] = True

        def state_for(sensor: str, value: int, lo: int, hi: int) -> str:
            if value == 0 and not self._tof_seen[sensor]:
                return "ok"  # No data yet — suppress the bootstrap flash.
            if value < lo:
                return "low"
            if value > hi:
                return "high"
            return "ok"

        self._set_warning("up",   state_for("up",   up_mm,   self.tof_up_min,   self.tof_up_max),   up_mm)
        self._set_warning("down", state_for("down", down_mm, self.tof_down_min, self.tof_down_max), down_mm)

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
        # Optimistically reflect estop on the UI before the Arduino's
        # next status JSON arrives (up to 200 ms gap). The status JSON
        # will confirm with latched:true, latch_reason:"estop".
        if cmd_name == "estop":
            if self._warning_state.get("up", "ok") == "ok":
                self._warning_state["up"] = "low"
                # Surface the latch_reason locally so the banner and
                # prompt render with the estop-specific copy on the very
                # next refresh tick instead of waiting on telemetry.
                self.telem.latched = True
                self.telem.latch_reason = "estop"
                self._refresh_banner()
                self._refresh_warn_prompt()
                self._add_log("E-stop sent: Arduino will latch.")
                self.logger.log_event("estop_sent")

    # ---------- Refresh ----------
    def _refresh_labels(self):
        with self.telem_lock:
            t = self.telem
        red = "color:#c33;font-weight:bold"
        up_state = self._warning_state.get("up", "ok")
        down_state = self._warning_state.get("down", "ok")
        self.lbl_tof_up.setText(str(t.tof_up))
        self.lbl_tof_up.setStyleSheet(
            f"font-family:Menlo,monospace;font-size:13px;{red}"
            if up_state != "ok" else
            "font-family:Menlo,monospace;font-size:13px"
        )
        self.lbl_tof_down.setText(str(t.tof_down))
        self.lbl_tof_down.setStyleSheet(
            f"font-family:Menlo,monospace;font-size:13px;{red}"
            if down_state != "ok" else
            "font-family:Menlo,monospace;font-size:13px"
        )
        self.lbl_speed.setText(str(t.drive_speed))
        self.lbl_dir.setText(t.direction)
        self.lbl_pose.setText(t.bar_pose)
        self.lbl_ack.setText(t.last_ack)
        self.lbl_active.setText("ACTIVE" if t.drive_speed > 0 else "idle")
        self.lbl_active.setStyleSheet(
            "color:#3c3;font-weight:bold" if t.drive_speed > 0 else ""
        )
        self.lbl_latched.setText("LATCHED" if t.latched else "released")
        self.lbl_latched.setStyleSheet(
            "font-family:Menlo,monospace;font-size:13px;color:#c33;font-weight:bold"
            if t.latched else
            "font-family:Menlo,monospace;font-size:13px;color:#3c3"
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
    parser.add_argument("--tof-up-min", type=int, default=TOF_UP_MIN_MM,
                        help="TOF up sensor lower limit in mm (default %(default)s)")
    parser.add_argument("--tof-up-max", type=int, default=TOF_UP_MAX_MM,
                        help="TOF up sensor upper limit in mm (default %(default)s)")
    parser.add_argument("--tof-down-min", type=int, default=TOF_DOWN_MIN_MM,
                        help="TOF down sensor lower limit in mm (default %(default)s)")
    parser.add_argument("--tof-down-max", type=int, default=TOF_DOWN_MAX_MM,
                        help="TOF down sensor upper limit in mm (default %(default)s)")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    win = Dashboard(
        args.host,
        tof_up_min=args.tof_up_min,
        tof_up_max=args.tof_up_max,
        tof_down_min=args.tof_down_min,
        tof_down_max=args.tof_down_max,
    )
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
