# Climbing Robot — Wireless AP Dashboard

The ESP32 now creates its own WiFi access point and runs a small HTTP/WS
server. A native-feeling PyQt5 app on macOS connects to it for live
telemetry, gamepad control, and on-disk recording.

## Files

| File | Purpose |
|------|---------|
| `esp32_robot.ino` | ESP32 firmware — AP mode, HTTP, WebSocket, MJPEG stream |
| `mac_dashboard.py` | macOS app — PyQt5 UI, video recording, CSV/JSONL logs |
| `gamepad_to_esp32.py` | Headless PC sender (no UI) for testing the WS protocol |

## Network Topology

```
┌─────────────────────┐                   ┌──────────────────────┐
│  ESP32              │                   │  MacBook             │
│  SSID ClimbingRobot │  192.168.4.1 ───► │  (joins the AP)      │
│  ws://:81/          │                   │  mac_dashboard.py    │
│  http://:80/stream  │                   │                      │
└─────────────────────┘                   └──────────────────────┘
```

## ESP32 Setup

Arduino IDE board settings (ESP32-DevKitC with camera):

```
Board             : ESP32 Dev Module
Partition Scheme  : Huge App (3MB No OTA)
Flash Frequency   : 80 MHz
Upload Speed      : 921600
PSRAM             : Enabled (AI-Thinker models)
```

Required Arduino libraries (install via Library Manager):

```
WiFi                 (built-in)
WebServer            (built-in)
WebSocketsServer     (by Markus Sattler)
ESPmDNS              (built-in)
Wire, SPI            (built-in)
Adafruit PWMServoDriver Library
Adafruit MPU6050 Library
Adafruit Unified Sensor
VL53L0X              (Pololu)
ArduinoJson          (>= 7.0.0)
esp_camera           (part of ESP32 board package)
```

Flashing an ESP32 with a camera often needs the **AI-Thinker ESP32-CAM**
board variant or generic "ESP32 Dev Module" with the camera pins defined
in the sketch. Pin block at the top of the sketch matches the AI-Thinker
model — change it if you use a different board.

Once flashed and powered:

1. The ESP32 creates `ClimbingRobot` WiFi with password `climb12345`.
2. mDNS is `http://climbing-robot.local/` (some routers strip this; use IP if so).
3. Direct IP fallback: `http://192.168.4.1/`.

Useful URLs (open these in a browser to verify the ESP32 is alive):

| URL | Returns |
|-----|---------|
| `http://192.168.4.1/` | Tiny HTML page with embedded video |
| `http://192.168.4.1/stream` | MJPEG video |
| `http://192.168.4.1/snapshot` | Single JPEG |
| `http://192.168.4.1/snapshot.json` | Latest telemetry JSON |

## Mac Dashboard

### Install dependencies

```bash
pip install pyqt5 opencv-python numpy websockets pygame
```

### Connect and run

1. Join your MacBook to the `ClimbingRobot` WiFi network.
2. Run:

```bash
python3 mac_dashboard.py
# or with a different IP:
python3 mac_dashboard.py --host 192.168.4.1
```

### What the dashboard shows

- Live MJPEG video in the left panel
- IMU (accel + gyro) readouts
- TOF front / rear distances in mm
- Status (drive active, e-stop, ESP uptime, last command)
- Manual command buttons (works without a gamepad)
- Log view
- Connection indicator (green = WS connected)

### Recording

Click **● Record** in the video panel to start writing
`~/Documents/ClimbingRobotLogs/<session>/video.mp4` from the live MJPEG
stream. Click again to stop. The button is toggleable and shows
**■ Recording** while on.

### Logs

Each dashboard session creates:

```
~/Documents/ClimbingRobotLogs/session_YYYYMMDD_HHMMSS/
    video.mp4           (H.264 mp4, only when recording)
    telemetry.csv       (one row per telemetry tick)
    telemetry.jsonl     (full JSON, one per line)
    events.log          (manual commands, record on/off)
```

## WebSocket Protocol

The ESP32 WS server lives at `ws://192.168.4.1:81/`. Messages are text:

- **PC → ESP32** (CSV, newline-terminated): `<speed>,<direction>,<command>`
- **ESP32 → PC** (JSON, every 1 s):

```json
{
  "imu":{"ax":0.1,"ay":0.0,"az":9.81,"gx":0.0,"gy":0.0,"gz":0.0},
  "tof":{"front":350,"rear":120},
  "status":{"drive_active":false,"e_stop":false,"uptime_ms":12345}
}
```

## Gamepad Mapping (same as `gamepad_to_esp32.py`)

| Button | Command |
|--------|---------|
| Cross (X) | down_detach |
| Circle (O) | down_attach |
| Square (□) | up_attach |
| Triangle (△) | up_detach |
| Share | both_attach |
| Options | both_detach |
| PS button | estop (latched) |
| L1 / R1 | direction toggle |
| D-pad up/down | fixed-speed forward/back |

The dashboard works with **no gamepad** — use the manual command
buttons. Plug in a PS4 controller via Bluetooth/USB for wireless control.

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| ESP32 doesn't show `ClimbingRobot` SSID | Verify `WiFi.softAP(...)` runs; check serial monitor for the IP print |
| mDNS doesn't resolve | Use the IP `192.168.4.1` directly |
| `ws://...` never connects | Confirm the Mac joined the AP; check that nothing is blocking port 81 |
| Video shows a still frame | Lower frame size to CIF in `initCamera()` (FRAMESIZE_VGA → FRAMESIZE_CIF) |
| `mp4v` doesn't play in QuickTime | Switch the fourcc in `VideoWorker._run()` to `avc1` and install `pyAV` |
| Gamepad not detected | Plug the controller in **before** starting the dashboard; on macOS grant Input Monitoring permission |

## Customizing

- AP password: change `AP_PASSWORD` in the sketch and reflash.
- Video quality / size: edit `cfg.frame_size` and `cfg.jpeg_quality` in `initCamera()`.
- Telemetry rate: change `if (millis() - last_telemetry_ms >= 1000)` in `loop()`.
- Log location: change `LOG_DIR` at the top of `mac_dashboard.py`.