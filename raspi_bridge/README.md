# Climbing Robot v2 — Arduino + Pi Zero 2 W Bridge

A three-tier pipeline built on top of the existing sketch_jul30.ino
motor/servo code and the gamepad_to_arduino.py controller mapping.

```
PS4 controller  --USB/BT-->  MacBook  --WiFi-->  Pi Zero 2 W  --USB-->  Arduino UNO
                                |                   |                     |
                                |                   |                     +- PCA9685 servos
                                |                   |                     +- L298NX2 drive
                                |                   |                     +- MPU6050 IMU
                                |                   |                     +- 2x VL53L0X TOF
                                |                   |
                                |                   +- HTTP /stream (MJPEG)
                                |                   +- WS  /bus (control + telemetry)
                                |
                                +- gamepad_to_pi.py   (publishes commands)
                                +- mac_dashboard.py   (subscribes)
```

## Files

| File | Where it runs | Purpose |
|------|---------------|---------|
| `arduino/arduino_bridge.ino` | Arduino UNO | Motor/servo control + IMU + TOF + 200ms status JSON |
| `pi/pi_serial_bridge.py`     | Pi Zero 2 W | USB-serial <-> WebSocket bus relay |
| `pi/pi_video_stream.py`      | Pi Zero 2 W | USB webcam -> MJPEG HTTP stream |
| `pi/pi_ap_setup.sh`          | Pi Zero 2 W | hostapd + dnsmasq access-point setup |
| `pi/pi_install.sh`           | Pi Zero 2 W | One-shot installer + systemd units |
| `laptop/gamepad_to_pi.py`    | MacBook | PS4 controller -> WS commands |
| `laptop/mac_dashboard.py`    | MacBook | PyQt5 dashboard + video recording + logs |

## Network topology

| Property | Value |
|----------|-------|
| AP SSID | `ClimbingRobot` |
| Password | `climb12345` |
| AP IP | `192.168.4.1` |
| mDNS | `climbing-robot.local` (if avahi enabled) |
| WS bus | `ws://192.168.4.1:81/bus` |
| MJPEG  | `http://192.168.4.1:8080/stream` |

## Wire plant (Arduino UNO)

The Arduino sketch retains every pin from the existing sketch_jul30.ino.
Additions for v2:

| Component | Arduino pin | Notes |
|-----------|-------------|-------|
| MPU6050 (IMU) | I2C (SDA=A4, SCL=A5) | AD0 -> GND (default 0x68) |
| VL53L0X #1 (front) | XSHUT -> A1, I2C | address 0x30 after re-init |
| VL53L0X #2 (rear)  | XSHUT -> A2, I2C | address 0x31 after re-init |
| L298NX2 drive | 6,13,12,5,8,7 | unchanged |
| PCA9685 (servos) | I2C | address 0x40, unchanged |
| Status LED | A0 | unchanged |

XSHUT wiring is required so the two VL53L0X sensors can be re-addressed
sequentially on the same bus. The sketch drives:
1. rear_XSHUT LOW, front_XSHUT HIGH, init front -> setAddress(0x30)
2. rear_XSHUT HIGH, init rear -> setAddress(0x31)

## Serial protocol

Arduino -> Pi (line oriented, ASCII, 115200):

```
STS:{...json...}\n        # 200 ms cadence
ACK:<cmd>\n              # one-shot, when a command is executed
```

Status JSON shape (one per 200 ms):

```json
{
  "speed": 150,
  "dir": "FORWARD",
  "pose": "parallel",
  "ack": "up_attach",
  "imu": {"ax": 0.0, "ay": 0.0, "az": 9.81, "gx": 0.0, "gy": 0.0, "gz": 0.0},
  "tof": {"front": 350, "rear": 120}
}
```

Pi -> Arduino (CSV, newline terminated):

```
<speed>,<direction>,<cmd>\n
```

`direction` is `FORWARD` or `BACKWARD`. `cmd` is one of `NONE`,
`up_attach`, `up_detach`, `down_attach`, `down_detach`, `both_attach`,
`both_detach`, `estop`.

## WebSocket protocol

`ws://192.168.4.1:81/bus` is a shared bus. Any client can connect and:

- Receive every outgoing frame (multiplexed broadcast).
- Send inbound commands. Two frame shapes are accepted:
  - `{"kind":"command","payload":"150,FORWARD,NONE"}`
  - Raw CSV: `150,FORWARD,NONE`

Outgoing message kinds:

```json
{ "kind": "telemetry", "speed": 150, "dir": "FORWARD", "pose": "parallel",
  "ack": "up_attach", "imu": {...}, "tof": {...} }
{ "kind": "ack", "cmd": "up_attach" }
```

## Install (Pi)

1. Flash the Pi with Raspberry Pi OS Lite, enable SSH + WiFi country.
2. Copy the bundle to the Pi:

   ```bash
   scp -r raspi_bridge/pi/ pi@<pi-ip>:~/
   ```

3. SSH in and run the installer:

   ```bash
   sudo bash pi_install.sh
   ```

   This installs `hostapd`, `dnsmasq`, `avahi`, Python venv with
   `websockets`, `opencv-python-headless`, `pyserial`, and enables
   two systemd services:

   - `serial-bridge.service` -> `pi_serial_bridge.py`
   - `video-stream.service`  -> `pi_video_stream.py`

4. Configure the AP:

   ```bash
   sudo bash pi_ap_setup.sh
   sudo reboot
   ```

5. Confirm the AP is alive:

   ```bash
   ip addr show wlan1          # 192.168.4.1/24
   sudo systemctl status hostapd serial-bridge video-stream
   curl http://192.168.4.1:8080/health   # -> OK
   ```

## Run (MacBook)

1. Join the `ClimbingRobot` WiFi network (password `climb12345`).
2. Install Python deps:

   ```bash
   pip install pyqt5 opencv-python numpy websockets pygame
   ```

3. In two terminals:

   ```bash
   python3 gamepad_to_pi.py           # sends commands
   python3 mac_dashboard.py           # live UI + recording
   ```

4. Optional browser check: open `http://192.168.4.1:8080/stream` to see
   the live video feed.

## Test mode

All three scripts have a `--test` flag, so you can develop on hardware
that isn't fully wired up:

```bash
# Pi side, no Arduino + no webcam
python3 pi_serial_bridge.py --test
python3 pi_video_stream.py --test

# Mac side, no controller
python3 gamepad_to_pi.py --test
```

In test mode the serial bridge emits a synthetic STS every 200 ms and the
video streamer draws a moving gradient. The gamepad sender oscillates
drive speed for free.

## Recording & logs

The dashboard stores per-session:

```
~/Documents/ClimbingRobotLogs/session_YYYYMMDD_HHMMSS/
    video.mp4
    telemetry.csv
    telemetry.jsonl
    events.log
```

`telemetry.csv` adds four columns over v1: `speed`, `direction`,
`bar_pose`, `last_ack`. The `Last ACK` UI label is sourced from the
most recent ACK frame received from the Arduino, not from the gamepad
sender - so the UI matches what the Arduino actually executed.

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `hostapd` won't start | `sudo journalctl -u hostapd`. Often the wifi adapter doesn't support AP mode (Realtek chips need extra firmware). Try a different USB adapter. |
| No `/dev/ttyACM0` | Check `ls /dev/serial/by-id`. Try a different USB cable (some are power-only). |
| `permission denied` on serial | `sudo usermod -aG dialout $USER` and re-login. |
| Dashboard says "Video: failed to open stream" | Verify the Pi is on the same AP and the stream is reachable: `curl http://192.168.4.1:8080/health` from the Mac. |
| WS keeps disconnecting | AP channel conflict with another router. Change `AP_CHANNEL` in `pi_ap_setup.sh` and re-run. |
| `mp4v` recording won't play in QuickTime | Switch fourcc to `avc1` in `laptop/mac_dashboard.py` `VideoWorker._run`, or remux with `ffmpeg -i video.mp4 -c:v copy video_remux.mp4`. |
| Arduino IMU/TOF not detected | `I2C scanner` sketch should show 0x68, 0x30, 0x31. If not, check XSHUT wiring and `Wire.begin()` runs before `setup()` calls. |
| Gamepad not detected | Plug the controller in **before** running the script. On macOS grant Input Monitoring permission (System Settings -> Privacy & Security). |
| `STS:` JSON parse error on Pi | Probably a line got truncated. Lower the baud rate or check the Arduino's `Serial.println` buffer behavior. |

## Optional: H.264 fancier recording

The current pipeline records an mp4 from the MJPEG stream on the laptop
(so the video is encoded as `mp4v`). If you want a more efficient H.264
output, run an `ffmpeg` pipe on the Pi instead of the MJPEG HTTP route:

```bash
# On the Pi (assumes the MJPEG endpoint already exists)
ffmpeg -i http://127.0.0.1:8080/stream -c:v libx264 -preset veryfast \
       -f hls -hls_time 1 -hls_list_size 5 /tmp/stream.m3u8
```

Then point the dashboard's `HTTP_STREAM` constant at
`http://192.168.4.1:8080/stream.m3u8` (or the `.mp4` muxed variant
ffmpeg produces). This is left as an opt-in swap.
