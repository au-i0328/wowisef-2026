"""
ESP32 Climbing Robot Controller (WiFi)

Reads a PS4 controller over SDL2 (same way as the Arduino version) and
sends commands to the ESP32 over TCP. Sensor data (IMU + TOF) and the
camera MJPEG stream are read from the same TCP/HTTP connection.

Usage:
    python3 gamepad_to_esp32.py
    python3 gamepad_to_esp32.py --ip 192.168.1.42

The ESP32 sketch (esp32_robot.ino) listens on:
    TCP  3333 - command channel   (this script sends here)
    TCP  3334 - sensor data stream (this script reads here)
    HTTP 80   - camera MJPEG       (this script prints a URL to open in VLC/browser)
"""

import argparse
import socket
import sys
import time
from threading import Thread

import pygame
from pygame._sdl2 import controller

# --- Configuration ---
SEND_RATE = 30       # Hz (must match ESP32 loop rate)
HOLD_FRAMES = 10     # ~333 ms pulse duration at 30 Hz
DEFAULT_ESP_IP = "192.168.1.42"
CMD_PORT = 3333
DATA_PORT = 3334
VIDEO_PORT = 80


# ====================== Controller Helpers ======================
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


def just_pressed(channels, prev, idx):
    return channels[idx] == 1 and prev[idx] == 0


# ====================== Network ======================
class ESP32Link:
    def __init__(self, ip, cmd_port=CMD_PORT, data_port=DATA_PORT):
        self.ip = ip
        self.cmd_port = cmd_port
        self.data_port = data_port
        self.cmd_sock = None
        self.data_sock = None
        self.video_url = f"http://{ip}:{VIDEO_PORT}/stream"

    def connect_cmd(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2.0)
        s.connect((self.ip, self.cmd_port))
        s.settimeout(None)  # blocking send
        self.cmd_sock = s
        return s

    def connect_data(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2.0)
        s.connect((self.ip, self.data_port))
        s.settimeout(0.2)
        self.data_sock = s
        return s

    def send_command(self, payload: str):
        if not self.cmd_sock:
            return False
        try:
            self.cmd_sock.sendall(payload.encode("utf-8"))
            return True
        except OSError:
            return False

    def read_sensor_data(self):
        """Returns (line, None) for any data received, or (None, None) if no data."""
        if not self.data_sock:
            return None
        try:
            return self.data_sock.recv(4096).decode("utf-8", errors="ignore"), None
        except (socket.timeout, BlockingIOError):
            return None
        except OSError:
            return None

    def close(self):
        for s in (self.cmd_sock, self.data_sock):
            if s:
                try: s.close()
                except OSError: pass
        self.cmd_sock = None
        self.data_sock = None


def parse_imu_line(line):
    """Best-effort JSON parse for sensor data (graceful on failure)."""
    try:
        import json
        return json.loads(line)
    except Exception:
        return None


# ====================== Sensor Data Thread ======================
class SensorReader(Thread):
    def __init__(self, link: ESP32Link):
        super().__init__(daemon=True)
        self.link = link
        self.running = True
        self.latest = None

    def run(self):
        while self.running:
            data = self.link.read_sensor_data()
            if data:
                self.latest = data
            time.sleep(0.05)


# ====================== Main ======================
def main():
    parser = argparse.ArgumentParser(description="PS4 -> ESP32 over WiFi")
    parser.add_argument("--ip", default=DEFAULT_ESP_IP, help="ESP32 IP address")
    parser.add_argument("--simulate", action="store_true",
                        help="Skip network and just print commands")
    args = parser.parse_args()

    pygame.init()
    controller.init()
    pad = None
    for i in range(controller.get_count()):
        if controller.is_controller(i):
            pad = controller.Controller(i)
            pad.init()
            break
    if not pad:
        print("No PS4 controller detected.")
        sys.exit(1)
    print(f"Connected: {pad.name}")

    link = ESP32Link(args.ip) if not args.simulate else None
    if link:
        try:
            link.connect_cmd()
            link.connect_data()
            print(f"Connected to ESP32 @ {args.ip}")
            print(f"Video stream: {link.video_url}")
        except OSError as e:
            print(f"Connection failed: {e}. Falling back to --simulate mode.")
            link = None
    if not link:
        print("Running in simulate mode (commands printed, not sent).")

    sensor = SensorReader(link) if link else None
    if sensor: sensor.start()

    drive_direction = "FORWARD"
    prev_channels = [0] * len(get_channels(pad))
    active_command = "NONE"
    hold_counter = 0

    clock = pygame.time.Clock()
    try:
        while True:
            pygame.event.pump()
            channels = get_channels(pad)

            # PS button (index 10) -> emergency stop
            if channels[10]:
                active_command = "estop"
                hold_counter = 1
            else:
                # Direction toggles (l1 / r1)
                if   channels[15]:   drive_direction = "FORWARD"
                elif channels[16]:   drive_direction = "BACKWARD"

                # D-pad overrides for fixed-speed climb
                drive_speed = channels[4]
                if   channels[11]:   drive_speed, drive_direction = 204, "FORWARD"
                elif channels[12]:   drive_speed, drive_direction = 204, "BACKWARD"

                # Button commands
                #   cross=6, circle=7, square=8, triangle=9
                #   share=17, options=18
                if   just_pressed(channels, prev_channels, 6):  active_command, hold_counter = "down_detach", HOLD_FRAMES
                elif just_pressed(channels, prev_channels, 7):  active_command, hold_counter = "down_attach", HOLD_FRAMES
                elif just_pressed(channels, prev_channels, 8):  active_command, hold_counter = "up_attach",   HOLD_FRAMES
                elif just_pressed(channels, prev_channels, 9):  active_command, hold_counter = "up_detach",   HOLD_FRAMES
                elif just_pressed(channels, prev_channels, 17): active_command, hold_counter = "both_attach", HOLD_FRAMES
                elif just_pressed(channels, prev_channels, 18): active_command, hold_counter = "both_detach", HOLD_FRAMES

            if hold_counter > 0:
                command_out = active_command
                hold_counter -= 1
            else:
                command_out = "NONE"
                active_command = "NONE"

            prev_channels = list(channels)

            payload = f"{drive_speed},{drive_direction},{command_out}\n"
            if link:
                link.send_command(payload)

            sensor_data = sensor.latest if sensor else None
            sys.stdout.write(f"\r\033[K[SEND] {payload.strip()}")
            if sensor_data:
                sys.stdout.write(f"  [DATA] {sensor_data[:80]}")
            sys.stdout.flush()

            clock.tick(SEND_RATE)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if link: link.close()
        pygame.quit()
        print("Cleaned up.")


if __name__ == "__main__":
    main()