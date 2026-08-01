"""
Gamepad -> Pi WebSocket bridge (laptop side)
============================================
Replaces the direct USB-serial link from gamepad_to_arduino.py.
The gamepad is held by the laptop, the laptop connects to the Pi Zero
AP over WiFi, and the Pi forwards the same CSV payload to the Arduino
over USB-serial.

Keybind logic, channel enumeration, edge detection, and the CSV payload
format are reused verbatim from gamepad_to_arduino.py so any test that
worked against the original script will work against this one.

Run:
    pip install pygame websockets
    python3 gamepad_to_pi.py
    python3 gamepad_to_pi.py --host 192.168.4.1
    python3 gamepad_to_pi.py --test      # no controller / no WS needed
"""

import argparse
import asyncio
import json
import sys
import time
from typing import Optional

import pygame
import websockets

# ---------------------------- Configuration (preserved) ----------------------------
BAUD_RATE_NOT_USED = 115200  # Documentation only; no longer a serial link.
SEND_RATE = 30               # 30 Hz
HOLD_FRAMES = 3               # ~100ms pulse

# ---------------------------- Helpers (from gamepad_to_arduino.py) ----------------------------
def normalize_stick(val):
    return max(0, min(255, int((val + 32768) / 65535.0 * 255)))

def normalize_trigger(val):
    if val < 0:
        val = (val + 32768) / 65535.0 * 255
    else:
        val = (val / 32767.0) * 255
    return max(0, min(255, int(val)))

def get_21_channels(pad):
    """Reads 21 discrete channels using SDL2 standard numerical IDs."""
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

    return [
        lx, ly, rx, ry, l2, r2,
        cross, circle, square, triangle, ps_btn,
        dp_up, dp_down, dp_left, dp_right,
        l1, r1, share, options, l3, r3
    ]

# ---------------------------- Async WS client ----------------------------
class PiLink:
    """Maintains a single persistent WS connection to the Pi bus."""

    def __init__(self, host: str, port: int, path: str = "/bus"):
        self.url = f"ws://{host}:{port}{path}"
        self.ws: Optional[object] = None
        self._send_queue: asyncio.Queue = asyncio.Queue()
        self._stop = False
        self._connected_evt = asyncio.Event()
        self._recv_task: Optional[asyncio.Task] = None
        self._retry_task: Optional[asyncio.Task] = None
        self._lock = asyncio.Lock()

    async def start(self):
        self._retry_task = asyncio.create_task(self._connect_loop())

    async def stop(self):
        self._stop = True
        if self._retry_task:
            self._retry_task.cancel()
            try:
                await self._retry_task
            except Exception:
                pass

    async def send(self, payload):
        if self.ws is None:
            return
        await self._send_queue.put(payload)

    async def _connect_loop(self):
        backoff = 1.0
        while not self._stop:
            try:
                async with websockets.connect(self.url, ping_interval=20, ping_timeout=20) as ws:
                    self.ws = ws
                    self._connected_evt.set()
                    print(f"[ws] connected to {self.url}")
                    backoff = 1.0
                    self._recv_task = asyncio.create_task(self._recv_loop(ws))
                    send_task = asyncio.create_task(self._send_loop())
                    done, pending = await asyncio.wait(
                        {self._recv_task, send_task}, return_when=asyncio.FIRST_COMPLETED
                    )
                    for t in pending:
                        t.cancel()
            except Exception as e:
                print(f"[ws] disconnect: {e}; retry in {backoff:.1f}s")
                self.ws = None
                self._connected_evt.clear()
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, 10.0)

    async def _recv_loop(self, ws):
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if msg.get("kind") == "ack":
                print(f"\n[ARDUINO REPLY] ACK:{msg.get('cmd', '')}")

    async def _send_loop(self):
        while True:
            payload = await self._send_queue.get()
            if payload is None:
                return
            try:
                await self.ws.send(payload)
            except Exception:
                return

    async def wait_connected(self):
        await self._connected_evt.wait()

# ---------------------------- Main loop ----------------------------
async def main_async(args):
    link = PiLink(args.host, args.port, args.path)
    await link.start()

    if not args.test:
        pygame.init()
        from pygame._sdl2 import controller
        controller.init()
        pad = None
        for i in range(controller.get_count()):
            if controller.is_controller(i):
                pad = controller.Controller(i)
                pad.init()
                break
        if not pad:
            print("No PS4 controller detected. Use --test or plug one in.", file=sys.stderr)
            sys.exit(1)
        print(f"Connected controller: {pad.name}")
    else:
        pad = None
        print("TEST MODE: no controller and no WS required.")

    drive_direction = "FORWARD"
    prev_channels = [0] * 21
    active_command = "NONE"
    hold_counter = 0

    last_send = 0.0
    try:
        while True:
            loop_dt = time.time() - last_send
            if loop_dt < 1.0 / SEND_RATE:
                await asyncio.sleep(1.0 / SEND_RATE - loop_dt)
            last_send = time.time()

            if pad is not None:
                pygame.event.pump()
                channels = get_21_channels(pad)

                def just_pressed(idx):
                    return channels[idx] == 1 and prev_channels[idx] == 0

                if channels[15] == 1:
                    drive_direction = "FORWARD"
                elif channels[16] == 1:
                    drive_direction = "BACKWARD"

                drive_speed = channels[4]

                if channels[11] == 1:
                    drive_speed = 204
                    drive_direction = "FORWARD"
                elif channels[12] == 1:
                    drive_speed = 204
                    drive_direction = "BACKWARD"

                if just_pressed(6):
                    active_command, hold_counter = "down_detach", HOLD_FRAMES
                elif just_pressed(7):
                    active_command, hold_counter = "down_attach", HOLD_FRAMES
                elif just_pressed(8):
                    active_command, hold_counter = "up_attach",   HOLD_FRAMES
                elif just_pressed(9):
                    active_command, hold_counter = "up_detach",   HOLD_FRAMES
                elif just_pressed(17):
                    active_command, hold_counter = "both_attach", HOLD_FRAMES
                elif just_pressed(18):
                    active_command, hold_counter = "both_detach", HOLD_FRAMES

                if hold_counter > 0:
                    command_out = active_command
                    hold_counter -= 1
                else:
                    command_out = "NONE"
                    active_command = "NONE"

                prev_channels = list(channels)
            else:
                # Test mode: synthesize a slow oscillating drive
                t = time.time()
                drive_speed = int((t * 0.5 % 1.0) * 255)
                command_out = "NONE"
                drive_direction = "FORWARD" if int(t) % 2 == 0 else "BACKWARD"

            csv_payload = f"{drive_speed},{drive_direction},{command_out}\n"
            if args.pretty:
                csv_payload = json.dumps({
                    "kind": "command",
                    "payload": csv_payload.strip(),
                }) + "\n"

            if args.test or link.ws is not None:
                await link.send(csv_payload)
                sys.stdout.write(f"\r[RAW] {csv_payload.strip()}    ")
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if pad is not None:
            pygame.quit()
        await link.stop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.4.1", help="Pi AP IP")
    ap.add_argument("--port", default=81, type=int)
    ap.add_argument("--path", default="/bus")
    ap.add_argument("--test", action="store_true",
                    help="Synthesize drive commands (no controller / no WS)")
    ap.add_argument("--pretty", action="store_true",
                    help="Send JSON envelopes instead of raw CSV")
    args = ap.parse_args()

    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()