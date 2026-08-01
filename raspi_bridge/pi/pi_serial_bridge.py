#!/usr/bin/env python3
"""
Pi Zero 2 W - Serial <-> WebSocket bridge
==========================================
Talks to the Arduino over USB-serial and re-publishes its output onto a
WebSocket bus on the local network. In the other direction, every inbound
WebSocket message is normalized to the CSV line format the Arduino expects
and written to the serial port.

Serial from Arduino (line oriented):
  STS:{...json...}      -> {"kind":"telemetry", ...payload...}
  ACK:<cmd>             -> {"kind":"ack", "cmd": "<cmd>"}

WebSocket inbound shapes (any of these will be forwarded as CSV):
  {"kind":"command", "payload":"150,FORWARD,NONE"}
  free text "150,FORWARD,NONE"          (raw CSV passthrough)

Run on the Pi:
  python3 pi_serial_bridge.py             # talks to a real Arduino
  python3 pi_serial_bridge.py --test      # synthesizes mock telemetry
"""

import argparse
import asyncio
import json
import glob
import logging
import os
import signal
import sys
import time
from typing import Optional, Set

import serial

try:
    import websockets
    from websockets.server import serve as ws_serve
except ImportError:
    print("Install websockets: pip install websockets", file=sys.stderr)
    raise

# ---------------------------- Config ----------------------------
WS_HOST       = "0.0.0.0"
WS_PORT       = 81
WS_PATH       = "/bus"
SERIAL_BAUD   = 115200
SERIAL_RDEAD  = 2.0
SERIAL_WDEAD  = 2.0
LOG_LEVEL     = "INFO"

logging.basicConfig(
    level=LOG_LEVEL,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("bridge")

# ---------------------------- Serial port discovery ----------------------------
def find_arduino_port(wait_seconds: float = 0.0,
                      log_waiting: bool = True) -> Optional[str]:
    """Return first /dev/serial/by-id/ symlink, else /dev/ttyACM0, else None.

    With ``wait_seconds > 0`` we poll for the device to appear before giving
    up — handy when the Arduino powers up a moment after the Pi.

    With ``wait_seconds < 0`` we wait essentially forever and only return
    when the device is found. This is the default for the systemd services
    so a missing Arduino no longer crash-loops the bridge.
    """
    first_pass = True
    while True:
        by_id = sorted(glob.glob("/dev/serial/by-id/*"))
        if by_id:
            log.info("Arduino serial device (by-id): %s", by_id[0])
            return by_id[0]
        for cand in ("/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"):
            if os.path.exists(cand):
                log.info("Arduino serial device (fallback): %s", cand)
                return cand
        if wait_seconds == 0.0:
            return None
        if first_pass and log_waiting:
            log.warning(
                "Arduino serial not detected yet (check USB cable / power). "
                "Use --test for a synthetic bridge.",
            )
            first_pass = False
        if wait_seconds > 0:
            time.sleep(1.0)
            return None  # one-shot probe; outer driver will handle retries
        # wait_seconds < 0 → poll forever
        time.sleep(2.0)

# ---------------------------- Bus ----------------------------
class SerialBus:
    """Owns the serial port, fans its lines out to all WS clients."""

    def __init__(self, port: Optional[str], baud: int, test_mode: bool):
        self.port = port
        self.baud = baud
        self.test_mode = test_mode
        self._ser: Optional[serial.Serial] = None
        self._clients: Set["websockets.WebSocketServerProtocol"] = set()
        self._clients_lock = asyncio.Lock()
        self._stop = False
        self._reader_task: Optional[asyncio.Task] = None
        self._writer_task: Optional[asyncio.Task] = None
        self._wqueue: asyncio.Queue = asyncio.Queue()
        self._mock_t = 0.0

    # ---- Lifecycle ----
    async def start(self):
        if not self.test_mode:
            assert self.port is not None
            # Open the serial port with a few retries; the Arduino may
            # reset itself right after enumeration and pyserial returns
            # "Resource temporarily unavailable" briefly.
            last_err: Optional[Exception] = None
            for attempt in range(8):
                try:
                    self._ser = serial.Serial(
                        self.port, self.baud,
                        timeout=0.1, write_timeout=SERIAL_WDEAD,
                        rtscts=False, xonxoff=False,
                    )
                    self._ser.reset_input_buffer()
                    self._ser.reset_output_buffer()
                    log.info("Opened serial %s @ %d", self.port, self.baud)
                    last_err = None
                    break
                except Exception as e:
                    last_err = e
                    log.warning("Serial open attempt %d/8 failed: %s",
                                attempt + 1, e)
                    await asyncio.sleep(0.5)
            if last_err is not None:
                log.error("Giving up opening serial: %s", last_err)
                raise last_err
        self._reader_task = asyncio.create_task(self._reader_loop())
        self._writer_task = asyncio.create_task(self._writer_loop())

    async def stop(self):
        self._stop = True
        for t in (self._reader_task, self._writer_task):
            if t:
                t.cancel()
        for t in (self._reader_task, self._writer_task):
            if t:
                try:
                    await t
                except BaseException:
                    # CancelledError (and any task-internal errors) are
                    # expected during shutdown — swallow them.
                    pass
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass

    # ---- Client registration ----
    async def add_client(self, ws):
        async with self._clients_lock:
            self._clients.add(ws)
        log.info("Client connected (%d total)", len(self._clients))

    async def remove_client(self, ws):
        async with self._clients_lock:
            self._clients.discard(ws)
        log.info("Client disconnected (%d total)", len(self._clients))

    # ---- Inbound (from WS -> Arduino) ----
    async def handle_inbound(self, ws, raw) -> None:
        text = raw.decode("utf-8", errors="ignore") if isinstance(raw, (bytes, bytearray)) else raw
        text = text.strip()
        if not text:
            return
        payload = self._to_csv(text)
        if payload is None:
            log.warning("Ignoring unrecognized inbound: %r", text[:80])
            return
        await self._wqueue.put(payload)
        log.debug("-> arduino: %s", payload.strip())

    @staticmethod
    def _to_csv(text: str) -> Optional[str]:
        """Coerce any inbound to '<speed>,<dir>,<cmd>\n'. Returns None on bad input."""
        try:
            msg = json.loads(text)
            if isinstance(msg, dict):
                if "payload" in msg and isinstance(msg["payload"], str):
                    p = msg["payload"].strip()
                    if SerialBus._looks_like_csv(p):
                        return p + "\n"
                # JSON command with split fields
                if msg.get("kind") == "command":
                    speed = msg.get("speed", 0)
                    direction = msg.get("dir", "FORWARD")
                    command = msg.get("cmd", "NONE")
                    return f"{int(speed)},{direction},{command}\n"
        except json.JSONDecodeError:
            pass
        if SerialBus._looks_like_csv(text):
            return text + "\n"
        return None

    @staticmethod
    def _looks_like_csv(text: str) -> bool:
        parts = text.split(",")
        if len(parts) != 3:
            return False
        try:
            int(parts[0])
        except ValueError:
            return False
        return parts[1] in ("FORWARD", "BACKWARD", "STOP")

    # ---- Outbound (from Arduino -> WS) ----
    async def _broadcast(self, payload: dict):
        if not self._clients:
            return
        msg = json.dumps(payload, separators=(",", ":"))
        async with self._clients_lock:
            dead = []
            for ws in list(self._clients):
                try:
                    await ws.send(msg)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                self._clients.discard(ws)

    # ---- Reader loop (arduino -> ws) ----
    async def _reader_loop(self):
        while not self._stop:
            try:
                line = await self._read_line()
            except asyncio.CancelledError:
                raise
            except Exception as e:
                log.error("Reader crashed: %s; restarting", e)
                await asyncio.sleep(0.5)
                continue
            if line is None:
                continue
            line = line.strip()
            if not line:
                continue
            if line.startswith("STS:"):
                body = line[4:]
                try:
                    payload = json.loads(body)
                except json.JSONDecodeError:
                    log.warning("Bad STS JSON: %r", body[:80])
                    continue
                payload.setdefault("kind", "telemetry")
                await self._broadcast(payload)
            elif line.startswith("ACK:"):
                cmd = line[4:].strip()
                await self._broadcast({"kind": "ack", "cmd": cmd})
            else:
                log.debug("Non-framed line: %r", line[:80])

    async def _read_line(self) -> Optional[str]:
        if self.test_mode:
            await asyncio.sleep(0.2)
            self._mock_t += 0.2
            t = self._mock_t
            payload = {
                "speed": 0,
                "dir": "FORWARD",
                "pose": "parallel",
                "ack": "NONE",
                "tof": {
                    "up": 350 + int(5 * (t % 3)),
                    "down": 120 + int(5 * (t % 4)),
                },
            }
            await self._broadcast(payload)
            return None
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, self._read_line_blocking)

    def _read_line_blocking(self) -> Optional[str]:
        if not self._ser:
            return None
        try:
            line = self._ser.readline()
        except (serial.SerialException, OSError) as e:
            log.error("Serial read error: %s", e)
            return None
        if not line:
            return None
        return line.decode("utf-8", errors="ignore")

    # ---- Writer loop (ws -> arduino) ----
    async def _writer_loop(self):
        while not self._stop:
            try:
                payload = await self._wqueue.get()
            except asyncio.CancelledError:
                raise
            if self.test_mode:
                log.info("MOCK arduino <- %s", payload.strip())
                continue
            loop = asyncio.get_running_loop()
            try:
                await asyncio.wait_for(
                    loop.run_in_executor(None, self._ser.write, payload.encode("utf-8")),
                    timeout=2.0,
                )
            except Exception as e:
                log.error("Serial write failed: %s", e)
            try:
                await asyncio.wait_for(
                    loop.run_in_executor(None, self._ser.flush),
                    timeout=2.0,
                )
            except Exception:
                pass

# ---------------------------- Server ----------------------------
class BridgeServer:
    def __init__(self, bus: SerialBus):
        self.bus = bus

    async def __call__(self, ws):
        if ws.path != WS_PATH:
            await ws.close(code=1008, reason="unknown path")
            return
        await self.bus.add_client(ws)
        try:
            async for raw in ws:
                await self.bus.handle_inbound(ws, raw)
        except Exception as e:
            log.debug("WS loop ended: %s", e)
        finally:
            await self.bus.remove_client(ws)

# ---------------------------- Entrypoint ----------------------------
async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=WS_PORT, type=int, help="WS port")
    ap.add_argument("--serial", default=None, help="Serial device path")
    ap.add_argument("--baud", default=SERIAL_BAUD, type=int)
    ap.add_argument("--test", action="store_true",
                    help="Mock serial I/O (no Arduino needed).")
    ap.add_argument(
        "--auto-test", action="store_true",
        help="Use --test mode automatically if no Arduino is detected "
             "within --wait-serial seconds (default off).",
    )
    ap.add_argument(
        "--wait-serial", default="60", type=float,
        help="Seconds to wait for the Arduino before giving up / falling "
             "back. Use -1 to wait forever (default 60).",
    )
    args = ap.parse_args()

    # /etc/default/climbingrobot can force mock mode at startup.
    env_force_test = os.environ.get("TEST_MODE", "0") == "1"

    port = args.serial
    test_mode = args.test or env_force_test
    fallback_to_test = (args.auto_test and not test_mode)

    if env_force_test and not args.test:
        log.warning("TEST_MODE=1 in environment — forcing mock serial")
        test_mode = True

    if not test_mode and not port:
        wait = args.wait_serial
        if wait < 0:
            log.info("Waiting for Arduino serial device (--wait-serial=-1)")
            port = find_arduino_port(wait_seconds=-1)
        else:
            port = find_arduino_port(wait_seconds=wait)
        if not port:
            if fallback_to_test:
                log.warning(
                    "No Arduino after %.0fs — falling back to --test mode",
                    wait,
                )
                test_mode = True
            else:
                log.error(
                    "No Arduino serial detected. Pass --test, "
                    "--auto-test, or set TEST_MODE=1 to run without hardware.",
                )
                sys.exit(1)

    bus = SerialBus(port, args.baud, test_mode)
    await bus.start()

    server = BridgeServer(bus)
    stop = asyncio.Event()

    def _sig(*_):
        log.info("Stopping...")
        stop.set()
    loop = asyncio.get_running_loop()
    for s in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(s, _sig)
        except NotImplementedError:
            pass

    mode = "MOCK" if test_mode else f"real serial {port}"
    log.info("WebSocket server on ws://%s:%d%s  (%s)",
             WS_HOST, args.port, WS_PATH, mode)
    async with ws_serve(server, WS_HOST, args.port, max_size=64 * 1024):
        await stop.wait()
    await bus.stop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass