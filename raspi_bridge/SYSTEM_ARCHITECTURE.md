# Raspberry Pi Bridge System Architecture

## System Overview

The Raspberry Pi Bridge system is a three-tier distributed control architecture for a climbing robot. It separates user input (laptop with gamepad), network communication (Raspberry Pi Zero 2 W), and hardware control (Arduino/ESP32 microcontroller) into independent, loosely-coupled components that communicate over standardized protocols.

```
┌─────────────────┐         ┌──────────────────┐         ┌─────────────────┐
│  Laptop/Mac     │  WiFi   │  Raspberry Pi    │  USB    │  Arduino/ESP32  │
│                 │────────>│  Zero 2 W        │────────>│  Microcontroller│
│  PS4 Gamepad    │  WS/81  │  Serial Bridge   │ Serial  │  Motor/Servo    │
│  Python Client  │<────────│  (WebSocket)     │<────────│  Controller     │
└─────────────────┘         └──────────────────┘         └─────────────────┘
     ^                              ^                            ^
     │                              │                            │
  User Input                   WiFi Bridge                  Hardware Control
  - Joystick mapping          - Protocol translation        - PWM generation
  - Button edge detection     - Multi-client fanout         - Sensor reading
  - Command generation        - Error recovery              - Safety latching
```

## Design Philosophy

### Separation of Concerns

1. **Laptop Layer** (gamepad_to_pi.py)
   - Human interface
   - Input device abstraction
   - High-level command generation
   - No knowledge of hardware details

2. **Pi Layer** (pi_serial_bridge.py)
   - Network-to-serial translation
   - Protocol normalization
   - Connection management
   - Device discovery

3. **Microcontroller Layer** (arduino_bridge.ino)
   - Real-time motor control
   - Servo timing
   - Sensor sampling
   - Hardware safety enforcement

### Communication Protocol Stack

```
Application Layer:  CSV commands (speed,dir,cmd) + JSON telemetry
Transport Layer:    WebSocket (laptop↔Pi) + UART (Pi↔Arduino)
Physical Layer:     WiFi 2.4GHz + USB 2.0 Full Speed
```

---

## Component 1: Arduino Bridge Firmware

### Hardware Platform

**Target**: Arduino Mega 2560 (ATmega2560)
- **Clock**: 16 MHz
- **Flash**: 256 KB
- **SRAM**: 8 KB
- **EEPROM**: 4 KB
- **I/O Pins**: 54 digital, 16 analog

### Pin Configuration

```cpp
// I2C Bus
SDA → Pin 20 (A4 on Uno)
SCL → Pin 21 (A5 on Uno)

// Motor Driver (L298N Dual H-Bridge)
Motor A EN  → Pin 10 (PWM)
Motor A IN1 → Pin 9
Motor A IN2 → Pin 8
Motor B EN  → Pin 5 (PWM)
Motor B IN1 → Pin 7
Motor B IN2 → Pin 6

// TOF Sensors (VL53L0X)
TOF_UP_XSHUT   → Pin 4
TOF_DOWN_XSHUT → Pin 3

// Status LED
LED → Pin 13 (onboard LED)
```

### I2C Device Map

```
Address 0x40: PCA9685 Servo Hub (16 channels)
Address 0x30: VL53L0X Upper TOF (re-addressed)
Address 0x31: VL53L0X Lower TOF (re-addressed)
```

### Serial Protocol (Arduino ↔ Pi)

#### Inbound Format (Pi → Arduino)

**Standard Command:**
```
<speed>,<direction>,<command>\n
```

**Fields:**
- `speed`: 0-255 (PWM duty cycle, 8-bit)
- `direction`: "FORWARD" | "BACKWARD"
- `command`: Action identifier

**Valid Commands:**
```
NONE           - No action (default)
up_attach      - Close upper gripper, reset bars
up_detach      - Open upper gripper, angle lower bars
down_attach    - Close lower gripper, reset bars
down_detach    - Open lower gripper, angle upper bars
both_attach    - Close both grippers, parallel bars
both_detach    - Open both grippers, parallel bars
estop          - Emergency stop, latch system
```

**Safety Warning:**
```
WARN:<sensor>\n
```
- `sensor`: "up" | "down"
- Triggers immediate motor stop and system latch

**Examples:**
```
150,FORWARD,NONE\n          # Drive forward at 60% speed
0,FORWARD,up_attach\n       # Stop and attach upper gripper
WARN:up\n                   # Emergency stop (upper sensor)
```

#### Outbound Format (Arduino → Pi)

**Status Telemetry (200 ms period):**
```json
STS:{"speed":150,"dir":"FORWARD","pose":"parallel","ack":"NONE","latched":false,"tof":{"enabled":true,"up":350,"down":120}}
```

**Command Acknowledgment:**
```
ACK:<command>\n
```

**Warning Echo:**
```
WARNED:<sensor>\n
```

**Initialization Message:**
```
INIT: tof up=OK down=OK\n
```

### Servo Control System

#### PCA9685 Configuration

```cpp
I2C Address: 0x40
PWM Frequency: 50 Hz (20 ms period)
Resolution: 12-bit (0-4095)
Pulse Range: 1000-2000 μs (standard servos)
```

#### Channel Assignment

```
Gripper Servos:
  chServoUpL    = 13  (Upper left gripper)
  chServoUpR    = 12  (Upper right gripper, mirrored)
  chServoDownL  = 9   (Lower left gripper)
  chServoDownR  = 8   (Lower right gripper, mirrored)

Bar Servos:
  chServoBarUpL    = 15  (Upper left bar)
  chServoBarUpR    = 14  (Upper right bar, mirrored)
  chServoBarDownL  = 11  (Lower left bar)
  chServoBarDownR  = 10  (Lower right bar, mirrored)
```

#### Bar Pose Configurations

**Parallel (Neutral):**
```cpp
All bars at 90° (neutral position)
Use: Both grippers attached, transitioning between poses
```

**Up-Detach:**
```cpp
Upper bars: 90° (neutral)
Lower bars: 55° / 125° (angled outward 35°)
Use: Upper gripper released, preparing to climb upward
Purpose: Lower bars push outward to create stable platform
```

**Down-Detach:**
```cpp
Upper bars: 125° / 55° (angled outward 35°)
Lower bars: 90° (neutral)
Use: Lower gripper released, preparing to descend
Purpose: Upper bars provide leverage for controlled descent
```

#### Servo Angle Conversion

```cpp
uint16_t angleToPulse(float angle) {
    // Maps 0-180° to 1000-2000 μs pulse width
    float t = angle / 180.0;
    return (uint16_t)(1000 + t * 1000);
}

// For mirrored servos (right side):
angleRight = 180 - angleLeft
```

### Motor Control System

#### PWM Configuration (ATmega2560 Timers)

```cpp
Timer 0 (8-bit): System millis() - DO NOT USE
Timer 1 (16-bit): Pin 11, 12 - Available
Timer 2 (8-bit): Pin 9, 10 - Motor A
Timer 3 (16-bit): Pin 2, 3, 5 - Motor B on Pin 5
Timer 4 (16-bit): Pin 6, 7, 8 - Available
Timer 5 (16-bit): Pin 44, 45, 46 - Available

Motor A: Timer 2 (pins 9, 10)
Motor B: Timer 3 (pin 5)
```

#### Direction Control Logic

```cpp
// Forward motion (climbing up)
digitalWrite(MOTOR_A_IN1, HIGH);
digitalWrite(MOTOR_A_IN2, LOW);
digitalWrite(MOTOR_B_IN1, HIGH);
digitalWrite(MOTOR_B_IN2, LOW);

// Backward motion (descending)
digitalWrite(MOTOR_A_IN1, LOW);
digitalWrite(MOTOR_A_IN2, HIGH);
digitalWrite(MOTOR_B_IN1, LOW);
digitalWrite(MOTOR_B_IN2, HIGH);

// Brake (both pins LOW)
digitalWrite(MOTOR_A_IN1, LOW);
digitalWrite(MOTOR_A_IN2, LOW);
digitalWrite(MOTOR_B_IN1, LOW);
digitalWrite(MOTOR_B_IN2, LOW);

// Coast (both pins HIGH - not used)
```

### Time-of-Flight Sensor System

#### VL53L0X Dual Sensor Setup

**Challenge**: Both sensors share I2C address 0x29 by default.

**Solution**: Sequential XSHUT-based re-addressing:

```cpp
1. Pull both XSHUT pins LOW (power down both sensors)
2. Wait 50 ms for sensors to fully power down
3. Pull TOF_UP_XSHUT HIGH (activate upper sensor only)
4. Wait 50 ms for upper sensor to initialize
5. Change upper sensor address: tof_up.setAddress(0x30)
6. Pull TOF_DOWN_XSHUT HIGH (activate lower sensor)
7. Wait 50 ms for lower sensor to initialize
8. Change lower sensor address: tof_down.setAddress(0x31)
9. Start continuous ranging on both sensors
```

#### Measurement Configuration

```cpp
Timing Budget: 10,000 μs (10 ms per reading)
Mode: Continuous ranging
Range: 30-1200 mm (typical)
Update Rate: ~50 Hz
Accuracy: ±3% at < 1000 mm
```

#### Calibration System

Both sensors use linear two-point calibration:

```
Raw Reading (mm)    Actual Distance (mm)
     87         →        100
     62         →         70
```

**Upper Sensor Calibration:**
```cpp
slope = (100 - 70) / (87 - 62) = 30 / 25 = 1.2
offset = 70 - slope × 62 = -4.4
calibrated = 1.2 × raw - 4.4
```

**Lower Sensor Calibration:**
```cpp
slope = (100 - 70) / (135 - 100) = 30 / 35 ≈ 0.857
offset = 70 - slope × 100 = -15.7
calibrated = 0.857 × raw - 15.7
```

#### Error Recovery

**Detection:**
- Reading = 0xFFFF (65535)
- Reading = 0
- timeout_occurred() returns true

**Recovery Strategy:**
1. Mark sensor as down (reading = 0xFFFF)
2. Wait 5 seconds (avoid I2C bus congestion)
3. Power cycle via XSHUT (LOW → delay → HIGH)
4. Attempt re-initialization
5. If successful, restart continuous ranging
6. If failed, wait another 5 seconds

**Throttling:**
- Minimum read interval: 50 ms
- Prevents I2C bus saturation
- Allows time for sensor measurement cycles

### Safety System

#### Emergency Stop (E-Stop)

**Trigger Conditions:**
1. `estop` command received via serial
2. `WARN:<sensor>` message received
3. Critical sensor failure (future enhancement)

**E-Stop Actions:**
```cpp
1. analogWrite(MOTOR_A_EN, 0)      // Stop Motor A
2. analogWrite(MOTOR_B_EN, 0)      // Stop Motor B
3. digitalWrite(MOTOR_A_IN1, LOW)  // Brake Motor A
4. digitalWrite(MOTOR_A_IN2, LOW)
5. digitalWrite(MOTOR_B_IN1, LOW)  // Brake Motor B
6. digitalWrite(MOTOR_B_IN2, LOW)
7. digitalWrite(LED_PIN, LOW)      // Turn off status LED
8. latched = true                  // Enter latched state
9. strncpy(latch_reason, cause, 15) // Record cause
```

#### Latch Mechanism

**Purpose**: Prevent system from resuming operation after emergency stop until operator explicitly acknowledges and clears the fault.

**Behavior When Latched:**
```cpp
if (latched) {
    if (commandStr == "NONE") {
        return;  // Ignore drive commands
    }
    // Any non-NONE command clears the latch
    latched = false;
    latch_reason[0] = '\0';
}
```

**Rationale**: Forces operator to send a deliberate command (e.g., `both_attach`) to resume, preventing accidental re-activation.

### Main Loop Architecture

```cpp
void loop() {
    unsigned long now = millis();
    
    // 1. Serial command parsing (highest priority)
    if (Serial.available() > 0) {
        String payload = Serial.readStringUntil('\n');
        parseAndExecutePayload(payload);
    }
    
    // 2. Sensor watchdog (every 1000 ms)
    if ((long)(now - next_watchdog_ms) >= 0) {
        next_watchdog_ms = now + 1000;
        ensureSensorsAlive();
    }
    
    // 3. Status telemetry (every 200 ms)
    if ((long)(now - next_status_ms) >= 0) {
        next_status_ms = now + STATUS_PERIOD_MS;
        readSensors();
        emitStatusLine();
    }
    
    // 4. LED update (when motors active)
    if (current_speed > 0) {
        // Flash LED every 500 ms
        if (now - led_flash_timer >= 500) {
            led_state = !led_state;
            digitalWrite(LED_PIN, led_state ? HIGH : LOW);
            led_flash_timer = now;
        }
    } else {
        digitalWrite(LED_PIN, LOW);
    }
}
```

### Memory Management

**SRAM Budget (8 KB total):**
```
VL53L0X objects (2×):          ~600 bytes
Servo hub object:              ~100 bytes
Serial input buffer (String):  ~150 bytes
State variables:               ~50 bytes
Stack (worst case):            ~500 bytes
Arduino core overhead:         ~500 bytes
--------------------------------
Total used:                    ~1900 bytes
Available:                     ~6100 bytes (76% free)
```

**Flash Usage (256 KB total):**
```
Sketch code:                   ~35 KB
Arduino core:                  ~30 KB
VL53L0X library:              ~15 KB
Adafruit_PWMServoDriver:      ~8 KB
Wire library:                  ~5 KB
--------------------------------
Total used:                    ~93 KB
Available:                     ~163 KB (64% free)
```

---

## Component 2: Raspberry Pi Serial Bridge

### Hardware Platform

**Device**: Raspberry Pi Zero 2 W
- **SoC**: Broadcom BCM2710A1 (quad-core Cortex-A53 @ 1 GHz)
- **RAM**: 512 MB LPDDR2
- **WiFi**: 2.4 GHz 802.11b/g/n
- **Bluetooth**: 4.2 BLE
- **USB**: 1× micro-USB OTG (for serial connection)
- **Power**: 5V @ 1.2A (via micro-USB or GPIO)

### Software Stack

```
Operating System: Raspberry Pi OS Lite (Debian 12 Bookworm)
Python Version: 3.11+
Key Dependencies:
  - pyserial (3.5+)
  - websockets (12.0+)
  - systemd (system service integration)
```

### Network Topology

#### Access Point Mode (Default)

```
┌──────────────┐
│   Laptop     │ WiFi          ┌─────────────┐
│  192.168.4.x │◄─────────────►│  Pi Zero 2  │ USB   ┌──────────┐
│              │   WPA2-PSK     │  192.168.4.1│──────►│ Arduino  │
└──────────────┘                └─────────────┘       └──────────┘
                                      │
                SSID: ClimbingRobot   │ 5V Power
                Pass: [configured]    ▼
                                   Battery
```

**Configuration**: `pi_ap_setup.sh`
- Creates isolated WiFi network
- DHCP server on 192.168.4.0/24
- No internet gateway (closed network)
- WPA2-PSK authentication

#### Tailscale Mode (Remote Access)

```
┌──────────────┐                    ┌─────────────┐
│   Laptop     │  Internet/LTE      │  Pi Zero 2  │ USB   ┌──────────┐
│  100.x.x.x   │◄──────────────────►│  100.y.y.y  │──────►│ Arduino  │
│              │  Tailscale VPN     │             │       └──────────┘
└──────────────┘  (encrypted)       └─────────────┘
                                          │
                  Bearer token auth       │ 4G/5G Hotspot
                  required                ▼
                                      Mobile data
```

**Configuration**: `pi_tailscale_setup.sh`
- Peer-to-peer encrypted tunnel
- NAT traversal (works behind firewalls)
- Bearer token authentication required
- Supports remote operation over cellular

### Serial Bridge Architecture

#### Class Hierarchy

```python
SerialBus
├── Serial port management (pyserial)
├── Client registry (WebSocket connections)
├── Read loop (Arduino → WebSocket broadcast)
├── Write loop (WebSocket → Arduino serial)
└── Test mode (mock telemetry generation)

BridgeServer
├── WebSocket server (websockets library)
├── Authentication (bearer token validation)
├── Path routing (/bus endpoint)
└── Client handler (per-connection lifecycle)
```

#### Serial Port Discovery

```python
def find_arduino_port(wait_seconds: float = 0.0) -> Optional[str]:
    """Priority order:
    1. /dev/serial/by-id/* (stable symlinks)
    2. /dev/ttyACM0, /dev/ttyACM1 (Arduino Mega USB)
    3. /dev/ttyUSB0, /dev/ttyUSB1 (FTDI adapters)
    4. None (no device found)
    
    With wait_seconds < 0, polls indefinitely until device appears.
    """
```

**Rationale**: Arduino may power up after the Pi, or USB enumeration may be delayed. Infinite wait mode prevents systemd service crash loops.

#### Protocol Translation

**WebSocket → Serial (Normalization)**

The bridge accepts multiple inbound formats and normalizes them to the Arduino's expected CSV format:

```python
# JSON envelope (preferred)
{"kind":"command", "payload":"150,FORWARD,NONE"}
→ "150,FORWARD,NONE\n"

# Raw CSV passthrough
"150,FORWARD,NONE"
→ "150,FORWARD,NONE\n"

# JSON with split fields
{"kind":"command", "speed":150, "dir":"FORWARD", "cmd":"up_attach"}
→ "150,FORWARD,up_attach\n"

# Safety warning (passthrough)
"WARN:up"
→ "WARN:up\n"
```

**Serial → WebSocket (Framing)**

Arduino lines are parsed and wrapped in JSON for WebSocket clients:

```python
# Status telemetry
"STS:{...json...}"
→ {"kind":"telemetry", ...parsed_payload...}

# Command acknowledgment
"ACK:up_attach"
→ {"kind":"ack", "cmd":"up_attach"}

# Warning echo
"WARNED:up"
→ {"kind":"warned", "sensor":"up"}

# Initialization message
"INIT: tof up=OK down=OK"
→ {"kind":"init", "message":"tof up=OK down=OK"}
```

#### Multi-Client Broadcast

**Design**: One-to-many fanout pattern.

```python
clients: Set[WebSocketServerProtocol] = set()

async def _broadcast(payload: dict):
    msg = json.dumps(payload)
    async with clients_lock:
        for ws in list(clients):
            try:
                await ws.send(msg)
            except Exception:
                # Dead connection; will be cleaned up
                clients.discard(ws)
```

**Use Cases:**
- Multiple operators monitoring telemetry
- Laptop + smartphone dashboard simultaneously
- Data logger + live control client

### Authentication System

#### Bearer Token Validation

**Purpose**: Secure the bridge when exposed over untrusted networks (Tailscale, mobile hotspot, public WiFi).

**Implementation:**
```python
# At WebSocket handshake (HTTP Upgrade)
def _check_auth_at_handshake(path, request_headers):
    if not auth_token:
        return None  # Auth disabled
    
    presented = request_headers.get("Authorization", "")
    if presented.startswith("Bearer ") and hmac.compare_digest(
        presented[7:].strip(), auth_token
    ):
        return None  # Accepted
    
    # Reject with HTTP 401
    return (401, [("WWW-Authenticate", "Bearer")], b"unauthorized\n")
```

**Timing-Safe Comparison**: Uses `hmac.compare_digest()` to prevent timing attacks.

**Client Side:**
```python
# In gamepad_to_pi.py
connect_kwargs = {
    "additional_headers": {
        "Authorization": f"Bearer {auth_token}"
    }
}
```

#### Security Model

**Threat Model:**

| Network Type | Auth Required | Rationale |
|--------------|---------------|-----------|
| Pi AP (WPA2-PSK) | No | WPA2 is the perimeter; only authorized devices can join |
| Home WiFi (trusted LAN) | No (with `--no-tailscale`) | Network already trusted |
| Tailscale | Yes | VPN doesn't imply authentication at app level |
| Mobile hotspot | Yes | Shared with untrusted devices |
| Public WiFi | Yes | Hostile environment |

**Token Management:**
```bash
# Generate secure token
openssl rand -base64 32

# Set on Pi
export BRIDGE_AUTH_TOKEN="your-token-here"
systemctl restart climbingrobot

# Set on laptop
export BRIDGE_AUTH_TOKEN="your-token-here"
python3 gamepad_to_pi.py --host 100.x.y.z
```

### Error Recovery

#### Serial Port Recovery

**Failure Modes:**
1. Arduino powers up after Pi
2. USB cable disconnected/reconnected
3. Arduino reset (sketch upload, power glitch)

**Recovery Strategy:**
```python
# Open serial with retries
for attempt in range(8):
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        break
    except Exception as e:
        await asyncio.sleep(0.5)  # Arduino may be resetting

# In read loop: catch serial exceptions, log but don't crash
try:
    line = ser.readline()
except (serial.SerialException, OSError) as e:
    log.error("Serial read error: %s", e)
    return None  # Return to loop; watchdog will retry
```

#### WebSocket Connection Recovery

**Failure Modes:**
1. WiFi signal loss
2. Pi network restart
3. Client crash/reconnect

**Client-Side Recovery (gamepad_to_pi.py):**
```python
async def _connect_loop(self):
    backoff = 1.0
    while not self._stop:
        try:
            async with websockets.connect(self.url) as ws:
                backoff = 1.0  # Reset on success
                # Handle messages...
        except Exception as e:
            print(f"disconnect: {e}; retry in {backoff:.1f}s")
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 10.0)  # Exponential backoff
```

**Server-Side Handling:**
- Each client connection runs independently
- Failed broadcasts are caught per-client
- Dead clients removed from registry automatically

### Test Mode

**Purpose**: Develop and test control clients without physical hardware.

**Activation:**
```bash
# On Pi
python3 pi_serial_bridge.py --test

# Or via systemd
echo "TEST_MODE=1" >> /etc/default/climbingrobot
systemctl restart climbingrobot
```

**Mock Behavior:**
```python
# Generates synthetic telemetry every 200 ms
payload = {
    "speed": 0,
    "dir": "FORWARD",
    "pose": "parallel",
    "latched": False,
    "tof": {
        "up": 350 + int(5 * (t % 3)),    # Oscillates 350-365
        "down": 120 + int(5 * (t % 4)),  # Oscillates 120-140
    }
}

# Logs inbound commands without sending to serial
log.info("MOCK arduino <- %s", payload.strip())
```

### Systemd Integration

**Service File**: `/etc/systemd/system/climbingrobot.service`

```ini
[Unit]
Description=Climbing Robot Serial Bridge
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/robot
EnvironmentFile=-/etc/default/climbingrobot
ExecStart=/usr/bin/python3 /home/pi/robot/pi_serial_bridge.py \
    --wait-serial=-1 \
    --auto-test
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Environment File**: `/etc/default/climbingrobot`

```bash
# Optional: force test mode (no Arduino needed)
TEST_MODE=0

# Optional: bearer token for authentication
BRIDGE_AUTH_TOKEN=your-secure-token-here

# Optional: custom serial port
# SERIAL_PORT=/dev/ttyACM0
```

**Management Commands:**
```bash
# Start service
sudo systemctl start climbingrobot

# Enable on boot
sudo systemctl enable climbingrobot

# View logs
sudo journalctl -u climbingrobot -f

# Restart after config change
sudo systemctl restart climbingrobot
```

---

## Component 3: Laptop Gamepad Client

### Hardware Requirements

**Gamepad**: Sony DualShock 4 (PS4 controller)
- **Connection**: USB or Bluetooth
- **Axes**: 6 (2 joysticks + 2 triggers)
- **Buttons**: 15 (face buttons + D-pad + shoulders + start/select)
- **Driver**: SDL2 (cross-platform)

**Laptop**: macOS, Linux, or Windows
- **Python**: 3.8+
- **WiFi**: 802.11n or better

### Software Dependencies

```bash
# Install dependencies
pip install pygame websockets

# On macOS, may need SDL2 from Homebrew
brew install sdl2
```

### Button Mapping

The gamepad maps 21 discrete channels:

```python
Channel  Input         Type       Range      Usage
--------------------------------------------------------------
0        Left stick X  Analog     0-255      (reserved)
1        Left stick Y  Analog     0-255      Drive speed
2        Right stick X Analog     0-255      (reserved)
3        Right stick Y Analog     0-255      (reserved)
4        L2 trigger    Analog     0-255      Drive speed (overrides stick)
5        R2 trigger    Analog     0-255      Altitude control (hold mode)
6        Cross (×)     Digital    0/1        down_detach
7        Circle (○)    Digital    0/1        down_attach
8        Square (□)    Digital    0/1        up_attach
9        Triangle (△)  Digital    0/1        up_detach
10       PS button     Digital    0/1        (reserved)
11       D-pad up      Digital    0/1        Quick drive forward (speed=204)
12       D-pad down    Digital    0/1        Quick drive backward (speed=204)
13       D-pad left    Digital    0/1        Zero altitude reference
14       D-pad right   Digital    0/1        (reserved)
15       L1            Digital    0/1        Set direction = FORWARD
16       R1            Digital    0/1        Set direction = BACKWARD
17       Share         Digital    0/1        both_attach
18       Options       Digital    0/1        both_detach
19       L3            Digital    0/1        (reserved)
20       R3            Digital    0/1        (reserved)
```

### Control Logic

#### Drive Speed

```python
# Primary: Left stick Y-axis
drive_speed = normalize_stick(pad.get_axis(1))  # 0-255

# Override: L2 trigger (analog)
if channels[4] > 0:
    drive_speed = channels[4]  # L2 takes precedence

# Override: D-pad quick drive
if channels[11]:  # D-pad up
    drive_speed = 204  # 80% power
    drive_direction = "FORWARD"
elif channels[12]:  # D-pad down
    drive_speed = 204
    drive_direction = "BACKWARD"
```

#### Direction Control

```python
# Default direction (persistent)
drive_direction = "FORWARD"

# Toggle with shoulder buttons
if channels[15]:  # L1
    drive_direction = "FORWARD"
elif channels[16]:  # R1
    drive_direction = "BACKWARD"

# D-pad overrides temporarily
if channels[11]:  # D-pad up
    drive_direction = "FORWARD"
elif channels[12]:  # D-pad down
    drive_direction = "BACKWARD"
```

#### Altitude Control Mode

```python
# Default mode
altitude_mode = "manual"

# Hold altitude when R2 trigger pulled
if channels[5] > 10:  # R2 threshold
    altitude_mode = "hold_alt"

# Zero altitude reference (pulse once on edge)
elif just_pressed(13):  # D-pad left
    altitude_mode = "zero_alt"
```

**Note**: This field is sent separately from the button command field, allowing simultaneous altitude control and gripper commands.

#### Button Commands (Edge-Triggered)

```python
def just_pressed(idx):
    return channels[idx] == 1 and prev_channels[idx] == 0

# Gripper control
if just_pressed(6):   # Cross
    active_command = "down_detach"
    hold_counter = HOLD_FRAMES  # Pulse for ~33 ms
elif just_pressed(7):  # Circle
    active_command = "down_attach"
    hold_counter = HOLD_FRAMES
elif just_pressed(8):  # Square
    active_command = "up_attach"
    hold_counter = HOLD_FRAMES
elif just_pressed(9):  # Triangle
    active_command = "up_detach"
    hold_counter = HOLD_FRAMES

# Both grippers
elif just_pressed(17):  # Share
    active_command = "both_attach"
    hold_counter = HOLD_FRAMES
elif just_pressed(18):  # Options
    active_command = "both_detach"
    hold_counter = HOLD_FRAMES

# Hold counter decrements each frame; command sent only while > 0
if hold_counter > 0:
    command_out = active_command
    hold_counter -= 1
else:
    command_out = "NONE"
```

**Rationale**: Edge detection prevents button holds from spamming the same command. A brief pulse (1-2 frames @ 30 Hz = 33-66 ms) is sufficient for the Arduino to register the command.

### WebSocket Client Architecture

```python
class PiLink:
    """Persistent WebSocket connection with automatic reconnection."""
    
    def __init__(self, host, port, path="/bus", auth_token=""):
        self.url = f"ws://{host}:{port}{path}"
        self.auth_token = auth_token
        self.ws: Optional[WebSocket] = None
        self._send_queue: asyncio.Queue = asyncio.Queue()
        self._connected_evt = asyncio.Event()
    
    async def _connect_loop(self):
        """Maintains connection with exponential backoff."""
        backoff = 1.0
        while not self._stop:
            try:
                connect_kwargs = {"ping_interval": 20, "ping_timeout": 20}
                if self.auth_token:
                    connect_kwargs["additional_headers"] = {
                        "Authorization": f"Bearer {self.auth_token}"
                    }
                async with websockets.connect(self.url, **connect_kwargs) as ws:
                    self.ws = ws
                    backoff = 1.0  # Reset on success
                    await self._recv_loop(ws)
            except Exception as e:
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, 10.0)
    
    async def send(self, payload):
        """Queue payload for transmission."""
        await self._send_queue.put(payload)
```

#### Ping/Pong Keepalive

```python
# websockets library configuration
ping_interval = 20   # Send ping every 20 seconds
ping_timeout = 20    # Close if no pong in 20 seconds
```

**Rationale**: Detects dead connections quickly, especially over cellular networks where TCP keepalive may not fire for minutes.

### Command Generation Pipeline

```
┌──────────────┐      ┌──────────────┐      ┌──────────────┐      ┌──────────────┐
│  SDL2 Input  │─────>│  Channel Map │─────>│ Control Logic│─────>│  CSV Format  │
│  (Raw Axes)  │      │  (21 values) │      │ (Edge Detect)│      │  (Serial)    │
└──────────────┘      └──────────────┘      └──────────────┘      └──────────────┘
  60-120 Hz              30 Hz                30 Hz                 30 Hz
  
  Button press     →   just_pressed(6)  →   "down_detach"    →   "0,FORWARD,down_detach,manual\n"
  Joystick Y=128   →   channels[1]=128  →   drive_speed=128  →   "128,FORWARD,NONE,manual\n"
  R2 trigger=200   →   channels[5]=200  →   altitude_mode="hold_alt" → "...,hold_alt\n"
```

### Main Loop Timing

```python
SEND_RATE = 30  # Hz
last_send = 0.0

while True:
    # Rate limiting
    loop_dt = time.time() - last_send
    if loop_dt < 1.0 / SEND_RATE:
        await asyncio.sleep(1.0 / SEND_RATE - loop_dt)
    last_send = time.time()
    
    # Poll controller
    pygame.event.pump()
    channels = get_21_channels(pad)
    
    # Process input
    # ... (button logic)
    
    # Generate CSV
    csv_payload = f"{drive_speed},{drive_direction},{command_out},{altitude_mode}\n"
    
    # Send to Pi
    await link.send(csv_payload)
```

**Rate**: 30 Hz (every 33.3 ms)
- Fast enough for responsive control
- Slow enough to avoid overwhelming serial link
- Matches typical gamepad polling rate

### Acknowledgment Handling

```python
async def _recv_loop(self, ws):
    """Receive and display Arduino responses."""
    async for raw in ws:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            continue
        
        if msg.get("kind") == "ack":
            print(f"\n[ARDUINO REPLY] ACK:{msg.get('cmd', '')}")
        elif msg.get("kind") == "telemetry":
            # Could update dashboard here
            pass
```

**Purpose**: Provides operator feedback that commands were executed. Useful for debugging missed commands or communication issues.

### Test Mode

```bash
# Run without controller or network
python3 gamepad_to_pi.py --test

# Generates synthetic oscillating drive commands
t = time.time()
drive_speed = int((t * 0.5 % 1.0) * 255)  # Ramps 0-255 over 2 seconds
direction = "FORWARD" if int(t) % 2 == 0 else "BACKWARD"  # Alternates every second
```

**Use Cases:**
- Test Pi bridge without gamepad
- Verify WebSocket connectivity
- Benchmark command throughput

---

## System Integration

### End-to-End Command Flow

```
1. Operator presses Circle button (down_attach)
   ↓
2. pygame detects button press (channel 7 = 1)
   ↓
3. just_pressed(7) edge detector triggers
   ↓
4. active_command = "down_attach", hold_counter = 1
   ↓
5. CSV generated: "0,FORWARD,down_attach,manual\n"
   ↓
6. WebSocket send to Pi (ws://192.168.4.1:81/bus)
   ↓
7. Pi receives WebSocket message
   ↓
8. Pi normalizes to CSV (already CSV, passes through)
   ↓
9. Pi writes to serial: "0,FORWARD,down_attach,manual\n"
   ↓
10. Arduino Serial.readStringUntil('\n') receives line
    ↓
11. parseAndExecutePayload() parses CSV
    ↓
12. executeCommand("down_attach") called
    ↓
13. Servos move: down gripper opens, bars return to parallel
    ↓
14. Arduino emits: "ACK:down_attach\n"
    ↓
15. Pi receives on serial read loop
    ↓
16. Pi broadcasts WebSocket: {"kind":"ack","cmd":"down_attach"}
    ↓
17. Laptop receives and prints: "[ARDUINO REPLY] ACK:down_attach"
```

**Total Latency**: Typically 50-100 ms
- WiFi: ~5-15 ms
- Serial UART: ~1 ms
- Processing: ~5-10 ms
- Servo response: 30-50 ms (mechanical)

### Telemetry Flow

```
1. Arduino timer triggers (every 200 ms)
   ↓
2. readSensors() samples TOF sensors
   ↓
3. emitStatusLine() generates JSON
   ↓
4. Arduino writes: "STS:{...}\n"
   ↓
5. Pi receives on serial read loop
   ↓
6. Pi parses "STS:" prefix, extracts JSON
   ↓
7. Pi wraps: {"kind":"telemetry", ...original_payload...}
   ↓
8. Pi broadcasts to all WebSocket clients
   ↓
9. Laptop receives telemetry (could update dashboard)
   ↓
10. Dashboard displays speed, direction, pose, sensor readings, latch status
```

**Update Rate**: 5 Hz (every 200 ms)
- Sufficient for human monitoring
- Low enough to avoid saturating WiFi
- Matches typical UI refresh rates

### Error Propagation

**Example: Upper TOF Sensor Obstruction**

```
1. Arduino TOF read: raw_up = 80 mm (calibrated: ~70 mm)
   ↓
2. Pi software monitors telemetry: tof.up < THRESHOLD (e.g., 100 mm)
   ↓
3. Pi sends: "WARN:up\n"
   ↓
4. Arduino receives on serial
   ↓
5. parseAndExecuteWarning("WARN:up")
   ↓
6. stopMotors(), latched=true, latch_reason="up"
   ↓
7. Arduino emits: "WARNED:up\n"
   ↓
8. Pi broadcasts: {"kind":"warned","sensor":"up"}
   ↓
9. Laptop displays: "[WARNING] Upper sensor triggered - motors stopped"
   ↓
10. Operator investigates, clears obstacle
    ↓
11. Operator sends command (e.g., both_attach) to clear latch
    ↓
12. Arduino clears latch, resumes normal operation
```

### Network Topologies

#### Topology 1: Direct AP (Low Latency)

```
Laptop ─────> Pi AP ─────> Arduino
  WiFi         USB Serial
  ~10 ms       ~1 ms
  
Total: ~11 ms + processing
Bandwidth: 54 Mbps (802.11n)
Range: 10-30 meters (line of sight)
```

**Use Case**: Competition, demonstrations, indoor testing

#### Topology 2: Tailscale VPN (Remote)

```
Laptop ─────> Internet ─────> Pi (via Tailscale) ─────> Arduino
  LTE/WiFi    Variable latency    USB Serial
  20-200 ms                       ~1 ms
  
Total: 21-201 ms + processing
Bandwidth: 1-50 Mbps (depends on cellular/internet)
Range: Unlimited (global)
```

**Use Case**: Remote monitoring, development from home, cloud logging

#### Topology 3: Hybrid (Tailscale + Local AP)

```
Laptop ─────> Pi AP ─────> Arduino
  WiFi (local)  USB Serial
  ~10 ms        ~1 ms

Dashboard (remote) ──> Internet ──> Pi (via Tailscale) ──> (read-only)
                       Variable latency
```

**Use Case**: Local operator with remote monitoring/telemetry logging

### Configuration Management

#### Laptop Configuration Files

**~/.config/robot/config.yaml**:
```yaml
pi_hosts:
  - name: "AP Mode"
    host: "192.168.4.1"
    port: 81
    auth: false
  
  - name: "Tailscale"
    host: "100.x.y.z"
    port: 81
    auth: true
    token_env: "BRIDGE_AUTH_TOKEN"

default_host: "AP Mode"
send_rate_hz: 30
button_hold_frames: 1
```

#### Pi Configuration Files

**/etc/default/climbingrobot**:
```bash
# Serial port (auto-detected if not set)
# SERIAL_PORT=/dev/ttyACM0

# WebSocket port
WS_PORT=81

# Authentication (required for Tailscale)
BRIDGE_AUTH_TOKEN=your-token-here

# Test mode (no Arduino needed)
TEST_MODE=0

# Wait for Arduino on boot (-1 = wait forever)
WAIT_SERIAL=-1
```

**/etc/systemd/system/climbingrobot.service**:
```ini
[Unit]
Description=Climbing Robot Serial Bridge
After=network.target tailscaled.service
Wants=tailscaled.service

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/robot
EnvironmentFile=-/etc/default/climbingrobot
ExecStart=/usr/bin/python3 pi_serial_bridge.py \
    --wait-serial=${WAIT_SERIAL:--1} \
    --port=${WS_PORT:-81} \
    --auto-test
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

### Deployment Procedures

#### Initial Setup (New Pi)

```bash
# 1. Flash Raspberry Pi OS Lite to SD card
# 2. Enable SSH (touch /boot/ssh)
# 3. Boot Pi, SSH in

# 4. Install dependencies
sudo apt update
sudo apt install -y python3-pip python3-serial python3-websockets

# 5. Clone repository
cd ~
git clone https://github.com/your-org/climbing-robot.git robot
cd robot/raspi_bridge/pi

# 6. Run setup scripts
bash pi_install.sh        # System dependencies
bash pi_ap_setup.sh       # WiFi AP configuration
bash pi_auth_setup.sh     # SSH keys and security

# 7. Configure service
sudo cp climbingrobot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable climbingrobot
sudo systemctl start climbingrobot

# 8. Verify
sudo journalctl -u climbingrobot -f
```

#### Laptop Setup

```bash
# 1. Install Python dependencies
pip install pygame websockets

# 2. Clone repository
git clone https://github.com/your-org/climbing-robot.git
cd climbing-robot/raspi_bridge/laptop

# 3. Test connection
python3 gamepad_to_pi.py --host 192.168.4.1 --test

# 4. Run with gamepad
python3 gamepad_to_pi.py --host 192.168.4.1
```

#### Arduino Setup

```bash
# 1. Install Arduino IDE or arduino-cli
brew install arduino-cli  # macOS

# 2. Install libraries
arduino-cli lib install "Adafruit PWM Servo Driver Library"
arduino-cli lib install "VL53L0X"

# 3. Compile and upload
cd raspi_bridge/arduino/arduino_bridge
arduino-cli compile --fqbn arduino:avr:mega arduino_bridge.ino
arduino-cli upload -p /dev/cu.usbmodem* --fqbn arduino:avr:mega

# 4. Monitor serial output
arduino-cli monitor -p /dev/cu.usbmodem* -c baudrate=115200
```

### Troubleshooting Guide

#### Issue: Gamepad not detected

```bash
# Check SDL2 installation
python3 -c "import pygame; pygame.init(); print(pygame.joystick.get_count())"

# List connected controllers
python3 -c "import pygame; pygame.init(); pygame.joystick.init(); print([pygame.joystick.Joystick(i).get_name() for i in range(pygame.joystick.get_count())])"

# On Linux, check permissions
ls -l /dev/input/js*
sudo usermod -a -G input $USER  # Add user to input group
```

#### Issue: Cannot connect to Pi

```bash
# Check WiFi connection
ping 192.168.4.1

# Check Pi is running bridge
ssh pi@192.168.4.1
sudo systemctl status climbingrobot

# Check WebSocket port
nc -zv 192.168.4.1 81  # Should connect

# View bridge logs
sudo journalctl -u climbingrobot -n 100
```

#### Issue: Arduino not responding

```bash
# On Pi, check serial device
ls -l /dev/serial/by-id/
ls -l /dev/ttyACM*

# Check USB cable and power
lsusb | grep Arduino

# Monitor serial output directly
python3 -m serial.tools.miniterm /dev/ttyACM0 115200

# View bridge logs for serial errors
sudo journalctl -u climbingrobot | grep -i serial
```

#### Issue: Commands not executing

```bash
# Check telemetry is flowing
# In gamepad client, should see "STS:{...}" messages

# Check for latch state
# In telemetry: "latched":true means system is stopped

# Send command to clear latch
echo "0,FORWARD,both_attach" | websocat ws://192.168.4.1:81/bus

# Monitor Arduino serial directly
# Should see "ACK:<command>" after sending commands
```

#### Issue: High latency / packet loss

```bash
# Check WiFi signal strength
iw dev wlan0 link  # On Pi
# (or use WiFi menu on laptop)

# Ping test
ping -i 0.033 192.168.4.1  # 30 Hz rate
# Should see <20 ms latency, 0% loss

# Check for interference
sudo iwlist wlan0 scan | grep -E "ESSID|Channel"

# Switch WiFi channel if congested
# Edit /etc/hostapd/hostapd.conf, change channel=X
```

---

## Performance Characteristics

### Latency Budget

| Component | Typical | Worst Case | Notes |
|-----------|---------|------------|-------|
| Gamepad polling | 16 ms | 33 ms | SDL2 event loop |
| Laptop processing | 1 ms | 5 ms | Python overhead |
| WiFi (AP mode) | 5 ms | 20 ms | 802.11n, low congestion |
| WiFi (Tailscale) | 50 ms | 200 ms | Internet routing |
| Pi WebSocket | 1 ms | 5 ms | Async I/O |
| Pi serial write | 0.2 ms | 2 ms | 115200 baud, ~40 bytes |
| Arduino parsing | 0.5 ms | 2 ms | String processing |
| Servo response | 20 ms | 60 ms | Mechanical movement |
| **Total (AP)** | **44 ms** | **127 ms** | Acceptable for teleoperation |
| **Total (Tailscale)** | **89 ms** | **307 ms** | Monitoring only |

### Throughput

**Command Rate**: 30 Hz (one command every 33 ms)
- Limited by gamepad polling, not network
- Serial link can handle >1000 commands/second

**Telemetry Rate**: 5 Hz (one status message every 200 ms)
- JSON payload: ~150 bytes
- Bandwidth: 750 bytes/second = 6 kbps
- Negligible compared to WiFi capacity (54 Mbps)

**Multi-Client Capacity**:
- Pi CPU: <5% utilization with 3 clients
- Network: Each client adds ~6 kbps telemetry
- Practical limit: ~10 clients (more limited by Pi WiFi than CPU)

### Power Consumption

| Component | Idle | Active | Peak | Notes |
|-----------|------|--------|------|-------|
| Pi Zero 2 W | 0.4 W | 0.8 W | 1.5 W | WiFi TX |
| Arduino Mega | 0.2 W | 0.3 W | 0.5 W | USB powered |
| Motors (2×) | 0 W | 12 W | 24 W | 12V @ 1A each |
| Servos (8×) | 0.4 W | 2 W | 8 W | 6V @ 0.5A each stall |
| TOF sensors (2×) | 0.04 W | 0.04 W | 0.04 W | Negligible |
| **Total (idle)** | **1 W** | - | - | Motors off, servos hold |
| **Total (driving)** | - | **15 W** | **34 W** | All active |

**Battery Life Estimates** (12V 5000 mAh LiPo):
- Idle: ~60 Wh / 1 W = 60 hours
- Typical operation: ~60 Wh / 15 W = 4 hours
- Continuous climb: ~60 Wh / 34 W = 1.8 hours

---

## Appendix: Complete Protocol Reference

### CSV Command Format (Laptop → Pi → Arduino)

```
<speed>,<direction>,<command>,<altitude_mode>\n
```

**Field 1: Speed** (0-255)
- PWM duty cycle for motor controller
- 0 = stopped, 255 = full power
- Typical range: 100-200 (40-80% power)

**Field 2: Direction** ("FORWARD" | "BACKWARD")
- "FORWARD": Climbing upward
- "BACKWARD": Descending

**Field 3: Command** (action identifier)
- "NONE": No action
- "up_attach": Close upper gripper
- "up_detach": Open upper gripper
- "down_attach": Close lower gripper
- "down_detach": Open lower gripper
- "both_attach": Close both grippers
- "both_detach": Open both grippers
- "estop": Emergency stop

**Field 4: Altitude Mode** (optional, defaults to "manual")
- "manual": Direct motor control (default)
- "hold_alt": PID holds current altitude
- "zero_alt": Reset altitude reference
- "set_alt": Run to target altitude (future)

### JSON Telemetry Format (Arduino → Pi → Laptop)

```json
{
  "kind": "telemetry",
  "speed": 150,
  "dir": "FORWARD",
  "pose": "parallel",
  "ack": "NONE",
  "latched": false,
  "tof": {
    "enabled": true,
    "up": 350,
    "down": 120
  }
}
```

**Optional fields when latched:**
```json
{
  "latched": true,
  "latch_reason": "estop"
}
```

### Warning Format (Pi → Arduino)

```
WARN:<sensor>\n
```

**Valid sensors:**
- "up": Upper TOF sensor triggered
- "down": Lower TOF sensor triggered
- "tilt": IMU excessive tilt (ESP32 revamped only)

### Acknowledgment Format (Arduino → Pi → Laptop)

```json
{"kind": "ack", "cmd": "up_attach"}
```

---

## Document Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-09-09 | System | Initial comprehensive documentation |

---

**End of System Architecture Documentation**
