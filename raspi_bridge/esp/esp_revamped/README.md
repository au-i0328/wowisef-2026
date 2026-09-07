# ESP32-S3 Altitude Control Bridge

**Version:** 1.0.0  
**Date:** September 5, 2026  
**File:** `esp_revamped.ino` (847 lines)

## Overview

Port of `esp_bridge.ino` for ESP32-S3 with added autonomous altitude control features. Maintains full compatibility with the original esp_bridge serial protocol while adding BNO055 IMU and VL53L1X ground sensor for altitude management.

**All original motor control, servo sequencing, and bar-pose logic preserved exactly as in esp_bridge.ino. Only additions: altitude control modes and new sensor readings.**

## What Changed from esp_bridge.ino

### Preserved (Unchanged)
- All motor control logic (setupMotorPWM, stopMotors, setMotors)
- VL53L0X TOF sensor initialization and reading (tof_up, tof_down)
- All servo control functions (up_attach, down_attach, setBarPosition, etc.)
- Command parsing and execution flow (parseAndExecutePayload, executeCommand)
- Safety latch mechanism (WARN: handling, estop)
- LED flash behavior
- Serial protocol structure (STS: and ACK: lines)

### Added (New Features)
1. **BNO055 IMU** for pitch/roll/heading measurement
2. **VL53L1X ground sensor** for altitude measurement
3. **Three altitude control modes:**
   - `MODE_MANUAL` - original manual control (default)
   - `MODE_HOLD_ALTITUDE` - maintain current height
   - `MODE_RUN_TO_ALTITUDE` - climb to target height
4. **New commands:**
   - `set_alt` - switch to run-to-altitude mode
   - `hold_alt` - lock current altitude
   - `zero_alt` - reset altitude reference
   - `manual` - return to manual control
5. **Extended JSON status** with altitude, pitch, roll, mode
6. **PID controller** for altitude stabilization
7. **Tilt-compensated height calculation**

### Key Behavior
- In `MODE_MANUAL`: motors controlled by serial commands (original behavior)
- In altitude modes: motors controlled by PID loop, serial speed/direction ignored
- All servo commands work in any mode
- Safety latches work identically to original

## Hardware Architecture

### Controller
- **ESP32-S3** microcontroller
- ⚠️ **Critical:** GPIOs 33-37 are RESERVED for internal OPI PSRAM and cannot be used

### Motor Driver
- **XY160D** dual-channel DC motor driver
- Upper Motor: PWM ENA (GPIO 32), IN1 (GPIO 27), IN2 (GPIO 14)
- Lower Motor: PWM ENB (GPIO 12), IN3 (GPIO 13), IN4 (GPIO 15)

### IMU Sensor
- **Stemma QT BNO055** absolute orientation sensor
- I2C address: `0x28` (fixed hardware address)
- Provides pitch and roll angles for tilt compensation

### Distance Sensors

| Sensor | Purpose | XSHUT Pin | I2C Address (Final) |
|--------|---------|-----------|---------------------|
| VL53L1X | Ground-facing altitude measurement | Hardwired to 3.3V (always-on) | 0x32 |
| VL53L0X | Upper beam proximity (clamp detection) | GPIO 25 | 0x30 |
| VL53L0X | Lower beam proximity (clamp detection) | GPIO 26 | 0x31 |

## Sensor Initialization Protocol

The "Always-ON First" boot sequence prevents I2C address conflicts:

1. **Pull both GPIO_XSHUT1 and GPIO_XSHUT2 LOW** → Turn off both VL53L0X sensors
2. **Initialize VL53L1X** at default address 0x29 (only active device)
3. **Re-address VL53L1X** in software to 0x32
4. **Configure VL53L1X ROI** to 4×4 array (narrows FOV to ~15° to prevent beam reflections)
5. **Set GPIO_XSHUT1 HIGH** → Boot Upper VL53L0X at 0x29, re-address to 0x30
6. **Set GPIO_XSHUT2 HIGH** → Boot Lower VL53L0X at 0x29, re-address to 0x31
7. **Verify BNO055** is active at 0x28

## Physics & Sensor Processing

### Exponential Moving Average (EMA) Filter
```cpp
alpha = 0.25
filtered = alpha * new_reading + (1 - alpha) * old_filtered
```
Eliminates laser jitter from ground distance sensor.

### Tilt-Compensated True Vertical Height
```
h_true = d_measured * cos(pitch_radians) * cos(roll_radians)
```
Corrects for robot tilt to calculate accurate vertical displacement.

### Safety Interlock
If `abs(pitch) > 15.0°` OR `abs(roll) > 15.0°`:
- **Immediate emergency motor halt**
- Prevents falls due to excessive tilt

## Control System

### Control Loop
- **Frequency:** 30 Hz (33ms cycle time)
- **Non-blocking:** State machine architecture

### Operational Modes

#### Mode 1: Run to Set Altitude
- **Command:** `s <target_mm>`
- **Behavior:** Move toward target height using asymmetric directional power
  - **Climbing UP** (against gravity): PWM = 155
  - **Descending DOWN** (with gravity): PWM = 45
- **Transition:** When altitude error enters ±3.0 mm deadband, automatically switch to Mode 2

#### Mode 2: Hold Altitude
- **Command:** `h`
- **Behavior:** State-aware hover bias based on clamp detection
  - **Both wheels clamped** (beam proximity < 20mm): PWM = 0 (passive friction)
  - **Only upper attached**: Upper PWM = 80, Lower PWM = 0
  - **Only lower attached**: Upper PWM = 0, Lower PWM = 85

#### Emergency Stop Mode
- **Command:** `e`
- **Triggers:**
  - Manual emergency stop command
  - Tilt exceeds 15.0° in any axis
- **Behavior:** All motors stop, LED on, system latched until reset

#### Idle Mode
- **Default state:** Motors off, waiting for commands

## Serial Protocol

### Command Format (CSV)

Primary protocol compatible with `esp_bridge.ino`:

```
<speed>,<direction>,<command>
```

**Parameters:**
- `speed`: Integer 0-255 (currently unused in autonomous mode, send 0)
- `direction`: `UP`, `DOWN`, or `STOP`
- `command`: One of:
  - `NONE` - No action
  - `set_alt` - Set target altitude (use legacy format for value)
  - `hold` - Hold current altitude
  - `zero` - Zero ground sensor calibration
  - `estop` - Emergency stop
  - `idle` - Return to idle mode

**Examples:**
```
0,STOP,hold        # Hold current altitude
0,STOP,zero        # Zero ground sensor
0,STOP,estop       # Emergency stop
0,STOP,NONE        # No operation
```

### Warning Protocol

External systems can trigger emergency stops:

```
WARN:<sensor>
```

**Sensors:**
- `WARN:upper` - Upper beam warning
- `WARN:lower` - Lower beam warning
- `WARN:tilt` - Tilt warning

**Response:** Immediate motor halt, system latched, outputs `WARNED:<sensor>`

### Status Output (JSON)

Every **200 ms**, the ESP32 outputs:

```
STS:{<json_object>}
```

**Fields:**
```json
{
  "mode": "RUN_TO_ALTITUDE|HOLD_ALTITUDE|IDLE|EMERGENCY_STOP",
  "target_alt": 500.0,
  "true_height": 498.23,
  "ground_raw": 520.5,
  "ground_filt": 519.8,
  "pitch": -0.34,
  "roll": 1.21,
  "upper_pwm": 155,
  "lower_pwm": 155,
  "dir": "UP|DOWN|STOP",
  "ack": "hold",
  "latched": false,
  "latch_reason": "",
  "sensors": {
    "upper_beam": 45,
    "lower_beam": 8191,
    "upper_clamped": false,
    "lower_clamped": false
  }
}
```

### Command Acknowledgment

After executing a command:

```
ACK:<command_name>
```

**Examples:**
```
ACK:hold
ACK:zero
ACK:estop
ACK:set_alt 500.0
```

### Error Messages

```
ERR:malformed CSV       # Invalid CSV format
ERR:field length        # Field too long
ERR:non-numeric speed   # Speed not a number
ERR:invalid direction   # Direction not UP/DOWN/STOP
ERR:line too long       # Input > 128 characters
ERR:latched             # System latched, cannot execute
ERR:unknown command     # Command not recognized
```

## Legacy Commands (Testing/Debugging)

For manual testing via Serial Monitor:

| Command | Description | Example |
|---------|-------------|---------|
| `s <mm>` | Set target altitude in millimeters | `s 500` |
| `h` | Hold current altitude | `h` |
| `z` | Zero ground sensor at current position | `z` |
| `e` | Emergency stop (press again to reset) | `e` |
| `?` | Display help message | `?` |

| Command | Description | Example |
|---------|-------------|---------|
| `s <mm>` | Set target altitude in millimeters | `s 500` |
| `h` | Hold current altitude | `h` |
| `z` | Zero ground sensor at current position | `z` |
| `e` | Emergency stop / reset (toggle) | `e` |
| `?` | Show help menu | `?` |

### Serial Configuration
- **Baud rate:** 115200
- **Line termination:** Newline (`\n`)
- **Commands:** Case-insensitive

## PWM Configuration

| Parameter | Value |
|-----------|-------|
| PWM Frequency | 5000 Hz |
| PWM Resolution | 8-bit (0-255) |
| Climb UP PWM | 155 |
| Descend DOWN PWM | 45 |
| Hold Upper Only | 80 |
| Hold Lower Only | 85 |

### Tuning Parameters (Configurable)

```cpp
// In code (global variables)
float altitudeDeadband_mm = 3.0f;          // ±3mm deadband for target
const float CLAMP_THRESHOLD_MM = 20.0f;    // Beam proximity threshold
const float EMA_ALPHA = 0.25f;              // Filter responsiveness
const float MAX_TILT_DEG = 15.0f;          // Safety tilt limit
```

## Code Structure

The firmware is organized into modular sections:

1. **Pin Configuration** - Hardware pin mapping
2. **Sensor Objects** - I2C device instances
3. **State Variables** - Physics, control, and status data
4. **Motor Control** - PWM and direction functions
5. **Sensor Initialization** - Boot sequence protocol
6. **Sensor Reading** - Non-blocking sensor updates
7. **Physics Calculations** - Tilt compensation and filtering
8. **Clamp Detection** - Beam proximity logic
9. **Control Modes** - Run-to-altitude and hold-altitude
10. **Serial Interface** - Command parser and help
11. **Setup & Loop** - Main program flow

## Safety Features

✅ **Out-of-range sensor status codes handled cleanly**  
✅ **Bounds checking on all PWM values** (0-255 clamping)  
✅ **Tilt safety interlock** (15° threshold)  
✅ **Emergency stop with latch** (manual reset required)  
✅ **Sensor initialization verification** (system halts on failure)  
✅ **Non-blocking control loop** (prevents watchdog timeouts)

## Status Output

Every 500ms, the system prints comprehensive telemetry:

```
========== STATUS ==========
Ground Distance (raw): 523.4 mm
Ground Distance (filtered): 521.8 mm
Ground Zero Offset: 0.0 mm
Pitch: -2.34°, Roll: 1.12°
True Vertical Height: 518.92 mm
Upper Beam Distance: 18 mm (CLAMPED)
Lower Beam Distance: 8192 mm (free)
Mode: RUN_TO_ALTITUDE (target: 500.0 mm, error: -18.9 mm)
Motor Upper PWM: 155, Lower PWM: 155, Direction: UP
============================
```

## Testing Procedure

### 1. Initial Setup
```
> z              // Zero sensor on ground
Ground sensor zeroed at: 245.3 mm
```

### 2. Test Run to Altitude
```
> s 500          // Climb to 500mm
Target altitude set to: 500.0 mm
// Robot climbs...
Target altitude reached, switching to HOLD mode
```

### 3. Test Hold Mode
```
> h              // Hold current position
Holding current altitude
```

### 4. Test Emergency Stop
```
> e              // Trigger e-stop
EMERGENCY STOP TRIGGERED!
Manual e-stop
> e              // Reset
Emergency stop reset - system in IDLE mode
```

## Required Arduino Libraries

Install via Arduino Library Manager:

1. **Wire** (built-in) - I2C communication
2. **Adafruit_Sensor** - Unified sensor library
3. **Adafruit_BNO055** - IMU driver
4. **VL53L0X** by Pololu - ToF sensor (short range)
5. **VL53L1X** by Pololu - ToF sensor (long range)

## Compilation

**Board:** ESP32-S3 Dev Module  
**Flash Size:** 8MB (or as appropriate for your board)  
**PSRAM:** OPI PSRAM (enabled)  
**Upload Speed:** 921600

## Pin Summary Table

| Function | GPIO | Notes |
|----------|------|-------|
| I2C SDA | 21 | Default ESP32 I2C |
| I2C SCL | 22 | Default ESP32 I2C |
| Upper Motor EN | 32 | PWM channel 0 |
| Upper Motor IN1 | 27 | Direction control |
| Upper Motor IN2 | 14 | Direction control |
| Lower Motor EN | 12 | PWM channel 1 |
| Lower Motor IN3 | 13 | Direction control |
| Lower Motor IN4 | 15 | Direction control |
| Upper ToF XSHUT | 25 | VL53L0X reset |
| Lower ToF XSHUT | 26 | VL53L0X reset |
| Status LED | 2 | Onboard LED |
| **RESERVED** | **33-37** | **OPI PSRAM - DO NOT USE** |

## Authors

ESP32-S3 Robotics Team  
ISEF Project 2025-2026

## License

Educational/Research Use
