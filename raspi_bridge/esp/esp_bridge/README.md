# ESP32 Bridge Firmware Technical Documentation

## Overview

The ESP32 Bridge firmware is a port of the Arduino bridge controller, designed to run on ESP32-S3 microcontrollers. It provides motor control, servo management, and optional time-of-flight (TOF) distance sensing for a climbing robot system. The firmware acts as a bridge between a Raspberry Pi control system and the robot's physical actuators.

## System Architecture

### Hardware Platform

- **Microcontroller**: ESP32-S3
- **Communication**: Serial UART (115200 baud)
- **I2C Bus**: SDA=GPIO21, SCL=GPIO22
- **Status Indicator**: Onboard LED (GPIO2)



### Key Components

1. **Motor Driver (XY160D Dual H-Bridge)**
  - Controls two DC motors for robot locomotion
  - Independent speed and direction control
  - PWM-based speed regulation (5 kHz, 8-bit resolution)
2. **Servo Controller (PCA9685)**
  - 16-channel I2C servo driver
  - Controls 8 servos: 4 grippers + 4 bar actuators
  - 50 Hz PWM frequency for standard servos
3. **Distance Sensors (Optional VL53L0X TOF)**
  - Two sensors: upper and lower obstacle detection
  - I2C communication with XSHUT-based re-addressing
  - Continuous ranging mode with calibration



## Pin Configuration



### Motor Control (L298N)

```
Motor A:
  EN  → GPIO32 (PWM Channel 0)
  IN1 → GPIO33 (Direction)
  IN2 → GPIO27 (Direction)

Motor B:
  EN  → GPIO14 (PWM Channel 1)
  IN1 → GPIO12 (Direction)
  IN2 → GPIO13 (Direction)
```



### I2C Devices

```
SDA → GPIO21
SCL → GPIO22

Connected devices:
  - PCA9685 Servo Hub (0x40)
  - VL53L0X Upper TOF (0x30) *
  - VL53L0X Lower TOF (0x31) *
  
* Optional, controlled via TOF_ENABLED flag
```



### TOF Sensor Control

```
Upper TOF XSHUT  → GPIO25
Lower TOF XSHUT  → GPIO26
```



### Status LED

```
LED → GPIO2 (Onboard LED)
```



## Communication Protocol



### Serial Configuration

- **Baud Rate**: 115200
- **Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Timeout**: 10 ms
- **Line Termination**: `\n` (newline)



### Inbound Messages (Pi → ESP32)



#### Drive Command Format

```
<speed>,<direction>,<command>\n
```

**Fields:**

- `speed`: Integer 0-255 (PWM duty cycle)
- `direction`: String "FORWARD" or "BACKWARD"
- `command`: Action command or "NONE"

**Example:**

```
150,FORWARD,NONE\n        # Drive forward at speed 150
0,FORWARD,up_attach\n     # Stop motors and attach upper gripper
```



#### Warning Format

```
WARN:<sensor>\n
```

**Triggers emergency stop and latches the system.**

**Valid sensors:**

- `up` - Upper TOF sensor triggered
- `down` - Lower TOF sensor triggered

**Example:**

```
WARN:up\n    # Stop immediately, latch with reason "up"
```



### Outbound Messages (ESP32 → Pi)



#### Status Line (Periodic, 200 ms)

```json
STS:{"speed":150,"dir":"FORWARD","pose":"parallel","ack":"NONE","latched":false,"tof":{"enabled":false,"up":65535,"down":65535}}
```

**Fields:**

- `speed`: Current motor speed (0-255)
- `dir`: Current direction ("FORWARD", "BACKWARD", "STOP")
- `pose`: Current bar configuration ("parallel", "up_detach", "down_detach")
- `ack`: Last acknowledged command
- `latched`: Emergency stop state (boolean)
- `latch_reason`: Reason for latch (present only if latched)
- `tof.enabled`: TOF sensor system status (boolean)
- `tof.up`: Upper sensor reading in mm (65535 = sensor down)
- `tof.down`: Lower sensor reading in mm (65535 = sensor down)



#### Command Acknowledgment (Event-Driven)

```
ACK:<command>\n
```

Emitted immediately after executing a command.

**Example:**

```
ACK:up_attach
ACK:estop
```



#### Warning Echo (Debug)

```
WARNED:<sensor>\n
```

Confirms receipt of warning signal.

## Motor Control System



### PWM Configuration

```cpp
Frequency: 5000 Hz
Resolution: 8-bit (0-255)
Channels: 0 (Motor A), 1 (Motor B)
```



### Direction Control

**Forward Motion:**

```
Motor A: IN1=HIGH, IN2=LOW
Motor B: IN1=HIGH, IN2=LOW
```

**Backward Motion:**

```
Motor A: IN1=LOW, IN2=HIGH
Motor B: IN1=LOW, IN2=HIGH
```

**Stop:**

```
Both Motors: IN1=LOW, IN2=LOW, PWM=0
```



### Motor State Machine

```
┌─────────┐
│  STOP   │ ◄─────── Initial state
└────┬────┘
     │
     ├──── setMotors(speed, FORWARD) ──┐
     │                                   ▼
     │                            ┌───────────┐
     │                            │  FORWARD  │
     │                            └─────┬─────┘
     │                                  │
     ├──── setMotors(speed, BACKWARD) ─┤
     │                                  ▼
     │                            ┌───────────┐
     │                            │ BACKWARD  │
     │                            └─────┬─────┘
     │                                  │
     └──── setMotors(0, STOP) ─────────┘
```



## Servo Control System



### Servo Hub Configuration

**PCA9685 Servo Driver**

- I2C Address: 0x40
- PWM Frequency: 50 Hz
- Pulse Width Range: 1000-2000 μs (standard servos)



### Channel Assignment

```
Grippers:
  Upper Left   → Channel 13
  Upper Right  → Channel 12
  Lower Left   → Channel 9
  Lower Right  → Channel 8

Bar Actuators:
  Upper Left   → Channel 15
  Upper Right  → Channel 14
  Lower Left   → Channel 11
  Lower Right  → Channel 10
```



### Position Definitions

```cpp
Open Position:  180° (grippers open)
Close Position:  27° (grippers closed)
Angle Change:    35° (bar angle adjustment)
```



### Bar Pose States



#### Parallel Configuration

```
All bars at 150° (neutral position)
Use case: Both grippers attached or transitioning
```



#### Up-Detach Configuration

```
Upper bars:  150° (neutral)
Lower bars:  115° / 185° (angled outward)
Use case: Upper gripper released, preparing to climb
```



#### Down-Detach Configuration

```
Upper bars:  185° / 115° (angled outward)
Lower bars:  150° (neutral)
Use case: Lower gripper released, preparing to descend
```



### Servo Angle Conversion

The firmware uses two conversion functions:

**Standard Servos (0-180°):**

```cpp
angleToPulse(angle, 0, 180, 1000, 2000)
// Returns pulse width in microseconds
```

**Extended Range Servos (0-300°):**

```cpp
angleToPulse2(angle, 0, 300, 500, 2500)
// Returns pulse width in microseconds
```



## TOF Sensor System



### Sensor Configuration

**VL53L0X Time-of-Flight Sensors**

- Measurement Mode: Continuous ranging
- Timing Budget: 10,000 μs (configurable)
- Range: 30-1200 mm (typical)
- Update Rate: ~50 Hz



### Initialization Sequence

1. **Power Down Both Sensors**
  ```cpp
   digitalWrite(TOF_UP_XSHUT, LOW);
   digitalWrite(TOF_DOWN_XSHUT, LOW);
   delay(50);
  ```
2. **Initialize Upper Sensor**
  ```cpp
   digitalWrite(TOF_UP_XSHUT, HIGH);
   delay(50);
   tof_up.init();
   tof_up.setAddress(0x30);  // Re-address to avoid conflict
   tof_up.startContinuous();
  ```
3. **Initialize Lower Sensor**
  ```cpp
   digitalWrite(TOF_DOWN_XSHUT, HIGH);
   delay(50);
   tof_down.init();
   tof_down.setAddress(0x31);  // Different address
   tof_down.startContinuous();
  ```



### Calibration

Both sensors use linear calibration:

```
Calibrated_Value = slope × Raw_Value + offset
```

**Upper Sensor:**

```cpp
slope  = 30.0 / 31.0
offset = 70.0 - (30.0/31.0) × 87.0
```

**Lower Sensor:**

```cpp
slope  = 30.0 / 35.0
offset = 70.0 - (30.0/35.0) × 135.0
```



### Error Handling

**Sensor-Down Detection:**

- Reading = 0xFFFF (65535)
- Reading = 0
- Timeout occurred

**Recovery Strategy:**

1. Detect sensor failure (reading = 0xFFFF)
2. Wait 5 seconds before retry
3. Re-initialize failed sensor via XSHUT cycle
4. Resume continuous ranging
5. If failed again, wait another 5 seconds



### Throttling

Sensor reads are throttled to prevent I2C bus congestion:

```cpp
Minimum read interval: 50 ms
Watchdog check interval: 1000 ms
```



## Safety Systems



### Emergency Stop (E-Stop)

**Trigger Conditions:**

1. Command: `estop` received via serial
2. Warning: `WARN:<sensor>` received

**E-Stop Actions:**

```cpp
1. stopMotors()           // Halt all drive motors
2. digitalWrite(LED_PIN, LOW)  // Turn off status LED
3. latched = true         // Enter latched state
4. latch_reason = <cause> // Record trigger cause
```



### Latch Mechanism

Once latched, the system ignores all drive commands except:

- New command (non-NONE) clears the latch
- System can be re-enabled by any valid action command

**Latch Clear Logic:**

```cpp
if (latched && commandStr != "NONE") {
    latched = false;
    latch_reason[0] = '\0';
    // Resume normal operation
}
```



### TOF Safety Integration

When `TOF_ENABLED = true`:

- Upper sensor triggers warning if obstacle detected above
- Lower sensor triggers warning if ground proximity critical
- Warning threshold determined by Pi-side logic
- ESP32 responds immediately to `WARN:` messages



## Command Reference



### Drive Commands


| Command       | Description           | Bar Pose      | Gripper Action | Timing    |
| ------------- | --------------------- | ------------- | -------------- | --------- |
| `NONE`        | No action             | No change     | No change      | Immediate |
| `up_detach`   | Release upper gripper | → up_detach   | Upper open     | 1500 ms   |
| `down_detach` | Release lower gripper | → down_detach | Lower open     | 1500 ms   |
| `up_attach`   | Engage upper gripper  | → parallel    | Both close     | 1500 ms   |
| `down_attach` | Engage lower gripper  | → parallel    | Both close     | 1500 ms   |
| `both_attach` | Engage both grippers  | → parallel    | Both close     | 1500 ms   |
| `both_detach` | Release both grippers | → parallel    | Both open      | 1500 ms   |
| `estop`       | Emergency stop        | No change     | No change      | Immediate |




### Command Sequencing

**Example: Climb Sequence**

```
1. 0,FORWARD,both_attach     # Ensure both grippers closed
2. 150,FORWARD,up_detach     # Drive up while releasing upper gripper
3. 0,FORWARD,up_attach       # Stop and attach upper gripper
4. 150,FORWARD,down_detach   # Drive up while releasing lower gripper
5. 0,FORWARD,down_attach     # Stop and attach lower gripper
6. (repeat from step 2)
```



## LED Indication System



### LED States


| State          | Pattern      | Meaning               |
| -------------- | ------------ | --------------------- |
| Motors stopped | Solid OFF    | System idle           |
| Motors running | Flash 150 ms | Drive active          |
| Latched        | Solid OFF    | Emergency stop active |
| Initializing   | Solid ON     | Startup phase         |




### Implementation

```cpp
if (current_speed > 0) {
    // Flash every 150 ms when driving
    if (millis() - led_flash_timer >= 150) {
        led_state = !led_state;
        digitalWrite(LED_PIN, led_state ? HIGH : LOW);
        led_flash_timer = millis();
    }
} else {
    // Solid OFF when stopped
    digitalWrite(LED_PIN, LOW);
}
```



## Timing and Performance



### Loop Cycle Performance


| Task                   | Frequency            | Priority |
| ---------------------- | -------------------- | -------- |
| Serial command parsing | As received          | Critical |
| Status line emission   | 200 ms               | High     |
| TOF sensor reading     | 50 ms minimum        | Medium   |
| Sensor watchdog        | 1000 ms              | Low      |
| LED update             | 150 ms (when active) | Low      |




### Blocking Operations

**Servo movements block execution:**

```cpp
delay_to_pose = 1500 ms  // Command functions block for this duration
```

**Impact:**

- Status lines pause during servo moves
- Serial commands queue (10 ms timeout)
- No sensor reads during servo transitions



### Serial Input Handling

```cpp
Timeout: 10 ms
Max line length: 128 characters
Buffer: String-based (dynamic allocation)
```



## Configuration Constants



### Configurable Parameters

```cpp
// TOF System
const bool TOF_ENABLED = false;  // Master enable/disable
const int TOF_MEASUREMENT_TIME = 10000;  // Timing budget (μs)
const long TOF_RETRY_AFTER_MS = 5000;  // Sensor recovery interval

// Motor PWM
#define PWM_FREQ 5000  // PWM frequency (Hz)
#define PWM_RESOLUTION 8  // PWM resolution (bits)

// Status Export
const long STATUS_PERIOD_MS = 200;  // Status line interval

// Servo Timing
const unsigned long delay_to_pose = 1500;  // Servo movement time (ms)

// Sensor Throttling
const unsigned long SENSOR_READ_TIMEOUT_MS = 50;  // Min read interval
```



### Servo Positions

```cpp
const float open_position = 180;    // Gripper open angle
const float close_position = 27;    // Gripper close angle
const float angle_change = 35;      // Bar angle adjustment
```



## Initialization Sequence



### Startup Flow

```
1. Serial.begin(115200)
2. pinMode(LED_PIN, OUTPUT) → HIGH
3. setupMotorPWM()
4. stopMotors()
5. Wire.begin(I2C_SDA, I2C_SCL)
6. servo_hub.begin()
7. servo_hub.reset()
8. servo_hub.setPWMFreq(50)
9. initTOF() → returns bitmask
10. Set tof_up_ready, tof_down_ready flags
11. Configure timing budgets
12. setBarPosition("parallel")
13. Close all grippers (initialize to safe state)
14. Serial.println("ESP32 Bridge Ready")
```



### Initialization Diagnostics

```
INIT: tof up=OK down=OK     # Both sensors initialized
INIT: tof up=FAIL down=OK   # Upper sensor failed
INIT: tof up=OK down=FAIL   # Lower sensor failed
INIT: tof up=FAIL down=FAIL # Both sensors failed
```



## Error Handling



### Error Messages


| Message                     | Cause                          | Recovery       |
| --------------------------- | ------------------------------ | -------------- |
| `ERR:line too long`         | Serial input > 128 chars       | Discard line   |
| `ERR:malformed CSV`         | Missing commas                 | Discard line   |
| `ERR:field length`          | Field validation failed        | Discard line   |
| `ERR:non-numeric speed`     | Invalid speed value            | Discard line   |
| `ERR:invalid direction`     | Direction not FORWARD/BACKWARD | Discard line   |
| `ERR:tof disabled:<sensor>` | TOF warning when disabled      | Ignore warning |




### Fault Tolerance

**Serial Communication:**

- Invalid commands → error message, continue operation
- Buffer overflow → discard, continue operation
- No loss of motor/servo state on serial error

**TOF Sensors:**

- Sensor failure → automatic retry after 5 seconds
- Failed sensor → 0xFFFF reading, non-blocking
- System operates normally without TOF data

**I2C Bus:**

- No explicit timeout handling
- Servo hub errors not monitored
- Sensor errors detected via timeout flags



## Memory Considerations



### RAM Usage Estimates

```
Static allocations:
  - VL53L0X objects (2×):       ~800 bytes
  - Servo hub object:           ~100 bytes
  - String buffers:             ~200 bytes
  - State variables:            ~50 bytes
  Total static:                 ~1150 bytes

Dynamic allocations:
  - Serial String buffers:      Variable (max 128 bytes)
  - I2C transfer buffers:       Handled by Wire library
```



### Flash Usage

```
Code size (typical):           ~50-70 KB
Libraries:
  - Arduino core:              ~30 KB
  - Wire:                      ~5 KB
  - Adafruit_PWMServoDriver:   ~8 KB
  - VL53L0X:                   ~15 KB
Total flash:                   ~58-78 KB
```



## Debugging and Diagnostics



### Serial Debug Output

**Initialization:**

```
INIT: tof up=OK down=OK
ESP32 Bridge Ready
```

**TOF Recovery:**

```
TOF: up re-init OK
TOF: down re-init FAIL
```

**Warning Handling:**

```
WARNED:up
WARNED:down
```



### Recommended Test Sequence

1. **Power-On Test**
  ```
   Expected: "ESP32 Bridge Ready" message
   LED should be solid ON initially
  ```
2. **Motor Test**
  ```
   Send: 100,FORWARD,NONE
   Expected: Motors spin forward, LED flashes
   Status: {"speed":100,"dir":"FORWARD",...}
  ```
3. **Servo Test**
  ```
   Send: 0,FORWARD,both_detach
   Expected: All grippers open, 1.5s delay
   Status: {"ack":"both_detach",...}
  ```
4. **E-Stop Test**
  ```
   Send: 150,FORWARD,NONE  (start motors)
   Send: 0,FORWARD,estop
   Expected: Motors stop immediately
   Status: {"latched":true,"latch_reason":"estop"}
  ```
5. **Latch Clear Test**
  ```
   Send: 0,FORWARD,both_attach
   Expected: Latch clears, servos move
   Status: {"latched":false,...}
  ```



## Differences from Arduino Version



### Hardware-Specific Changes

1. **PWM System**
  - Arduino: `analogWrite()` with Timer registers
  - ESP32: `ledcSetup()` / `ledcWrite()` API
2. **I2C Pins**
  - Arduino: A4 (SDA), A5 (SCL)
  - ESP32: GPIO21 (SDA), GPIO22 (SCL)
3. **Pin Assignments**
  - Completely remapped for ESP32-S3 GPIO layout
  - Uses high-current capable pins for motors



### Software Changes

1. **Serial Timeout**
  - Explicit `Serial.setTimeout(10)` call
2. **LED Flash Rate**
  - Changed from 500 ms to 150 ms for faster visual feedback
3. **Servo Timing**
  - Increased from 750 ms to 1500 ms for smoother motion



## Integration Guidelines



### Connecting to Raspberry Pi

**Wiring:**

```
ESP32 TX → Pi RX (GPIO15)
ESP32 RX → Pi TX (GPIO14)
ESP32 GND → Pi GND
```

**Pi-Side Serial Configuration:**

```python
import serial
ser = serial.Serial(
    port='/dev/serial0',  # or '/dev/ttyAMA0'
    baudrate=115200,
    timeout=0.1
)
```



### Sample Python Control Code

```python
def send_command(speed, direction, command):
    """Send CSV command to ESP32"""
    msg = f"{speed},{direction},{command}\n"
    ser.write(msg.encode('utf-8'))

def read_status():
    """Parse JSON status line"""
    line = ser.readline().decode('utf-8').strip()
    if line.startswith('STS:'):
        import json
        return json.loads(line[4:])  # Remove 'STS:' prefix
    return None

# Example usage
send_command(150, "FORWARD", "NONE")
status = read_status()
if status:
    print(f"Speed: {status['speed']}, Direction: {status['dir']}")
```



## Future Enhancements



### Potential Improvements

1. **Non-Blocking Servo Control**
  - Implement gradual servo movement
  - Remove `delay()` calls from command handlers
  - Allow status updates during servo transitions
2. **Enhanced TOF Features**
  - Add VL53L1X support for longer range
  - Implement multi-zone detection
  - Add automatic obstacle avoidance
3. **Diagnostic Modes**
  - Add self-test command
  - Implement motor current sensing
  - Add I2C bus health monitoring
4. **Configuration Protocol**
  - Runtime adjustment of timing constants
  - Servo calibration commands
  - TOF threshold configuration



## Appendix



### Full Pin Summary


| Function       | GPIO | Type          | Description               |
| -------------- | ---- | ------------- | ------------------------- |
| I2C SDA        | 21   | Bidirectional | I2C data line             |
| I2C SCL        | 22   | Output        | I2C clock line            |
| TOF_UP_XSHUT   | 25   | Output        | Upper TOF sensor shutdown |
| TOF_DOWN_XSHUT | 26   | Output        | Lower TOF sensor shutdown |
| MOTOR_A_EN     | 32   | PWM Output    | Motor A speed control     |
| MOTOR_A_IN1    | 33   | Output        | Motor A direction         |
| MOTOR_A_IN2    | 27   | Output        | Motor A direction         |
| MOTOR_B_EN     | 14   | PWM Output    | Motor B speed control     |
| MOTOR_B_IN1    | 12   | Output        | Motor B direction         |
| MOTOR_B_IN2    | 13   | Output        | Motor B direction         |
| LED            | 2    | Output        | Status indicator          |




### Document Revision History


| Version | Date       | Author | Changes               |
| ------- | ---------- | ------ | --------------------- |
| 1.0     | 2026-09-07 | System | Initial documentation |


---

**End of Technical Documentation**