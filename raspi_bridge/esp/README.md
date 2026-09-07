# ESP32 Bridge

ESP32 port of the Arduino bridge for the climbing robot. Maintains the same Raspberry Pi → Serial → ESP32 architecture with identical command protocol and motor control logic.

## Hardware Differences from Arduino

### Pin Mapping

**I2C (for PCA9685 servo hub and VL53L0X sensors):**
- SDA: GPIO 21 (ESP32 default)
- SCL: GPIO 22 (ESP32 default)

**VL53L0X Time-of-Flight Sensors:**
- Up sensor XSHUT: GPIO 25
- Down sensor XSHUT: GPIO 26

**L298N Motor Driver (Motor A - Left):**
- EN (Enable/Speed): GPIO 32
- IN1 (Direction): GPIO 33
- IN2 (Direction): GPIO 27

**L298N Motor Driver (Motor B - Right):**
- EN (Enable/Speed): GPIO 14
- IN1 (Direction): GPIO 12
- IN2 (Direction): GPIO 13

**LED:**
- GPIO 2 (onboard LED)

**PCA9685 Servo Hub Channels:** (same as Arduino)
- Gripper UpL: channel 13
- Gripper UpR: channel 12
- Gripper DownL: channel 9
- Gripper DownR: channel 8
- Bar UpL: channel 15
- Bar UpR: channel 14
- Bar DownL: channel 11
- Bar DownR: channel 10

## Key ESP32-Specific Changes

1. **PWM Motor Control**: Uses ESP32's LEDC (LED Controller) for PWM instead of `analogWrite()`:
   - 8-bit resolution (0-255)
   - 5000 Hz frequency
   - Two dedicated PWM channels (0 and 1) for motor speed control

2. **I2C Initialization**: Explicitly sets SDA/SCL pins with `Wire.begin(SDA, SCL)`

3. **Direct Motor Control**: Removed dependency on L298NX2 library, using direct GPIO and PWM control instead

4. **Serial Interface**: Same as Arduino - 115200 baud USB serial connection to Raspberry Pi

## Serial Protocol

**Pi → ESP32 (CSV format):**
```
<speed>,<direction>,<command>
```
- `speed`: 0-255 (PWM duty cycle)
- `direction`: FORWARD or BACKWARD
- `command`: NONE, up_attach, up_detach, down_attach, down_detach, both_attach, both_detach, estop

**ESP32 → Pi (line-oriented):**
```
STS:{...json...}     # Status telemetry every 200ms
ACK:<command>        # Command acknowledgment
WARNED:<sensor>      # Warning echo (debug)
```

**Warning Protocol:**
```
WARN:up    # Emergency stop due to upper TOF sensor
WARN:down  # Emergency stop due to lower TOF sensor
```

## Installation

### Required Libraries

Install these via Arduino IDE Library Manager or PlatformIO:

1. **Adafruit PWM Servo Driver Library** - for PCA9685 servo hub
2. **VL53L0X** (Pololu) - for time-of-flight distance sensors
3. **Wire** (built-in) - for I2C communication

### Board Setup

1. Install ESP32 board support in Arduino IDE:
   - File → Preferences → Additional Board Manager URLs
   - Add: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - Tools → Board → Boards Manager → search "esp32" → Install

2. Select your ESP32 board:
   - Tools → Board → ESP32 Arduino → ESP32 Dev Module (or your specific board)

3. Configure upload settings:
   - Upload Speed: 115200
   - Flash Frequency: 80MHz
   - Partition Scheme: Default 4MB with spiffs

### Upload

1. Connect ESP32 to computer via USB
2. Select correct COM port: Tools → Port
3. Click Upload

## Configuration

### Enable/Disable TOF Sensors

In `esp_bridge.ino`, line 53:
```cpp
const bool TOF_ENABLED = false;  // Set to true to enable TOF sensors
```

Set to `true` to enable time-of-flight distance sensors. When disabled:
- Both XSHUT pins stay LOW (sensors in hardware reset)
- Status reports `"tof":{"enabled":false,"up":65535,"down":65535}`
- No sensor watchdog retries

### Servo Timing

Line 79:
```cpp
const unsigned long delay_to_pose = 1500;  // milliseconds
```

Adjust this value to change the blocking delay between servo motion steps in attach/detach sequences.

### TOF Calibration

Lines 61-63 define per-sensor linear calibration:
```cpp
const TofCalib TOF_UP_CALIB = { 30.0f / 31.0f, 70.0f - (30.0f / 31.0f) * 87.0f };
const TofCalib TOF_DOWN_CALIB = { 30.0f / 35.0f, 70.0f - (30.0f / 35.0f) * 135.0f };
```

To recalibrate:
1. Measure at two known distances (e.g., 70mm and 100mm)
2. Record raw sensor readings
3. Calculate: `slope = (real_b - real_a) / (raw_b - raw_a)`
4. Calculate: `offset = real_a - slope * raw_a`

## Compatibility with Raspberry Pi Scripts

The ESP32 version is **fully compatible** with the existing Pi scripts:
- `raspi_bridge/pi/pi_serial_bridge.py`
- `raspi_bridge/laptop/gamepad_to_pi.py`
- `raspi_bridge/laptop/mac_dashboard.py`

No changes needed - just update the serial port to match your ESP32's device path (e.g., `/dev/ttyUSB0` or `/dev/ttyACM0`).

## Wiring ESP32 to Raspberry Pi

Connect ESP32 to Raspberry Pi via USB cable:
- **Power**: USB provides 5V power to ESP32
- **Data**: USB serial communication (TX/RX handled automatically)
- **Port**: Will appear as `/dev/ttyUSB0` or `/dev/ttyACM0` on the Pi

Check the port with: `ls -l /dev/ttyUSB* /dev/ttyACM*`

## Troubleshooting

### ESP32 won't upload
- Press and hold BOOT button during upload
- Check USB cable supports data (not charge-only)
- Verify correct COM port selected

### Motors don't move
- Check L298N 12V power supply connected
- Verify GPIO pin connections
- Check serial monitor for error messages

### TOF sensors show 65535
- Sensors disabled (set `TOF_ENABLED = true`)
- I2C wiring issue (check SDA/SCL connections)
- XSHUT pin connections incorrect
- Check Serial Monitor for "INIT: tof up=FAIL down=FAIL"

### Serial connection issues with Pi
- Verify baud rate: 115200
- Check USB cable connection
- Run `dmesg | grep tty` on Pi to see USB device detection
- Grant permissions: `sudo chmod 666 /dev/ttyUSB0`

## Performance Notes

- **CPU**: ESP32 dual-core 240MHz vs Arduino Uno single-core 16MHz
- **PWM**: 8-bit (0-255) vs Arduino's 8-bit analogWrite
- **Memory**: 520KB RAM vs Arduino Uno 2KB
- **Serial latency**: Similar to Arduino, ~30Hz command rate supported

The ESP32 has significantly more computational headroom, making it suitable for future expansion (WiFi telemetry, camera streaming, etc.) while maintaining backward compatibility with the current serial protocol.
