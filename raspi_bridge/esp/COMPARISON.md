# Comparison: Arduino Bridge vs ESP32 Bridge

## Summary
The ESP32 version maintains **identical logic and behavior** to the Arduino version, with changes only in hardware-specific implementations for pins, PWM, and motor control libraries.

---

## 1. Hardware Pin Definitions

### Arduino (`arduino_bridge.ino`)
```cpp
// TOF sensors
const int TOF_UP_XSHUT = 10;      // Arduino pin 10 (A1)
const int TOF_DOWN_XSHUT = 11;    // Arduino pin 11 (A2)

// Motors (L298NX2 library pins)
const uint8_t DRV_EN_A = 6;
const uint8_t DRV_IN1_A = 13;
const uint8_t DRV_IN2_A = 12;
const uint8_t DRV_EN_B = 5;
const uint8_t DRV_IN1_B = 8;
const uint8_t DRV_IN2_B = 7;

// LED
const int ledPin = 2;
digitalWrite(LED_BUILTIN, ...);   // Also controls built-in LED
```

### ESP32 (`esp_bridge.ino`)
```cpp
// TOF sensors
#define TOF_UP_XSHUT 25           // ESP32 GPIO 25
#define TOF_DOWN_XSHUT 26         // ESP32 GPIO 26

// Motors (direct GPIO control)
#define MOTOR_A_EN 32
#define MOTOR_A_IN1 33
#define MOTOR_A_IN2 27
#define MOTOR_B_EN 14
#define MOTOR_B_IN1 12
#define MOTOR_B_IN2 13

// LED
#define LED_PIN 2                 // ESP32 onboard LED only
```

**Differences:**
- ESP32 uses `#define` instead of `const int` for pin definitions
- Different GPIO numbers for all pins
- ESP32 doesn't use `LED_BUILTIN` (only one LED)
- ESP32 explicitly defines I2C pins: `I2C_SDA 21`, `I2C_SCL 22`

---

## 2. Include Headers

### Arduino
```cpp
#include <Adafruit_PWMServoDriver.h>
#include <L298N.h>              // Motor library
#include <Servo.h>              // Unused but included
#include <L298NX2.h>            // Motor library
#include <Wire.h>
#include <VL53L0X.h>
```

### ESP32
```cpp
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L0X.h>
// No motor libraries - uses direct PWM control
```

**Differences:**
- ESP32 removed `L298N.h`, `L298NX2.h`, and `Servo.h`
- ESP32 implements motor control directly using ESP32 LEDC PWM

---

## 3. Motor Control Implementation

### Arduino (Library-based)
```cpp
// Uses L298NX2 library object
L298NX2 motor_drive(DRV_EN_A, DRV_IN1_A, DRV_IN2_A, 
                    DRV_EN_B, DRV_IN1_B, DRV_IN2_B);

// Motor operations
motor_drive.stop();
motor_drive.setSpeed(speed);
motor_drive.run(curDirection);

// Reading state
unsigned int speed = motor_drive.getSpeedA();
L298N::Direction d = motor_drive.getDirectionA();
```

### ESP32 (Direct PWM Control)
```cpp
// PWM configuration
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1

// State tracking variables
int current_speed = 0;
enum Direction { STOP, FORWARD, BACKWARD };
Direction current_direction = STOP;

// Setup function
void setupMotorPWM() {
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_A_EN, PWM_CHANNEL_A);
  ledcAttachPin(MOTOR_B_EN, PWM_CHANNEL_B);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
}

// Motor operations
void stopMotors() {
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  current_speed = 0;
  current_direction = STOP;
}

void setMotors(int speed, Direction dir) {
  current_speed = speed;
  current_direction = dir;
  
  // Set direction pins
  if (dir == FORWARD) {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
  } else {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
  }
  
  // Set PWM speed
  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, speed);
}

// Reading state (direct variable access)
// Uses: current_speed and current_direction
```

**Differences:**
- Arduino uses library abstraction; ESP32 uses direct hardware control
- ESP32 manually tracks motor state in variables
- ESP32 has dedicated `setupMotorPWM()` function
- ESP32 uses `ledcWrite()` for PWM instead of library methods
- ESP32 defines custom `enum Direction` instead of using `L298N::Direction`

---

## 4. I2C Initialization

### Arduino
```cpp
void setup() {
  // ...
  Wire.begin();  // Uses default Arduino I2C pins
  // ...
}
```

### ESP32
```cpp
void setup() {
  // ...
  Wire.begin(I2C_SDA, I2C_SCL);  // Explicitly sets SDA=21, SCL=22
  // ...
}
```

**Differences:**
- ESP32 must explicitly specify I2C pins
- Arduino uses implicit default pins

---

## 5. Direction Handling in setDriveMotors()

### Arduino
```cpp
void setDriveMotors(int speed, String direction) {
  if (speed == 0) {
    motor_drive.stop();
    return;
  }

  L298N::Direction curDirection;
  if (direction == "FORWARD") {
    curDirection = L298N::FORWARD;
  }
  else if (direction == "BACKWARD") {
    curDirection = L298N::BACKWARD;
  }
  else {
    Serial.println(F("ERR:invalid direction in setDriveMotors"));
    motor_drive.stop();
    return;
  }

  motor_drive.setSpeed(speed);
  motor_drive.run(curDirection);
  digitalWrite(ledPin, HIGH);
}
```

### ESP32
```cpp
void setDriveMotors(int speed, String direction) {
  if (speed == 0) {
    stopMotors();
    return;
  }

  Direction dir;
  if (direction == "FORWARD") {
    dir = FORWARD;
  } else if (direction == "BACKWARD") {
    dir = BACKWARD;
  } else {
    Serial.println("ERR:invalid direction in setDriveMotors");
    stopMotors();
    return;
  }

  setMotors(speed, dir);
  digitalWrite(LED_PIN, HIGH);
}
```

**Differences:**
- Arduino uses `L298N::Direction`, ESP32 uses custom `Direction` enum
- Arduino calls library methods, ESP32 calls custom functions
- ESP32 uses `stopMotors()` instead of `motor_drive.stop()`
- ESP32 uses `setMotors()` instead of `setSpeed()` + `run()`
- Arduino uses `F()` macro for string constants (PROGMEM), ESP32 doesn't need it

---

## 6. Status JSON Generation

### Arduino
```cpp
void emitStatusLine() {
  uint16_t safe_tof_up, safe_tof_down;
  noInterrupts();
  safe_tof_up = tof_up_mm;
  safe_tof_down = tof_down_mm;
  interrupts();
  
  unsigned int speed = motor_drive.getSpeedA();
  L298N::Direction d = motor_drive.getDirectionA();
  const char* dir_s = (d == L298N::FORWARD) ? "FORWARD" :
                      (d == L298N::BACKWARD) ? "BACKWARD" : "STOP";
  
  Serial.print(F("STS:{..."));  // Uses F() macro
  // ...
}
```

### ESP32
```cpp
void emitStatusLine() {
  uint16_t safe_tof_up = tof_up_mm;
  uint16_t safe_tof_down = tof_down_mm;
  
  const char* dir_s = (current_direction == FORWARD) ? "FORWARD" :
                      (current_direction == BACKWARD) ? "BACKWARD" : "STOP";

  Serial.print("STS:{...");  // No F() macro needed
  Serial.print(current_speed);
  // ...
}
```

**Differences:**
- Arduino uses `noInterrupts()`/`interrupts()` for atomic reads (8-bit AVR CPU)
- ESP32 doesn't need interrupt protection (32-bit CPU with atomic operations)
- Arduino reads from library with `getSpeedA()` and `getDirectionA()`
- ESP32 reads from state variables `current_speed` and `current_direction`
- Arduino uses `F()` macro to save RAM; ESP32 has plenty of RAM so doesn't need it

---

## 7. LED Control

### Arduino
```cpp
digitalWrite(ledPin, ...);
digitalWrite(LED_BUILTIN, ...);  // Controls two LEDs
```

### ESP32
```cpp
digitalWrite(LED_PIN, ...);  // Controls one LED only
```

**Differences:**
- Arduino controls two separate LEDs
- ESP32 only controls one onboard LED

---

## 8. Setup Sequence

### Arduino
```cpp
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Direct pin setup
  pinMode(DRV_EN_A, OUTPUT);
  pinMode(DRV_IN1_A, OUTPUT);
  // ... (individual pin setups)

  motor_drive.stop();

  Wire.begin();
  servo_hub.begin();
  // ...
}
```

### ESP32
```cpp
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Setup motor PWM (all in one function)
  setupMotorPWM();
  stopMotors();

  // Initialize I2C with explicit pins
  Wire.begin(I2C_SDA, I2C_SCL);
  
  servo_hub.begin();
  // ...
}
```

**Differences:**
- ESP32 uses `setupMotorPWM()` helper function instead of individual pin setups
- ESP32 explicitly specifies I2C pins in `Wire.begin()`
- Arduino initializes L298NX2 object implicitly; ESP32 configures PWM channels

---

## 9. String Literals

### Arduino
```cpp
Serial.println(F("INIT: tof up="));  // Uses F() macro extensively
Serial.print(F("STS:{"));
```

### ESP32
```cpp
Serial.println("INIT: tof up=");  // No F() macro
Serial.print("STS:{");
```

**Differences:**
- Arduino uses `F()` macro to store strings in flash (PROGMEM) to save 2KB RAM
- ESP32 has 520KB RAM so doesn't need this optimization

---

## 10. Complete Feature Parity

### ✅ Identical Features (Logic Preserved)
- Serial protocol: same CSV format, same JSON output
- TOF sensor management: same init, retry, watchdog logic
- Safety latch: same behavior on WARN: and estop
- Servo sequences: same blocking delays, same bar positions
- Status telemetry: same 200ms period, same JSON structure
- Command parsing: identical validation and execution
- Calibration: same two-point linear calibration constants
- All attachment commands: up_attach, down_attach, both_attach, etc.

---

## Summary Table

| Feature | Arduino | ESP32 | Compatible? |
|---------|---------|-------|-------------|
| **Serial Protocol** | 115200 baud, CSV commands | Same | ✅ Yes |
| **Motor Control** | L298NX2 library | Direct LEDC PWM | ✅ Same behavior |
| **TOF Sensors** | VL53L0X on pins 10,11 | VL53L0X on GPIO 25,26 | ✅ Same logic |
| **Servo Hub** | PCA9685 on I2C | Same | ✅ Yes |
| **I2C** | Default pins | Explicit GPIO 21,22 | ✅ Compatible |
| **Status JSON** | 200ms updates | Same | ✅ Yes |
| **Safety Latch** | WARN/estop support | Same | ✅ Yes |
| **LED** | 2 LEDs | 1 LED | ⚠️ Visual only |
| **Memory** | F() macros needed | No F() needed | ✅ Compatible |
| **Atomic Ops** | noInterrupts() | Not needed | ✅ Compatible |

---

## Conclusion

The ESP32 version is a **faithful hardware port** with:
- **100% protocol compatibility** with Raspberry Pi scripts
- **Identical motor control behavior** (just different implementation)
- **Same sensor handling, safety features, and command logic**
- **Only hardware-specific differences** (pins, PWM method, library choice)

No changes needed to Python scripts on the Raspberry Pi side!
