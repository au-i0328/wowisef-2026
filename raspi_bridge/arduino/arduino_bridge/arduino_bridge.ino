/**
 * Arduino Bridge — extends sketch_jul30/sketch_jul30.ino
 * --------------------------------------------------
 *  - Motor control, servos, bar-pose logic: unchanged from sketch_jul30
 *  - Adds: 2x VL53L0X TOF read (up + down) via XSHUT re-addressing
 *  - Adds: 200 ms periodic JSON status exports with prefix "STS:"
 *  - Adds: safety latch on `WARN:<sensor>` lines from the Pi
 *  - Keeps: existing `ACK:<cmd>` lines emitted only on command events
 *
 * Serial protocol on Arduino -> Pi (line oriented, ASCII):
 *   STS:{...json...}      - one per 200 ms (telemetry, with sensor data)
 *   ACK:<cmd>             - one per command event (e.g. ACK:up_attach)
 *   WARNED:<sensor>       - one per WARN: line received (debug-only echo)
 *
 * Serial protocol on Pi -> Arduino (CSV line, newline-terminated):
 *   <speed>,<dir>,<cmd>   - e.g. "150,FORWARD,NONE\n"
 *   <dir> in {FORWARD, BACKWARD}
 *   <cmd> in {NONE, up_attach, up_detach, down_attach, down_detach,
 *             both_attach, both_detach, estop}
 *
 *   WARN:<sensor>         - e.g. "WARN:up\n"; immediately stops the
 *                           drive motors and latches. While latched, all
 *                           CSV lines are dropped until the next inbound
 *                           non-NONE command, which clears the latch and
 *                           runs normally.
 *
 *   estop                 - sent as "0,FORWARD,estop". Immediately
 *                           stops the drive motors and latches with
 *                           latch_reason="estop". Recovery is the same
 *                           as WARN: click "Run both_attach" in the
 *                           dashboard; the first inbound non-NONE
 *                           command clears the latch and runs.
 *
 * Wire the sensors on the Arduino's I2C bus (the Wire already started in
 * sketch_jul30 setup()):
 *   - VL53L0X up       XSHUT on A1, INT -> NC
 *   - VL53L0X down     XSHUT on A2, INT -> NC
 */

#include <Adafruit_PWMServoDriver.h>
#include <L298N.h>
#include <Servo.h>
#include <L298NX2.h>
#include <Wire.h>
#include <VL53L0X.h>

const int TOF_UP_XSHUT = 10;
const int TOF_DOWN_XSHUT  = 11;
const int TOF_MEASUREMENT_TIME = 10000;

// Set this to false to disable both VL53L0X sensors completely.
// When disabled, the sketch keeps both XSHUT pins low and skips all TOF
// initialization, reads, and watchdog re-initialisation calls.
const bool TOF_ENABLED = false;

VL53L0X          tof_up;
VL53L0X          tof_down;

// ===================== Sensors state =====================
// tof_*_mm == 0xFFFF means "sensor down / no fresh sample" and is the
// distinct sentinel that propagates into the JSON status. Raw readings
// are calibrated only when they are not this sentinel.
const uint16_t TOF_SENSOR_DOWN = 0xFFFF;

volatile uint16_t tof_up_mm    = TOF_SENSOR_DOWN;
volatile uint16_t tof_down_mm  = TOF_SENSOR_DOWN;
volatile bool     sensors_ok   = false;

// Per-sensor health for fail-safe / retry logic. Initialised in setup()
// once the first init attempt has had a chance to succeed or fail.
bool     tof_up_ready         = false;
bool     tof_down_ready       = false;

unsigned long tof_up_next_retry_ms   = 0;  // earliest time we'll re-init
unsigned long tof_down_next_retry_ms = 0;
const     long TOF_RETRY_AFTER_MS    = 5000;  // milliseconds between retry attempts

// I2C read timeout budget: if sensor reads take longer than this, return stale values
unsigned long last_sensor_read_ms = 0;
const unsigned long SENSOR_READ_TIMEOUT_MS = 50;  // allow 50ms for both sensors

// Init phase failure bitmask for the boot banner.
const uint8_t TOF_INIT_UP_OK   = 0x01;
const uint8_t TOF_INIT_DOWN_OK = 0x02;

// ===================== TOF calibration =====================
// Manual two-point linear calibration. Each VL53L0X is biased differently,
// so the raw millimetre reading is corrected via a per-sensor line fit
// through two known-distance reference points:
//
//   up   sensor: real 70,100 -> measured 87,118  => slope=30/31,  offset=-14.194
//   down sensor: real 70,100 -> measured 135,170 => slope=30/35,  offset=-45.714
//
// Corrected = slope * raw + offset, clamped to [0, 65535] mm.
//
// To re-derive from new measurements (raw_a, real_a), (raw_b, real_b):
//   slope  = (real_b - real_a) / (raw_b - raw_a)
//   offset = real_a - slope * raw_a
//
// Linear extrapolation outside [raw_a, raw_b] is naive by design; tighten
// the reference points if the corners matter.
struct TofCalib { float slope; float offset; };
const TofCalib TOF_UP_CALIB   = { 30.0f / 31.0f, 70.0f - (30.0f / 31.0f)  *  87.0f };
const TofCalib TOF_DOWN_CALIB = { 30.0f / 35.0f, 70.0f - (30.0f / 35.0f) * 135.0f };

static uint16_t applyTofCalib(uint16_t raw, const TofCalib& c) {
  float v = c.slope * (float)raw + c.offset;
  // Guard against NaN/Inf from corrupted sensor readings
  if (isnan(v) || isinf(v)) v = 0.0f;
  if (v < 0.0f)     v = 0.0f;
  if (v > 65535.0f) v = 65535.0f;
  return (uint16_t)(v + 0.5f);  // round-to-nearest
}

// ===================== Safety latch =====================
// When the Pi / dashboard raises a TOF warning, it sends a `WARN:up` or
// `WARN:down` line. The Arduino stops the drive motors and latches: every
// subsequent inbound CSV line is ignored (the dashboard stops motors, the
// gamepad is still streaming at 30 Hz but nothing moves) until the next
// inbound command (anything other than `NONE`) arrives, at which point the
// latch clears and the command executes normally.
bool     latched = false;
char     latch_reason[16] = "";

// ===================== Status export =====================
unsigned long next_status_ms = 0;
const     long STATUS_PERIOD_MS = 200;
unsigned long next_watchdog_ms = 0;  // Separate timer for sensor watchdog
char     last_cmd[24]   = "NONE";
String   current_bar_pose = "parallel";
char     current_dir[10] = "FORWARD";

// ===================== Servo timing configuration =====================
const unsigned long delay_to_pose = 1500;  // milliseconds to wait between servo steps

// ===================== LEDs/serial =====================
const int ledPin = 2;
unsigned long led_flash_timer = 0;
bool led_state = true;

// --------- Motors ----------
//unsigned int run_time_to_pose = 1500;

// L298NX2 pin map (must match the constructor above):
// EN_A=6, IN1_A=13, IN2_A=12, EN_B=5, IN1_B=8, IN2_B=7.
const uint8_t DRV_EN_A  = 6;
const uint8_t DRV_IN1_A = 13;
const uint8_t DRV_IN2_A = 12;
const uint8_t DRV_EN_B  = 5;
const uint8_t DRV_IN1_B = 8;
const uint8_t DRV_IN2_B = 7;

L298NX2 motor_drive(DRV_EN_A, DRV_IN1_A, DRV_IN2_A, DRV_EN_B, DRV_IN1_B, DRV_IN2_B);

Adafruit_PWMServoDriver servo_hub = Adafruit_PWMServoDriver(0x40);

// PCA9685 servo hub channels (0-15) 
// DO NOT EDIT
const int chServoUpL = 13;
const int chServoUpR = 12;
const int chServoDownL = 9;
const int chServoDownR = 8;
const int chServoBarUpL = 15;
const int chServoBarUpR = 14;
const int chServoBarDownL = 11;
const int chServoBarDownR = 10;

const float open_position = 180; //degrees
const float close_position = 27; //degrees
const float angle_change = 35; //degrees

static uint16_t angleToPulse(float angle, float minAngle=0, float maxAngle=180, //for MG90S
                             uint16_t minPulse=1000, uint16_t maxPulse=2000) {
  if (angle < minAngle) angle = minAngle;
  if (angle > maxAngle) angle = maxAngle;
  float t = (angle - minAngle) / (maxAngle - minAngle);
  return (uint16_t)(minPulse + t * (maxPulse - minPulse));
}

static uint16_t angleToPulse2(float angle, float minAngle=0, float maxAngle=300, //for goBilda
                             uint16_t minPulse=500, uint16_t maxPulse=2500) {
  if (angle < minAngle) angle = minAngle;
  if (angle > maxAngle) angle = maxAngle;
  float t = (angle - minAngle) / (maxAngle - minAngle);
  return (uint16_t)(minPulse + t * (maxPulse - minPulse));
}

// ===================== Sensor init =====================
// boot both sensors in a fresh power-up sequence so each gets a unique
// I2C address. Returns a bitmask of TOF_INIT_*_OK; 0 means total failure,
// in which case setup() continues without blocking (fail-safe).
uint8_t initTOF() {
  if (!TOF_ENABLED) {
    // Hold both sensor shutdown pins low so the devices stay in hardware
    // reset and cannot participate in I2C traffic.
    pinMode(TOF_UP_XSHUT, OUTPUT);
    pinMode(TOF_DOWN_XSHUT, OUTPUT);
    digitalWrite(TOF_UP_XSHUT, LOW);
    digitalWrite(TOF_DOWN_XSHUT, LOW);
    return 0;
  }

  pinMode(TOF_UP_XSHUT, OUTPUT);
  pinMode(TOF_DOWN_XSHUT, OUTPUT);
  // Power-cycle everything: both XSHUT low -> both chips reset -> up comes
  // up first so we can re-address it before down wakes.
  digitalWrite(TOF_DOWN_XSHUT, LOW);
  digitalWrite(TOF_UP_XSHUT,   LOW);
  delay(50);
  digitalWrite(TOF_UP_XSHUT, HIGH);
  delay(50);
  uint8_t r = 0;
  if (tof_up.init()) {
    tof_up.setAddress(0x30);
    tof_up.startContinuous();
    r |= TOF_INIT_UP_OK;
  }
  digitalWrite(TOF_DOWN_XSHUT, HIGH);
  delay(50);
  if (tof_down.init()) {
    tof_down.setAddress(0x31);
    tof_down.startContinuous();
    r |= TOF_INIT_DOWN_OK;
  }
  return r;
}

// Re-initialise a single sensor (power-cycle XSHUT, re-init, re-address,
// restart continuous). Cheap enough to call from the loop on a watchdog
// schedule. Returns true on success. The other sensor's bus state is not
// touched (XSHUT stays HIGH, I2C address unchanged).
bool reinitOneSensor(bool which_up) {
  if (!TOF_ENABLED) {
    return false;
  }
  if (which_up) {
    digitalWrite(TOF_UP_XSHUT, LOW);
    delay(20);
    digitalWrite(TOF_UP_XSHUT, HIGH);
    delay(50);
    if (!tof_up.init()) return false;
    tof_up.setAddress(0x30);
    tof_up.startContinuous();
    return true;
  } else {
    digitalWrite(TOF_DOWN_XSHUT, LOW);
    delay(20);
    digitalWrite(TOF_DOWN_XSHUT, HIGH);
    delay(50);
    if (!tof_down.init()) return false;
    tof_down.setAddress(0x31);
    tof_down.startContinuous();
    return true;
  }
}

void readSensors() {
  if (!TOF_ENABLED) {
    tof_up_mm = TOF_SENSOR_DOWN;
    tof_down_mm = TOF_SENSOR_DOWN;
    return;
  }

  unsigned long now = millis();
  
  // Timeout budget: only attempt read if sufficient time has passed
  // If sensors are slow/hung, return stale values rather than blocking
  if ((long)(now - last_sensor_read_ms) < SENSOR_READ_TIMEOUT_MS) {
    return;  // Return stale values, don't block
  }
  last_sensor_read_ms = now;

  // If a sensor failed to init at boot or its retry hasn't succeeded yet,
  // surface TOF_SENSOR_DOWN immediately rather than calling into a
  // half-initialised VL53L0X object (which can stall the I2C bus and
  // wedge the entire status pipeline).
  uint16_t raw_up   = tof_up_ready   ? tof_up.readRangeContinuousMillimeters()
                                     : TOF_SENSOR_DOWN;
  uint16_t raw_down = tof_down_ready ? tof_down.readRangeContinuousMillimeters()
                                     : TOF_SENSOR_DOWN;

  // Validate raw reading before calibration to prevent NaN propagation
  if (raw_up == TOF_SENSOR_DOWN || raw_up == 0 || tof_up.timeoutOccurred()) {
    tof_up_mm = TOF_SENSOR_DOWN;
  } else {
    tof_up_mm = applyTofCalib(raw_up, TOF_UP_CALIB);
  }

  if (raw_down == TOF_SENSOR_DOWN || raw_down == 0 || tof_down.timeoutOccurred()) {
    tof_down_mm = TOF_SENSOR_DOWN;
  } else {
    tof_down_mm = applyTofCalib(raw_down, TOF_DOWN_CALIB);
  }
}

// Watchdog: if a sensor has been returning 0xFFFF for at least
// TOF_RETRY_AFTER_MS, attempt a power-cycle re-init. Uses signed-long
// subtraction to handle millis() rollover at ~49 days. On success the
// sensor immediately goes "up"; on failure the cached value stays
// 0xFFFF and we wait another retry window before trying again.
void ensureSensorsAlive() {
  if (!TOF_ENABLED) {
    return;
  }

  unsigned long now = millis();

  // Retry up sensor if it's down and retry window has elapsed
  // (removed tof_up_ready check so boot-failed sensors can recover)
  if (tof_up_mm == TOF_SENSOR_DOWN
      && (long)(now - tof_up_next_retry_ms) >= 0) {
    if (reinitOneSensor(true)) {
      tof_up_ready = true;
      tof_up_mm = TOF_SENSOR_DOWN;  // will be filled on next readSensors()
      tof_up_next_retry_ms = now + TOF_RETRY_AFTER_MS;
      Serial.println(F("TOF: up re-init OK"));
    } else {
      tof_up_next_retry_ms = now + TOF_RETRY_AFTER_MS;
      Serial.println(F("TOF: up re-init FAIL"));
    }
  } else if (tof_up_mm != TOF_SENSOR_DOWN) {
    // Healthy -- schedule the next retry window relative to now so a
    // future timeout is debounced.
    tof_up_next_retry_ms = now + TOF_RETRY_AFTER_MS;
  }

  // Retry down sensor if it's down and retry window has elapsed
  if (tof_down_mm == TOF_SENSOR_DOWN
      && (long)(now - tof_down_next_retry_ms) >= 0) {
    if (reinitOneSensor(false)) {
      tof_down_ready = true;
      tof_down_mm = TOF_SENSOR_DOWN;
      tof_down_next_retry_ms = now + TOF_RETRY_AFTER_MS;
      Serial.println(F("TOF: down re-init OK"));
    } else {
      tof_down_next_retry_ms = now + TOF_RETRY_AFTER_MS;
      Serial.println(F("TOF: down re-init FAIL"));
    }
  } else if (tof_down_mm != TOF_SENSOR_DOWN) {
    tof_down_next_retry_ms = now + TOF_RETRY_AFTER_MS;
  }
}

// ===================== Hand-rolled JSON (no ArduinoJson) =====================
void emitStatusLine() {
  // Atomic read of volatile 16-bit variables (AVR is 8-bit CPU)
  uint16_t safe_tof_up, safe_tof_down;
  noInterrupts();
  safe_tof_up = tof_up_mm;
  safe_tof_down = tof_down_mm;
  interrupts();
  
  // L298NX2 has no aggregate getter; both motors are commanded to the
  // same value in setDriveMotors(), so reading motor A is the source
  // of truth for the status line.
  unsigned int speed = motor_drive.getSpeedA();
  L298N::Direction d = motor_drive.getDirectionA();
  const char* dir_s = (d == L298N::FORWARD) ? "FORWARD" :
                      (d == L298N::BACKWARD) ? "BACKWARD" : "STOP";

  Serial.print(F("STS:{"
                 "\"speed\":"));
  Serial.print(speed);
  Serial.print(F(",\"dir\":\""));
  Serial.print(dir_s);
  Serial.print(F("\",\"pose\":\""));
  Serial.print(current_bar_pose);
  Serial.print(F("\",\"ack\":\""));
  Serial.print(last_cmd);
  Serial.print(F("\",\"latched\":"));
  Serial.print(latched ? "true" : "false");
  if (latched) {
    Serial.print(F(",\"latch_reason\":\""));
    Serial.print(latch_reason);
    Serial.print(F("\""));
  }
  Serial.print(F(",\"tof\":{"
                 "\"enabled\":"));
  Serial.print(TOF_ENABLED ? "true" : "false");
  Serial.print(F(",\"up\":"));
  Serial.print(safe_tof_up);
  Serial.print(F(",\"down\":"));
  Serial.print(safe_tof_down);
  Serial.print(F("}}"));
  Serial.println();
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(DRV_EN_A, OUTPUT);
  pinMode(DRV_IN1_A, OUTPUT);
  pinMode(DRV_IN2_A, OUTPUT);
  pinMode(DRV_EN_B, OUTPUT);
  pinMode(DRV_IN1_B, OUTPUT);
  pinMode(DRV_IN2_B, OUTPUT);
  digitalWrite(DRV_EN_A, LOW);
  digitalWrite(DRV_IN1_A, LOW);
  digitalWrite(DRV_IN2_A, LOW);
  digitalWrite(DRV_EN_B, LOW);
  digitalWrite(DRV_IN1_B, LOW);
  digitalWrite(DRV_IN2_B, LOW);

  motor_drive.stop();

  Wire.begin();
  servo_hub.begin();
  servo_hub.reset();
  servo_hub.setPWMFreq(50);

  // TOF sensors: set both XSHUT pins low before the optional hardware test.
  // If disabled, the motors, servos, serial protocol, and telemetry remain
  // fully operational; status reports enabled:false with 0xFFFF placeholders.
  pinMode(TOF_UP_XSHUT, OUTPUT);
  pinMode(TOF_DOWN_XSHUT, OUTPUT);
  digitalWrite(TOF_UP_XSHUT, LOW);
  digitalWrite(TOF_DOWN_XSHUT, LOW);

  // TOF sensors: boot them in a fail-safe way. If one or both fail to
  // come up we keep going -- the dashboard will show "sensor down" via
  // the 0xFFFF sentinel and ensureSensorsAlive() will retry every
  // TOF_RETRY_AFTER_MS. This is what the user asked for: never block
  // setup() on a missing sensor.
  uint8_t tof_r = initTOF();
  tof_up_ready   = (tof_r & TOF_INIT_UP_OK)   != 0;
  tof_down_ready = (tof_r & TOF_INIT_DOWN_OK) != 0;
  sensors_ok     = (tof_r != 0);
  unsigned long _t0 = millis();
  tof_up_next_retry_ms   = _t0 + TOF_RETRY_AFTER_MS;
  tof_down_next_retry_ms = _t0 + TOF_RETRY_AFTER_MS;

  Serial.print(F("INIT: tof up="));
  Serial.print(tof_up_ready   ? "OK" : "FAIL");
  Serial.print(F(" down="));
  Serial.println(tof_down_ready ? "OK" : "FAIL");

  if (tof_up_ready) {
    tof_up.setMeasurementTimingBudget(TOF_MEASUREMENT_TIME);
  }
  if (tof_down_ready) {
    tof_down.setMeasurementTimingBudget(TOF_MEASUREMENT_TIME);
  }

  // Initialize servos to default positions (preserved from sketch_jul30)
  setBarPosition("parallel");
  
  // Initialize all gripper servos to closed position at startup
  for (uint8_t ch = chServoUpL; ch <= chServoDownR; ch++) {
    servo_hub.writeMicroseconds(ch, angleToPulse(close_position));
  }
}

// ===================== Main loop =====================
void loop() {
  unsigned long now = millis();
  
  // 1. Inbound commands / warnings
  if (Serial.available() > 0) {
    String payload = Serial.readStringUntil('\n');
    payload.trim();
    
    // Validate length to prevent heap exhaustion
    if (payload.length() > 128) {
      Serial.println(F("ERR:line too long"));
      return;
    }
    
    if (payload.length() > 0) {
      if (payload.startsWith("WARN:")) {
        parseAndExecuteWarning(payload);
      } else {
        parseAndExecutePayload(payload);
      }
    }
  }

  // 2. Sensor watchdog: if any sensor is down and the retry window has
  // elapsed, attempt to bring it back. Runs once per second (gated by timer).
  if ((long)(now - next_watchdog_ms) >= 0) {
    next_watchdog_ms = now + 1000;
    ensureSensorsAlive();
  }

  // 3. Periodic status export (200 ms)
  if ((long)(now - next_status_ms) >= 0) {
    next_status_ms = now + STATUS_PERIOD_MS;
    readSensors();
    emitStatusLine();
  }

  // 5. LED flash when drive is active (preserved)
  if (motor_drive.getSpeedA() > 0) {
    if (now - led_flash_timer >= 150) {
      led_state = !led_state;
      digitalWrite(ledPin, led_state ? HIGH : LOW);
      digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);
      led_flash_timer = now;
    }
  } else {
    digitalWrite(ledPin, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    led_state = true;
    led_flash_timer = now;
  }
}

// ===================== Command parsing (preserved + extended) =====================
void parseAndExecutePayload(String payload) {
  int firstComma = payload.indexOf(',');
  int secondComma = payload.indexOf(',', firstComma + 1);
  int thirdComma = payload.indexOf(',', secondComma + 1);

  // Support both 3-field (old) and 4-field (new with altitude) formats
  bool hasAltitudeField = (thirdComma != -1);

  // Validate CSV structure: at least 2 commas, non-empty fields
  if (firstComma == -1 || secondComma == -1 || firstComma == 0) {
    Serial.println(F("ERR:malformed CSV"));
    return;
  }

  String speedStr     = payload.substring(0, firstComma);
  String directionStr = payload.substring(firstComma + 1, secondComma);
  String commandStr;
  
  if (hasAltitudeField) {
    // New format: speed,direction,command,altitude_mode
    // Arduino doesn't support altitude control, so just extract command and ignore 4th field
    commandStr = payload.substring(secondComma + 1, thirdComma);
  } else {
    // Old format: speed,direction,command
    commandStr = payload.substring(secondComma + 1);
  }

  // Validate field lengths to prevent buffer overflow
  if (speedStr.length() == 0 || directionStr.length() > 9 || commandStr.length() > 20) {
    Serial.println(F("ERR:field length"));
    return;
  }

  // Safety latch: while latched, ignore drive-only updates (NONE) so the
  // robot stays stopped even while the gamepad keeps streaming. A real
  // command clears the latch and runs normally -- that's the "next manual
  // command" recovery path.
  if (latched) {
    if (commandStr == "NONE") {
      // Drop silently: motors stay at zero, no ACK.
      return;
    }
    // Clear the latch on the first inbound command and fall through.
    latched = false;
    latch_reason[0] = '\0';
  }

  int speedVal = speedStr.toInt();
  // Validate numeric conversion (toInt() returns 0 for non-numeric strings)
  if (speedVal == 0 && speedStr != "0") {
    Serial.println(F("ERR:non-numeric speed"));
    return;
  }

  // Validate direction before copying
  if (directionStr != "FORWARD" && directionStr != "BACKWARD") {
    Serial.println(F("ERR:invalid direction"));
    return;
  }

  // Mirror direction string for status output
  strncpy(current_dir, directionStr.c_str(), sizeof(current_dir) - 1);
  current_dir[sizeof(current_dir) - 1] = '\0';

  setDriveMotors(speedVal, directionStr);

  if (commandStr != "NONE") {
    executeCommand(commandStr);
  }
}

// ===================== Warning handler =====================
void parseAndExecuteWarning(String payload) {
  // payload looks like "WARN:up" or "WARN:down".
  String sensor = payload.substring(5);
  sensor.trim();
  if (sensor != "up" && sensor != "down") {
    return;
  }
  if (!TOF_ENABLED) {
    Serial.print(F("ERR:tof disabled:"));
    Serial.println(sensor);
    return;
  }
  // Stop the drive motors immediately.
  motor_drive.stop();
  digitalWrite(ledPin, LOW);
  latched = true;
  strncpy(latch_reason, sensor.c_str(), sizeof(latch_reason) - 1);
  latch_reason[sizeof(latch_reason) - 1] = '\0';
  Serial.print(F("WARNED:"));
  Serial.println(sensor);
}

void setDriveMotors(int speed, String direction){
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
    // Should never reach here due to validation in parseAndExecutePayload,
    // but fail safe to STOP if invalid direction somehow passes through
    Serial.println(F("ERR:invalid direction in setDriveMotors"));
    motor_drive.stop();
    return;
  }

  motor_drive.setSpeed(speed);
  motor_drive.run(curDirection);

  digitalWrite(ledPin, HIGH);
}

void executeCommand(String cmd) {
  if (cmd == "down_detach") {
    down_detach();
  }
  else if (cmd == "down_attach") {
    down_attach();
  }
  else if (cmd == "up_attach") {
    up_attach();
  }
  else if (cmd == "up_detach") {
    up_detach();
  }
  else if (cmd == "both_attach") {
    both_attach();
  }
  else if (cmd == "both_detach") {
    both_detach();
  }
  else if (cmd == "estop") {
    // Emergency stop: kill drive motors immediately and engage the
    // safety latch so the gamepad's 30 Hz drive stream can't re-arm
    // them. Servos are intentionally left in place -- the operator
    // clicks "Run both_attach" to recover, which clears the latch
    // and moves the gripper into the safe pose in one step.
    motor_drive.stop();
    digitalWrite(ledPin, LOW);
    // Only latch if not already in recovery mode (prevents deadlock)
    if (!latched) {
      latched = true;
      strncpy(latch_reason, "estop", sizeof(latch_reason) - 1);
      latch_reason[sizeof(latch_reason) - 1] = '\0';
    }
  }
  else {
    // Unknown command: ACK so the dashboard doesn't sit idle, but don't
    // pretend the Arduino executed it.
    Serial.print("ACK:");
    Serial.println(cmd);
    return;
  }
  // Record for status JSON
  strncpy(last_cmd, cmd.c_str(), sizeof(last_cmd) - 1);
  last_cmd[sizeof(last_cmd) - 1] = '\0';
  // Ack any recognized command (preserved from sketch_jul30.ino)
  Serial.print("ACK:");
  Serial.println(cmd);
  return;
}

void setBarPosition(String pose){
  if (pose == "parallel") {
    servo_hub.writeMicroseconds(chServoBarUpL, angleToPulse2(150));
    servo_hub.writeMicroseconds(chServoBarUpR, angleToPulse2(150));
    servo_hub.writeMicroseconds(chServoBarDownL, angleToPulse2(150));
    servo_hub.writeMicroseconds(chServoBarDownR, angleToPulse2(150));
  }
  else if (pose == "up_detach") {
    servo_hub.writeMicroseconds(chServoBarDownL, angleToPulse2(150 - angle_change));
    servo_hub.writeMicroseconds(chServoBarDownR, angleToPulse2(300 - (150 - angle_change)));
    servo_hub.writeMicroseconds(chServoBarUpL, angleToPulse2(150));
    servo_hub.writeMicroseconds(chServoBarUpR, angleToPulse2(150));
  }
  else if (pose == "down_detach") {
    servo_hub.writeMicroseconds(chServoBarUpL, angleToPulse2(150 + angle_change));
    servo_hub.writeMicroseconds(chServoBarUpR, angleToPulse2(300 - (150 + angle_change)));
    servo_hub.writeMicroseconds(chServoBarDownL, angleToPulse2(150));
    servo_hub.writeMicroseconds(chServoBarDownR, angleToPulse2(150));
  }
  current_bar_pose = pose;
}

// Helper function to eliminate servo command duplication
void setServoAngle(int channelL, int channelR, float angle) {
  servo_hub.writeMicroseconds(channelL, angleToPulse(angle));
  servo_hub.writeMicroseconds(channelR, angleToPulse(180 - angle));
}

void up_detach() {
  // Step 1: open UpLR servos
  setServoAngle(chServoUpL, chServoUpR, open_position);
  // Blocking delay
  delay(delay_to_pose);
  // Step 2: set bar pose to up_detach
  setBarPosition("up_detach");
}

void down_detach() {
  // Step 1: open DownLR servos
  setServoAngle(chServoDownL, chServoDownR, open_position);
  // Blocking delay
  delay(delay_to_pose);
  // Step 2: set bar pose to down_detach
  setBarPosition("down_detach");
}

void up_attach() {
  // Step 1: set bar pose to parallel
  setBarPosition("parallel");
  // Blocking delay
  delay(delay_to_pose);
  // Step 2: close UpLR and DownLR grippers
  setServoAngle(chServoUpL, chServoUpR, close_position);
  setServoAngle(chServoDownL, chServoDownR, close_position);
}

void down_attach() {
  // Step 1: set bar pose to parallel
  setBarPosition("parallel");
  // Blocking delay
  delay(delay_to_pose);
  // Step 2: close UpLR and DownLR grippers
  setServoAngle(chServoUpL, chServoUpR, close_position);
  setServoAngle(chServoDownL, chServoDownR, close_position);
}

void both_attach() {
  // Step 1: set bar pose to parallel
  setBarPosition("parallel");
  // Blocking delay
  delay(delay_to_pose);
  // Step 2: close UpLR and DownLR grippers
  setServoAngle(chServoUpL, chServoUpR, close_position);
  setServoAngle(chServoDownL, chServoDownR, close_position);
}

void both_detach() {
  // Step 1: set bar pose to parallel
  setBarPosition("parallel");
  // Blocking delay
  delay(delay_to_pose);
  // Step 2: open UpLR and DownLR grippers
  setServoAngle(chServoUpL, chServoUpR, open_position);
  setServoAngle(chServoDownL, chServoDownR, open_position);
}