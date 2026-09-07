/**
 * ESP32-S3 Bridge with Altitude Control — extends esp_bridge.ino
 * ----------------------------------------------------------------
 *  - All original motor control, servos, bar-pose logic preserved
 *  - Adds: BNO055 IMU for pitch/roll/heading
 *  - Adds: VL53L1X ground distance sensor for altitude measurement
 *  - Adds: Autonomous altitude control modes (manual, hold, run-to)
 *  - Adds: Adaptive PIDG with gravity compensation based on DIRECTION + bar pose
 *  - Adds: Extended JSON status with altitude/IMU/gripper/active PIDG data
 *
 * New commands:
 *   set_alt    - switch to run-to-altitude mode
 *   hold_alt   - switch to hold-altitude mode
 *   zero_alt   - zero the altitude reference
 *   manual     - return to manual motor control
 *
 * New warnings:
 *   WARN:tilt  - stops motors on excessive tilt
 *
 * Gravity Compensation (6 profiles):
 *   Selected based on motor DIRECTION (up/down) + bar pose (parallel/angled):
 *   
 *   MOVING UP (FORWARD):
 *     - PIDG_UP_PARALLEL: Standard climb with parallel bars
 *     - PIDG_UP_ANGLED: Climb with up_detach pose (mechanical advantage)
 *   
 *   MOVING DOWN (BACKWARD):
 *     - PIDG_DOWN_PARALLEL: Standard descent with parallel bars
 *     - PIDG_DOWN_ANGLED: Descent with down_detach pose (better control)
 *   
 *   STOPPED or BOTH GRIPPERS:
 *     - PIDG_BOTH: Holding position or error state (no compensation)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L0X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <VL53L1X.h>

// ===================== ESP32-S3 Pin Configuration =====================
// I2C pins
#define I2C_SDA 21
#define I2C_SCL 22

// TOF sensor XSHUT pins (VL53L0X for proximity)
#define TOF_UP_XSHUT 25
#define TOF_DOWN_XSHUT 26

// L298N Motor Driver Pins
#define MOTOR_A_EN 32
#define MOTOR_A_IN1 33
#define MOTOR_A_IN2 27
#define MOTOR_B_EN 14
#define MOTOR_B_IN1 12
#define MOTOR_B_IN2 13

// LED
#define LED_PIN 2

// PWM Configuration for Motors
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1

// ===================== TOF Configuration (VL53L0X - original) =====================
const int TOF_MEASUREMENT_TIME = 10000;
const bool TOF_ENABLED = false;

VL53L0X tof_up;
VL53L0X tof_down;

const uint16_t TOF_SENSOR_DOWN = 0xFFFF;
volatile uint16_t tof_up_mm = TOF_SENSOR_DOWN;
volatile uint16_t tof_down_mm = TOF_SENSOR_DOWN;
volatile bool sensors_ok = false;

bool tof_up_ready = false;
bool tof_down_ready = false;

unsigned long tof_up_next_retry_ms = 0;
unsigned long tof_down_next_retry_ms = 0;
const long TOF_RETRY_AFTER_MS = 5000;

// M-6 FIX: Non-blocking sensor recovery state machine
enum SensorRecoveryState { RECOVERY_IDLE, RECOVERY_XSHUT_LOW, RECOVERY_XSHUT_HIGH, RECOVERY_INIT };
SensorRecoveryState recovery_up_state = RECOVERY_IDLE;
SensorRecoveryState recovery_down_state = RECOVERY_IDLE;
unsigned long recovery_up_state_start_ms = 0;
unsigned long recovery_down_state_start_ms = 0;

unsigned long last_sensor_read_ms = 0;
const unsigned long SENSOR_READ_TIMEOUT_MS = 50;

const uint8_t TOF_INIT_UP_OK = 0x01;
const uint8_t TOF_INIT_DOWN_OK = 0x02;

struct TofCalib { float slope; float offset; };
const TofCalib TOF_UP_CALIB = { 30.0f / 31.0f, 70.0f - (30.0f / 31.0f) * 87.0f };
const TofCalib TOF_DOWN_CALIB = { 30.0f / 35.0f, 70.0f - (30.0f / 35.0f) * 135.0f };

static uint16_t applyTofCalib(uint16_t raw, const TofCalib& c) {
  float v = c.slope * (float)raw + c.offset;
  if (isnan(v) || isinf(v)) v = 0.0f;
  if (v < 0.0f) v = 0.0f;
  if (v > 65535.0f) v = 65535.0f;
  return (uint16_t)(v + 0.5f);
}

// ===================== NEW: Altitude Control Sensors =====================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
VL53L1X groundSensor;

bool bno_ready = false;
bool ground_sensor_ready = false;

float pitchDeg = 0.0f;
float rollDeg = 0.0f;
float headingDeg = 0.0f;
float groundDistanceFiltered_mm = 0.0f;
float groundZeroOffset_mm = 0.0f;
float trueVerticalHeight_mm = 0.0f;

const float ALPHA_GROUND = 0.3f;
const int GROUND_SENSOR_TIMEOUT_MS = 100;

// ===================== NEW: Altitude Control Modes =====================
enum AltitudeMode {
  MODE_MANUAL = 0,
  MODE_HOLD_ALTITUDE = 1,
  MODE_RUN_TO_ALTITUDE = 2
};

AltitudeMode currentMode = MODE_MANUAL;
float targetAltitude_mm = 0.0f;

// PID state
float pidIntegral = 0.0f;
float pid_last_error = 0.0f;
unsigned long pid_last_update_ms = 0;

// ===================== PID + Gravity Compensation Configuration =====================
// Different coefficients based on gripper state AND bar pose
// Bar pose affects mechanical advantage and force distribution
struct PIDGConfig {
  float kp;
  float ki;
  float kd;
  float gravity_compensation;  // PWM offset to counteract weight
};

// ===== CLIMBING UP (upper detached, lower attached) =====

// UP + PARALLEL BARS: Balanced load, standard climbing
const PIDGConfig PIDG_UP_PARALLEL = {
  .kp = 0.8f,
  .ki = 0.05f,
  .kd = 0.2f,
  .gravity_compensation = 100.0f
};

// UP + UP_DETACH POSE: Bars angled to give mechanical advantage while climbing
// Lower bars push outward, reducing effective weight on motors
const PIDGConfig PIDG_UP_ANGLED = {
  .kp = 0.7f,
  .ki = 0.04f,
  .kd = 0.18f,
  .gravity_compensation = 80.0f  // Less compensation needed due to bar angle
};

// ===== HOLDING DOWN (lower detached, upper attached) =====

// DOWN + PARALLEL BARS: Balanced descent
const PIDGConfig PIDG_DOWN_PARALLEL = {
  .kp = 0.6f,
  .ki = 0.04f,
  .kd = 0.15f,
  .gravity_compensation = 40.0f
};

// DOWN + DOWN_DETACH POSE: Bars angled for controlled descent
// Upper bars provide leverage, easier to control descent
const PIDGConfig PIDG_DOWN_ANGLED = {
  .kp = 0.5f,
  .ki = 0.03f,
  .kd = 0.12f,
  .gravity_compensation = 30.0f  // Less force needed with bar assist
};

// ===== FALLBACK (both attached or both detached) =====

const PIDGConfig PIDG_BOTH = {
  .kp = 0.5f,
  .ki = 0.03f,
  .kd = 0.1f,
  .gravity_compensation = 0.0f  // No compensation when both grippers engaged
};

const float ALTITUDE_DEADBAND_MM = 5.0f;
const int BASE_CLIMB_SPEED = 150;

// Minimum motor speed threshold - motors won't move below this PWM value
const int MIN_HOLD_SPEED = 80;

// ===================== Safety latch =====================
bool latched = false;
char latch_reason[16] = "";

// ===================== Status export =====================
unsigned long next_status_ms = 0;
const long STATUS_PERIOD_MS = 200;
unsigned long next_watchdog_ms = 0;
char last_cmd[24] = "NONE";
String current_bar_pose = "parallel";
char current_dir[10] = "FORWARD";

// ===================== Servo timing configuration =====================
const unsigned long delay_to_pose = 750;

// ===================== LED =====================
unsigned long led_flash_timer = 0;
bool led_state = true;

// ===================== Motor state =====================
int current_speed = 0;
enum Direction { STOP, FORWARD, BACKWARD };
Direction current_direction = STOP;

// ===================== Gripper State Tracking =====================
enum GripperState { GRIPPER_OPEN, GRIPPER_CLOSED };
GripperState upperGripperState = GRIPPER_CLOSED;  // Start with both closed
GripperState lowerGripperState = GRIPPER_CLOSED;

// ===================== Servo Hub =====================
Adafruit_PWMServoDriver servo_hub = Adafruit_PWMServoDriver(0x40);

const int chServoUpL = 13;
const int chServoUpR = 12;
const int chServoDownL = 9;
const int chServoDownR = 8;
const int chServoBarUpL = 15;
const int chServoBarUpR = 14;
const int chServoBarDownL = 11;
const int chServoBarDownR = 10;

const float open_position = 180;
const float close_position = 27;
const float angle_change = 35;

// ===================== Helper Functions =====================
static uint16_t angleToPulse(float angle, float minAngle=0, float maxAngle=180,
                             uint16_t minPulse=1000, uint16_t maxPulse=2000) {
  if (angle < minAngle) angle = minAngle;
  if (angle > maxAngle) angle = maxAngle;
  float t = (angle - minAngle) / (maxAngle - minAngle);
  return (uint16_t)(minPulse + t * (maxPulse - minPulse));
}

static uint16_t angleToPulse2(float angle, float minAngle=0, float maxAngle=300,
                             uint16_t minPulse=500, uint16_t maxPulse=2500) {
  if (angle < minAngle) angle = minAngle;
  if (angle > maxAngle) angle = maxAngle;
  float t = (angle - minAngle) / (maxAngle - minAngle);
  return (uint16_t)(minPulse + t * (maxPulse - minPulse));
}

// ===================== Motor Control (ESP32 PWM) =====================
void setupMotorPWM() {
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  
  ledcAttachPin(MOTOR_A_EN, PWM_CHANNEL_A);
  ledcAttachPin(MOTOR_B_EN, PWM_CHANNEL_B);
  
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
}

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
  if (speed == 0 || dir == STOP) {
    stopMotors();
    return;
  }
  
  current_speed = speed;
  current_direction = dir;
  
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
  
  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, speed);
}

// ===================== Sensor init (original TOF) =====================
uint8_t initTOF() {
  if (!TOF_ENABLED) {
    pinMode(TOF_UP_XSHUT, OUTPUT);
    pinMode(TOF_DOWN_XSHUT, OUTPUT);
    digitalWrite(TOF_UP_XSHUT, LOW);
    digitalWrite(TOF_DOWN_XSHUT, LOW);
    return 0;
  }

  pinMode(TOF_UP_XSHUT, OUTPUT);
  pinMode(TOF_DOWN_XSHUT, OUTPUT);
  digitalWrite(TOF_DOWN_XSHUT, LOW);
  digitalWrite(TOF_UP_XSHUT, LOW);
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

// M-6 FIX: Non-blocking sensor recovery - call from loop()
void updateSensorRecovery(bool which_up) {
  unsigned long now = millis();
  SensorRecoveryState& state = which_up ? recovery_up_state : recovery_down_state;
  unsigned long& state_start = which_up ? recovery_up_state_start_ms : recovery_down_state_start_ms;
  uint8_t xshut_pin = which_up ? TOF_UP_XSHUT : TOF_DOWN_XSHUT;
  
  switch (state) {
    case RECOVERY_IDLE:
      // Not recovering
      break;
      
    case RECOVERY_XSHUT_LOW:
      if ((long)(now - state_start) >= 20) {
        digitalWrite(xshut_pin, HIGH);
        state = RECOVERY_XSHUT_HIGH;
        state_start = now;
      }
      break;
      
    case RECOVERY_XSHUT_HIGH:
      if ((long)(now - state_start) >= 50) {
        state = RECOVERY_INIT;
        state_start = now;
      }
      break;
      
    case RECOVERY_INIT:
      // Non-blocking init attempt
      if (which_up) {
        if (tof_up.init()) {
          tof_up.setAddress(0x30);
          tof_up.startContinuous();
          tof_up_ready = true;
          Serial.println("TOF: up re-init OK");
        } else {
          Serial.println("TOF: up re-init FAIL");
        }
        recovery_up_state = RECOVERY_IDLE;
      } else {
        if (tof_down.init()) {
          tof_down.setAddress(0x31);
          tof_down.startContinuous();
          tof_down_ready = true;
          Serial.println("TOF: down re-init OK");
        } else {
          Serial.println("TOF: down re-init FAIL");
        }
        recovery_down_state = RECOVERY_IDLE;
      }
      break;
  }
}

bool reinitOneSensor(bool which_up) {
  if (!TOF_ENABLED) {
    return false;
  }
  // M-6 FIX: Start non-blocking recovery instead of blocking
  unsigned long now = millis();
  if (which_up) {
    digitalWrite(TOF_UP_XSHUT, LOW);
    recovery_up_state = RECOVERY_XSHUT_LOW;
    recovery_up_state_start_ms = now;
  } else {
    digitalWrite(TOF_DOWN_XSHUT, LOW);
    recovery_down_state = RECOVERY_XSHUT_LOW;
    recovery_down_state_start_ms = now;
  }
  return true;  // Started recovery process
}

void readSensors() {
  if (!TOF_ENABLED) {
    tof_up_mm = TOF_SENSOR_DOWN;
    tof_down_mm = TOF_SENSOR_DOWN;
    return;
  }

  unsigned long now = millis();
  
  if ((long)(now - last_sensor_read_ms) < SENSOR_READ_TIMEOUT_MS) {
    return;
  }
  last_sensor_read_ms = now;

  uint16_t raw_up = tof_up_ready ? tof_up.readRangeContinuousMillimeters() : TOF_SENSOR_DOWN;
  uint16_t raw_down = tof_down_ready ? tof_down.readRangeContinuousMillimeters() : TOF_SENSOR_DOWN;

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

void ensureSensorsAlive() {
  if (!TOF_ENABLED) {
    return;
  }

  unsigned long now = millis();

  if (tof_up_mm == TOF_SENSOR_DOWN && (long)(now - tof_up_next_retry_ms) >= 0) {
    if (reinitOneSensor(true)) {
      tof_up_ready = true;
      tof_up_mm = TOF_SENSOR_DOWN;
      tof_up_next_retry_ms = now + TOF_RETRY_AFTER_MS;
      Serial.println("TOF: up re-init OK");
    } else {
      tof_up_next_retry_ms = now + TOF_RETRY_AFTER_MS;
      Serial.println("TOF: up re-init FAIL");
    }
  } else if (tof_up_mm != TOF_SENSOR_DOWN) {
    tof_up_next_retry_ms = now + TOF_RETRY_AFTER_MS;
  }

  if (tof_down_mm == TOF_SENSOR_DOWN && (long)(now - tof_down_next_retry_ms) >= 0) {
    if (reinitOneSensor(false)) {
      tof_down_ready = true;
      tof_down_mm = TOF_SENSOR_DOWN;
      tof_down_next_retry_ms = now + TOF_RETRY_AFTER_MS;
      Serial.println("TOF: down re-init OK");
    } else {
      tof_down_next_retry_ms = now + TOF_RETRY_AFTER_MS;
      Serial.println("TOF: down re-init FAIL");
    }
  } else if (tof_down_mm != TOF_SENSOR_DOWN) {
    tof_down_next_retry_ms = now + TOF_RETRY_AFTER_MS;
  }
}

// ===================== NEW: Altitude Control Sensor Init =====================
bool initAltitudeSensors() {
  bno_ready = bno.begin();
  if (bno_ready) {
    bno.setExtCrystalUse(true);
    Serial.println("BNO055 initialized");
  } else {
    Serial.println("BNO055 init failed");
  }

  Wire.beginTransmission(0x29);
  byte error = Wire.endTransmission();
  if (error == 0) {
    groundSensor.setTimeout(GROUND_SENSOR_TIMEOUT_MS);
    ground_sensor_ready = groundSensor.init();
    if (ground_sensor_ready) {
      groundSensor.setDistanceMode(VL53L1X::Long);
      groundSensor.setMeasurementTimingBudget(50000);
      groundSensor.startContinuous(50);
      Serial.println("VL53L1X ground sensor initialized");
    } else {
      Serial.println("VL53L1X init failed");
    }
  } else {
    Serial.println("VL53L1X not found on I2C");
    ground_sensor_ready = false;
  }

  return (bno_ready && ground_sensor_ready);
}

void readAltitudeSensors() {
  if (bno_ready) {
    sensors_event_t event;
    bno.getEvent(&event);
    headingDeg = event.orientation.x;
    pitchDeg = event.orientation.y;
    rollDeg = event.orientation.z;
  }

  if (ground_sensor_ready) {
    uint16_t rawDist = groundSensor.read(false);
    if (!groundSensor.timeoutOccurred() && rawDist < 4000) {
      groundDistanceFiltered_mm = ALPHA_GROUND * rawDist + 
                                   (1.0f - ALPHA_GROUND) * groundDistanceFiltered_mm;
    }
  }

  float pitchRad = pitchDeg * DEG_TO_RAD;
  float rollRad = rollDeg * DEG_TO_RAD;
  float cosPitch = cos(pitchRad);
  float cosRoll = cos(rollRad);
  float tiltFactor = cosPitch * cosRoll;

  if (tiltFactor < 0.01f) tiltFactor = 0.01f;

  float sensorHeight_mm = groundDistanceFiltered_mm - groundZeroOffset_mm;
  trueVerticalHeight_mm = sensorHeight_mm / tiltFactor;
}

// ===================== Helper: Select PIDG Config Based on Gripper State, Bar Pose, AND Direction =====================
const PIDGConfig& selectPIDGConfig() {
  // Both attached or both detached = no gravity compensation
  if ((upperGripperState == GRIPPER_CLOSED && lowerGripperState == GRIPPER_CLOSED) ||
      (upperGripperState == GRIPPER_OPEN && lowerGripperState == GRIPPER_OPEN)) {
    return PIDG_BOTH;
  }
  
  // Exactly one gripper attached - check direction and bar pose
  bool movingUp = (current_direction == FORWARD);
  bool movingDown = (current_direction == BACKWARD);
  
  if (movingUp) {
    // CLIMBING UP - check bar pose for mechanical advantage
    if (current_bar_pose == "up_detach") {
      return PIDG_UP_ANGLED;  // Bars angled to assist climb
    } else {
      return PIDG_UP_PARALLEL;  // Parallel bars, full weight
    }
  }
  else if (movingDown) {
    // DESCENDING DOWN - check bar pose for control
    if (current_bar_pose == "down_detach") {
      return PIDG_DOWN_ANGLED;  // Bars angled for controlled descent
    } else {
      return PIDG_DOWN_PARALLEL;  // Parallel bars, gravity assists
    }
  }
  else {
    // STOPPED or holding position - use moderate fallback
    return PIDG_BOTH;
  }
}

// ===================== NEW: Altitude Control Logic with Adaptive PIDG =====================
void updateAltitudeControl() {
  if (currentMode == MODE_MANUAL) {
    return;
  }

  if (!bno_ready || !ground_sensor_ready) {
    return;
  }

  unsigned long now = millis();
  float dt = (now - pid_last_update_ms) / 1000.0f;
  if (dt <= 0.0f || dt > 1.0f) {
    pid_last_update_ms = now;
    return;
  }
  pid_last_update_ms = now;

  // Select appropriate PIDG configuration based on gripper states
  const PIDGConfig& config = selectPIDGConfig();

  float error = targetAltitude_mm - trueVerticalHeight_mm;

  if (fabs(error) < ALTITUDE_DEADBAND_MM) {
    if (currentMode == MODE_HOLD_ALTITUDE) {
      stopMotors();
      pidIntegral = 0.0f;
      pid_last_error = 0.0f;
      return;
    }
  }

  // M-3 FIX: Anti-windup - only integrate when output is not saturated
  float output_raw = config.kp * error + config.ki * pidIntegral + config.kd * ((error - pid_last_error) / dt);
  
  // Check if we would saturate (including gravity compensation)
  bool would_saturate = (fabs(output_raw + config.gravity_compensation) > 255.0f);
  
  // Only accumulate integral if not saturated OR if error would reduce integral
  if (!would_saturate || (error * pidIntegral < 0)) {
    pidIntegral += error * dt;
    pidIntegral = constrain(pidIntegral, -500.0f, 500.0f);
  }

  float derivative = (error - pid_last_error) / dt;
  pid_last_error = error;

  // Calculate PID output
  float pidOutput = config.kp * error + config.ki * pidIntegral + config.kd * derivative;
  
  // Add gravity compensation only when gripper is detached
  float totalOutput = pidOutput + config.gravity_compensation;
  totalOutput = constrain(totalOutput, -255.0f, 255.0f);

  int speed = (int)fabs(totalOutput);
  speed = constrain(speed, 0, 255);

  // Minimum holding speed threshold - only move if above this
  if (speed < MIN_HOLD_SPEED) {
    stopMotors();
    return;
  }

  Direction dir = (totalOutput > 0) ? FORWARD : BACKWARD;
  setMotors(speed, dir);
}

// ===================== JSON Status Output (Extended) =====================
void emitStatusLine() {
  uint16_t safe_tof_up = tof_up_mm;
  uint16_t safe_tof_down = tof_down_mm;
  
  const char* dir_s = (current_direction == FORWARD) ? "FORWARD" :
                      (current_direction == BACKWARD) ? "BACKWARD" : "STOP";
  
  const char* mode_s = (currentMode == MODE_MANUAL) ? "manual" :
                       (currentMode == MODE_HOLD_ALTITUDE) ? "hold" : "run_to";

  Serial.print("STS:{"
                 "\"speed\":");
  Serial.print(current_speed);
  Serial.print(",\"dir\":\"");
  Serial.print(dir_s);
  Serial.print("\",\"pose\":\"");
  Serial.print(current_bar_pose);
  Serial.print("\",\"ack\":\"");
  Serial.print(last_cmd);
  Serial.print("\",\"latched\":");
  Serial.print(latched ? "true" : "false");
  if (latched) {
    Serial.print(",\"latch_reason\":\"");
    Serial.print(latch_reason);
    Serial.print("\"");
  }
  Serial.print(",\"mode\":\"");
  Serial.print(mode_s);
  Serial.print("\",\"target_alt\":");
  Serial.print(targetAltitude_mm, 1);
  Serial.print(",\"true_height\":");
  Serial.print(trueVerticalHeight_mm, 2);
  Serial.print(",\"pitch\":");
  Serial.print(pitchDeg, 2);
  Serial.print(",\"roll\":");
  Serial.print(rollDeg, 2);
  Serial.print(",\"gripper\":{\"upper\":\"");
  Serial.print(upperGripperState == GRIPPER_CLOSED ? "closed" : "open");
  Serial.print("\",\"lower\":\"");
  Serial.print(lowerGripperState == GRIPPER_CLOSED ? "closed" : "open");
  Serial.print("\"}");
  
  // Show which PIDG config is active for debugging
  if (currentMode != MODE_MANUAL) {
    const PIDGConfig& activeConfig = selectPIDGConfig();
    Serial.print(",\"pidg\":{\"kp\":");
    Serial.print(activeConfig.kp, 2);
    Serial.print(",\"ki\":");
    Serial.print(activeConfig.ki, 3);
    Serial.print(",\"kd\":");
    Serial.print(activeConfig.kd, 2);
    Serial.print(",\"g\":");
    Serial.print(activeConfig.gravity_compensation, 1);
    Serial.print("}");
  }
  
  Serial.print(",\"tof\":{"
                 "\"enabled\":");
  Serial.print(TOF_ENABLED ? "true" : "false");
  Serial.print(",\"up\":");
  Serial.print(safe_tof_up);
  Serial.print(",\"down\":");
  Serial.print(safe_tof_down);
  Serial.print("}}");
  Serial.println();
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  setupMotorPWM();
  stopMotors();

  Wire.begin(I2C_SDA, I2C_SCL);
  
  servo_hub.begin();
  servo_hub.reset();
  servo_hub.setPWMFreq(50);

  pinMode(TOF_UP_XSHUT, OUTPUT);
  pinMode(TOF_DOWN_XSHUT, OUTPUT);
  digitalWrite(TOF_UP_XSHUT, LOW);
  digitalWrite(TOF_DOWN_XSHUT, LOW);

  uint8_t tof_r = initTOF();
  tof_up_ready = (tof_r & TOF_INIT_UP_OK) != 0;
  tof_down_ready = (tof_r & TOF_INIT_DOWN_OK) != 0;
  sensors_ok = (tof_r != 0);
  
  unsigned long _t0 = millis();
  tof_up_next_retry_ms = _t0 + TOF_RETRY_AFTER_MS;
  tof_down_next_retry_ms = _t0 + TOF_RETRY_AFTER_MS;

  Serial.print("INIT: tof up=");
  Serial.print(tof_up_ready ? "OK" : "FAIL");
  Serial.print(" down=");
  Serial.println(tof_down_ready ? "OK" : "FAIL");

  if (tof_up_ready) {
    tof_up.setMeasurementTimingBudget(TOF_MEASUREMENT_TIME);
  }
  if (tof_down_ready) {
    tof_down.setMeasurementTimingBudget(TOF_MEASUREMENT_TIME);
  }

  if (initAltitudeSensors()) {
    Serial.println("Altitude sensors OK");
  } else {
    Serial.println("Altitude sensors FAIL (continuing anyway)");
  }

  setBarPosition("parallel");
  
  for (uint8_t ch = chServoUpL; ch <= chServoDownR; ch++) {
    servo_hub.writeMicroseconds(ch, angleToPulse(close_position));
  }
  
  Serial.println("ESP32-S3 Bridge Ready");
}

// ===================== Main loop =====================
void loop() {
  unsigned long now = millis();
  
  if (Serial.available() > 0) {
    String payload = Serial.readStringUntil('\n');
    payload.trim();
    
    if (payload.length() > 128) {
      Serial.println("ERR:line too long");
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

  if ((long)(now - next_watchdog_ms) >= 0) {
    next_watchdog_ms = now + 1000;
    ensureSensorsAlive();
  }
  
  // M-6 FIX: Update non-blocking sensor recovery state machines
  updateSensorRecovery(true);   // up sensor
  updateSensorRecovery(false);  // down sensor

  if ((long)(now - next_status_ms) >= 0) {
    next_status_ms = now + STATUS_PERIOD_MS;
    readSensors();
    readAltitudeSensors();
    emitStatusLine();
  }
  
  if (currentMode != MODE_MANUAL) {
    updateAltitudeControl();
  }

  if (current_speed > 0) {
    if (now - led_flash_timer >= 500) {
      led_state = !led_state;
      digitalWrite(LED_PIN, led_state ? HIGH : LOW);
      led_flash_timer = now;
    }
  } else {
    digitalWrite(LED_PIN, LOW);
    led_state = true;
    led_flash_timer = now;
  }
}

// ===================== Command parsing =====================
void parseAndExecutePayload(String payload) {
  int firstComma = payload.indexOf(',');
  int secondComma = payload.indexOf(',', firstComma + 1);
  int thirdComma = payload.indexOf(',', secondComma + 1);

  // Support both 3-field (old) and 4-field (new) formats
  bool hasAltitudeField = (thirdComma != -1);

  if (firstComma == -1 || secondComma == -1 || firstComma == 0) {
    Serial.println("ERR:malformed CSV");
    return;
  }

  String speedStr = payload.substring(0, firstComma);
  String directionStr = payload.substring(firstComma + 1, secondComma);
  String commandStr;
  String altitudeModeStr;

  if (hasAltitudeField) {
    // New format: speed,direction,command,altitude_mode
    commandStr = payload.substring(secondComma + 1, thirdComma);
    altitudeModeStr = payload.substring(thirdComma + 1);
  } else {
    // Old format: speed,direction,command
    commandStr = payload.substring(secondComma + 1);
    altitudeModeStr = "manual";  // Default to manual mode
  }

  commandStr.trim();
  altitudeModeStr.trim();

  if (speedStr.length() == 0 || directionStr.length() > 9 || commandStr.length() > 20 || altitudeModeStr.length() > 20) {
    Serial.println("ERR:field length");
    return;
  }

  if (latched) {
    if (commandStr == "NONE" && altitudeModeStr == "manual") {
      return;
    }
    latched = false;
    latch_reason[0] = '\0';
    // M-1 FIX: Force manual mode when clearing latch to prevent autonomous resume
    currentMode = MODE_MANUAL;
    pidIntegral = 0.0f;  // Also reset PID state
  }

  int speedVal = speedStr.toInt();
  if (speedVal == 0 && speedStr != "0") {
    Serial.println("ERR:non-numeric speed");
    return;
  }

  if (directionStr != "FORWARD" && directionStr != "BACKWARD") {
    Serial.println("ERR:invalid direction");
    return;
  }

  strncpy(current_dir, directionStr.c_str(), sizeof(current_dir) - 1);
  current_dir[sizeof(current_dir) - 1] = '\0';

  // Process altitude mode changes first
  if (altitudeModeStr != "manual" && altitudeModeStr != "") {
    executeAltitudeMode(altitudeModeStr);
  }

  // Set drive motors (only if in manual mode)
  if (currentMode == MODE_MANUAL) {
    setDriveMotors(speedVal, directionStr);
  }

  // Execute button commands
  if (commandStr != "NONE") {
    executeCommand(commandStr);
  }
}

void parseAndExecuteWarning(String payload) {
  String sensor = payload.substring(5);
  sensor.trim();
  if (sensor != "up" && sensor != "down" && sensor != "tilt") {
    return;
  }
  stopMotors();
  digitalWrite(LED_PIN, LOW);
  latched = true;
  strncpy(latch_reason, sensor.c_str(), sizeof(latch_reason) - 1);
  latch_reason[sizeof(latch_reason) - 1] = '\0';
  Serial.print("WARNED:");
  Serial.println(sensor);
}

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

void executeAltitudeMode(String mode) {
  if (mode == "hold_alt") {
    currentMode = MODE_HOLD_ALTITUDE;
    targetAltitude_mm = trueVerticalHeight_mm;
  }
  else if (mode == "zero_alt") {
    groundZeroOffset_mm = groundDistanceFiltered_mm;
    trueVerticalHeight_mm = 0.0f;
    currentMode = MODE_MANUAL;
    pid_integral = 0.0f;
    pid_last_error = 0.0f;
  }
  else if (mode == "manual") {
    currentMode = MODE_MANUAL;
    pid_integral = 0.0f;
    pid_last_error = 0.0f;
  }
  else if (mode == "set_alt") {
    currentMode = MODE_RUN_TO_ALTITUDE;
  }
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
    stopMotors();
    digitalWrite(LED_PIN, LOW);
    if (!latched) {
      latched = true;
      strncpy(latch_reason, "estop", sizeof(latch_reason) - 1);
      latch_reason[sizeof(latch_reason) - 1] = '\0';
    }
  }
  else {
    Serial.print("ACK:");
    Serial.println(cmd);
    strncpy(last_cmd, cmd.c_str(), sizeof(last_cmd) - 1);
    last_cmd[sizeof(last_cmd) - 1] = '\0';
    return;
  }
  
  strncpy(last_cmd, cmd.c_str(), sizeof(last_cmd) - 1);
  last_cmd[sizeof(last_cmd) - 1] = '\0';
  Serial.print("ACK:");
  Serial.println(cmd);
}

void setBarPosition(String pose) {
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

void setServoAngle(int channelL, int channelR, float angle) {
  servo_hub.writeMicroseconds(channelL, angleToPulse(angle));
  servo_hub.writeMicroseconds(channelR, angleToPulse(180 - angle));
}

void up_detach() {
  setServoAngle(chServoUpL, chServoUpR, open_position);
  delay(delay_to_pose);
  setBarPosition("up_detach");
  upperGripperState = GRIPPER_OPEN;  // Track state
}

void down_detach() {
  setServoAngle(chServoDownL, chServoDownR, open_position);
  delay(delay_to_pose);
  setBarPosition("down_detach");
  lowerGripperState = GRIPPER_OPEN;  // Track state
}

void up_attach() {
  setBarPosition("parallel");
  delay(delay_to_pose);
  setServoAngle(chServoUpL, chServoUpR, close_position);
  setServoAngle(chServoDownL, chServoDownR, close_position);
  upperGripperState = GRIPPER_CLOSED;  // Track state
}

void down_attach() {
  setBarPosition("parallel");
  delay(delay_to_pose);
  setServoAngle(chServoUpL, chServoUpR, close_position);
  setServoAngle(chServoDownL, chServoDownR, close_position);
  lowerGripperState = GRIPPER_CLOSED;  // Track state
}

void both_attach() {
  setBarPosition("parallel");
  delay(delay_to_pose);
  setServoAngle(chServoUpL, chServoUpR, close_position);
  setServoAngle(chServoDownL, chServoDownR, close_position);
  upperGripperState = GRIPPER_CLOSED;  // Track state
  lowerGripperState = GRIPPER_CLOSED;  // Track state
}

void both_detach() {
  setBarPosition("parallel");
  delay(delay_to_pose);
  setServoAngle(chServoUpL, chServoUpR, open_position);
  setServoAngle(chServoDownL, chServoDownR, open_position);
  upperGripperState = GRIPPER_OPEN;  // Track state
  lowerGripperState = GRIPPER_OPEN;  // Track state
}
