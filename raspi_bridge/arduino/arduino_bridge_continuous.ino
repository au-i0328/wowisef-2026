/**
 * Arduino Bridge — Continuous Rotation Variant
 * --------------------------------------------------
 * Identical to arduino_bridge.ino in every way EXCEPT:
 * - Attachment servos (UpL, UpR, DownL, DownR) run in continuous
 *   rotation mode: PWM 1000 = full-speed CCW, PWM 2000 = full-speed CW.
 *   They spin for `continuous_rotation_ms` milliseconds then stop.
 * - Bar servos (BarUpL, BarUpR, BarDownL, BarDownR) are unchanged:
 *   still positional, still driven by setBarPosition().
 * - Global variables `CONTINUOUS_ROTATION_MS`, `CONT_ROT_SPEED_LEFT`,
 *   and `CONT_ROT_SPEED_RIGHT` control duration and direction.
 *
 * The Pi-side serial protocol and all status/ACK/WARN logic are
 * identical to arduino_bridge.ino.
 */

#include <Adafruit_PWMServoDriver.h>
#include <L298N.h>
#include <Servo.h>
#include <L298NX2.h>
#include <Wire.h>
#include <VL53L0X.h>

const int TOF_UP_XSHUT = 10;
const int TOF_DOWN_XSHUT  = 11;

VL53L0X          tof_up;
VL53L0X          tof_down;

// ===================== Sensors state =====================
volatile uint16_t tof_up_mm = 0;
volatile uint16_t tof_down_mm  = 0;
volatile bool     sensors_ok = false;

// ===================== Safety latch =====================
bool     latched = false;
char     latch_reason[16] = "";

// ===================== Status export =====================
unsigned long next_status_ms = 0;
const     long STATUS_PERIOD_MS = 200;
char     last_cmd[24]   = "NONE";
String   current_bar_pose = "parallel";
char     current_dir[10] = "FORWARD";

// ===================== LEDs/serial =====================
const int ledPin = 14;
unsigned long led_flash_timer = 0;
bool led_state = true;

// --------- Motors ----------
L298NX2 motor_drive(6, 13, 12, 5, 8, 7);
unsigned int run_time_to_pose = 1500;

Adafruit_PWMServoDriver servo_hub = Adafruit_PWMServoDriver(0x40);

// Channel assignments (same as arduino_bridge.ino)
const int chServoUpL = 0;
const int chServoUpR = 1;
const int chServoDownL = 2;
const int chServoDownR = 3;
const int chServoBarUpL = 4;
const int chServoBarUpR = 5;
const int chServoBarDownL = 6;
const int chServoBarDownR = 7;

// ---------- Bar servo positions (unchanged from arduino_bridge.ino) ----------
const float open_position = 0;
const float close_position = 100;
const float angle_change = 30;
const float delay_to_pose = 800;

// ----- Continuous rotation globals (attachment servos only) -----
// Duration each attach/detach motion runs before stopping (ms).
const unsigned long CONTINUOUS_ROTATION_MS = 500;

// PWM values for the attachment servos:
//   1000 = full-speed CCW  (left side, forward)
//   2000 = full-speed CW   (right side, backward)
//   1500 = stopped
const uint16_t CONT_ROT_SPEED_LEFT  = 1000;   // CCW / "forward" direction
const uint16_t CONT_ROT_SPEED_RIGHT = 2000;   // CW  / "backward" direction
const uint16_t CONT_ROT_STOP       = 1500;   // stopped

static uint16_t angleToPulse(float angle, float minAngle=0, float maxAngle=180,
                             uint16_t minPulse=1000, uint16_t maxPulse=2000) {
  if (angle < minAngle) angle = minAngle;
  if (angle > maxAngle) angle = maxAngle;
  float t = (angle - minAngle) / (maxAngle - minAngle);
  return (uint16_t)(minPulse + t * (maxPulse - minPulse));
}

static uint16_t angleToPulse2(float angle, float minAngle=0, float maxAngle=180,
                             uint16_t minPulse=500, uint16_t maxPulse=2500) {
  if (angle < minAngle) angle = minAngle;
  if (angle > maxAngle) angle = maxAngle;
  float t = (angle - minAngle) / (maxAngle - minAngle);
  return (uint16_t)(minPulse + t * (maxPulse - minPulse));
}

// ===================== Sensor init =====================
bool initTOF() {
  pinMode(TOF_UP_XSHUT, OUTPUT);
  pinMode(TOF_DOWN_XSHUT,  OUTPUT);
  digitalWrite(TOF_DOWN_XSHUT,  LOW);
  digitalWrite(TOF_UP_XSHUT, LOW);
  digitalWrite(TOF_UP_XSHUT, HIGH);
  delay(50);
  if (!tof_up.init()) return false;
  tof_up.setAddress(0x30);
  delay(50);
  digitalWrite(TOF_DOWN_XSHUT, HIGH);
  delay(50);
  if (!tof_down.init()) return false;
  tof_down.setAddress(0x31);
  delay(50);
  tof_up.startContinuous();
  tof_down.startContinuous();
  return true;
}

void readSensors() {
  tof_up_mm = tof_up.readRangeContinuousMillimeters();
  tof_down_mm  = tof_down.readRangeContinuousMillimeters();
}

// ===================== Hand-rolled JSON (no ArduinoJson) =====================
void emitStatusLine() {
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
                 "\"up\":"));
  Serial.print(tof_up_mm);
  Serial.print(F(",\"down\":"));
  Serial.print(tof_down_mm);
  Serial.print(F("}}"));
  Serial.println();
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  motor_drive.stop();

  Wire.begin();
  servo_hub.begin();
  servo_hub.reset();
  servo_hub.setPWMFreq(50);

  bool tof_ok = initTOF();
  sensors_ok = tof_ok;
  Serial.print(F("INIT: tof="));
  Serial.println(tof_ok ? "OK" : "FAIL");

  both_attach();
}

// ===================== Main loop =====================
void loop() {
  // 1. Inbound commands / warnings
  if (Serial.available() > 0) {
    String payload = Serial.readStringUntil('\n');
    payload.trim();
    if (payload.length() > 0) {
      if (payload.startsWith("WARN:")) {
        parseAndExecuteWarning(payload);
      } else {
        parseAndExecutePayload(payload);
      }
    }
  }

  // 2. Periodic status export (200 ms)
  unsigned long now = millis();
  if ((long)(now - next_status_ms) >= 0) {
    next_status_ms = now + STATUS_PERIOD_MS;
    readSensors();
    emitStatusLine();
  }

  // 3. LED flash when drive is active
  if (motor_drive.getSpeedA() > 0) {
    if (millis() - led_flash_timer >= 150) {
      led_state = !led_state;
      digitalWrite(ledPin, led_state ? HIGH : LOW);
      led_flash_timer = millis();
    }
  } else {
    digitalWrite(ledPin, LOW);
    led_state = true;
    led_flash_timer = millis();
  }
}

// ===================== Command parsing =====================
void parseAndExecutePayload(String payload) {
  int firstComma = payload.indexOf(',');
  int secondComma = payload.indexOf(',', firstComma + 1);

  if (firstComma != -1 && secondComma != -1) {
    String speedStr     = payload.substring(0, firstComma);
    String directionStr = payload.substring(firstComma + 1, secondComma);
    String commandStr   = payload.substring(secondComma + 1);

    if (latched) {
      if (commandStr == "NONE") {
        return;
      }
      latched = false;
      latch_reason[0] = '\0';
    }

    int speedVal = speedStr.toInt();

    strncpy(current_dir, directionStr.c_str(), sizeof(current_dir) - 1);
    current_dir[sizeof(current_dir) - 1] = '\0';

    setDriveMotors(speedVal, directionStr);

    if (commandStr != "NONE") {
      executeCommand(commandStr);
    }
  }
}

// ===================== Warning handler =====================
void parseAndExecuteWarning(String payload) {
  String sensor = payload.substring(5);
  sensor.trim();
  if (sensor != "up" && sensor != "down") {
    return;
  }
  motor_drive.stop();
  digitalWrite(ledPin, LOW);
  latched = true;
  strncpy(latch_reason, sensor.c_str(), sizeof(latch_reason) - 1);
  latch_reason[sizeof(latch_reason) - 1] = '\0';
  Serial.print(F("WARNED:"));
  Serial.println(sensor);
}

void setDriveMotors(int speed, String direction){
  L298N::Direction curDirection;
  if (direction == "FORWARD") {
    curDirection = L298N::FORWARD;
  }
  else if (direction == "BACKWARD") {
    curDirection = L298N::BACKWARD;
  }

  motor_drive.setSpeed(speed);
  motor_drive.run(curDirection);

  if (speed > 0) {
    digitalWrite(ledPin, HIGH);
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
    motor_drive.stop();
    digitalWrite(ledPin, LOW);
    latched = true;
    strncpy(latch_reason, "estop", sizeof(latch_reason) - 1);
    latch_reason[sizeof(latch_reason) - 1] = '\0';
  }
  else {
    Serial.print("ACK:");
    Serial.println(cmd);
    return;
  }
  strncpy(last_cmd, cmd.c_str(), sizeof(last_cmd) - 1);
  last_cmd[sizeof(last_cmd) - 1] = '\0';
  Serial.print("ACK:");
  Serial.println(cmd);
  return;
}

// ===================== Bar servos (unchanged) =====================
void setBarPosition(String pose){
  if (pose == "parallel") {
    servo_hub.writeMicroseconds(chServoBarUpL, angleToPulse2(90));
    servo_hub.writeMicroseconds(chServoBarUpR, angleToPulse2(90));
    servo_hub.writeMicroseconds(chServoBarDownL, angleToPulse2(90));
    servo_hub.writeMicroseconds(chServoBarDownR, angleToPulse2(90));
  }
  else if (pose == "up_detach") {
    servo_hub.writeMicroseconds(chServoBarUpL, angleToPulse2(90 - angle_change));
    servo_hub.writeMicroseconds(chServoBarUpR, angleToPulse2(180 - (90 - angle_change)));
    servo_hub.writeMicroseconds(chServoBarDownL, angleToPulse2(90 + angle_change));
    servo_hub.writeMicroseconds(chServoBarDownR, angleToPulse2(180 - (90 + angle_change)));
  }
  else if (pose == "down_detach") {
    servo_hub.writeMicroseconds(chServoBarUpL, angleToPulse2(90 + angle_change));
    servo_hub.writeMicroseconds(chServoBarUpR, angleToPulse2(180 - (90 + angle_change)));
    servo_hub.writeMicroseconds(chServoBarDownL, angleToPulse2(90 - angle_change));
    servo_hub.writeMicroseconds(chServoBarDownR, angleToPulse2(180 - (90 - angle_change)));
  }
  current_bar_pose = pose;
}

// ===================== Continuous rotation attachment servos =====================
// All four attachment servos (UpL, UpR, DownL, DownR) are continuous rotation.
// PWM 1000 = full-speed CCW, PWM 2000 = full-speed CW, PWM 1500 = stopped.
// Motion runs for CONTINUOUS_ROTATION_MS milliseconds then the motors stop,
// leaving the servos in whatever orientation they happened to reach.

void runContinuousRotation() {
  servo_hub.writeMicroseconds(chServoUpL,    CONT_ROT_SPEED_LEFT);
  servo_hub.writeMicroseconds(chServoUpR,    CONT_ROT_SPEED_RIGHT);
  servo_hub.writeMicroseconds(chServoDownL,  CONT_ROT_SPEED_LEFT);
  servo_hub.writeMicroseconds(chServoDownR,  CONT_ROT_SPEED_RIGHT);
  delay(CONTINUOUS_ROTATION_MS);
  servo_hub.writeMicroseconds(chServoUpL,    CONT_ROT_STOP);
  servo_hub.writeMicroseconds(chServoUpR,    CONT_ROT_STOP);
  servo_hub.writeMicroseconds(chServoDownL,  CONT_ROT_STOP);
  servo_hub.writeMicroseconds(chServoDownR,  CONT_ROT_STOP);
}

void up_detach() {
  runContinuousRotation();
  setBarPosition("up_detach");
}

void down_detach() {
  runContinuousRotation();
  setBarPosition("down_detach");
}

void up_attach() {
  setBarPosition("parallel");
  delay(delay_to_pose);
  runContinuousRotation();
}

void down_attach() {
  setBarPosition("parallel");
  delay(delay_to_pose);
  runContinuousRotation();
}

void both_attach() {
  setBarPosition("parallel");
  delay(delay_to_pose);
  runContinuousRotation();
}

void both_detach() {
  setBarPosition("parallel");
  delay(delay_to_pose);
  runContinuousRotation();
}
