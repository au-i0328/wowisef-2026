/**
 * Arduino Bridge — extends sketch_jul30/sketch_jul30.ino
 * --------------------------------------------------
 *  - Motor control, servos, bar-pose logic: unchanged from sketch_jul30
 *  - Adds: 2x VL53L0X TOF read (up + down) via XSHUT re-addressing
 *  - Adds: 200 ms periodic JSON status exports with prefix "STS:"
 *  - Keeps: existing `ACK:<cmd>` lines emitted only on command events
 *
 * Serial protocol on Arduino -> Pi (line oriented, ASCII):
 *   STS:{...json...}      - one per 200 ms (telemetry, with sensor data)
 *   ACK:<cmd>             - one per command event (e.g. ACK:up_attach)
 *
 * Serial protocol on Pi -> Arduino (CSV line, newline-terminated):
 *   <speed>,<dir>,<cmd>   - e.g. "150,FORWARD,NONE\n"
 *   <dir> in {FORWARD, BACKWARD}
 *   <cmd> in {NONE, up_attach, up_detach, down_attach, down_detach,
 *             both_attach, both_detach, estop}
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

VL53L0X          tof_up;
VL53L0X          tof_down;

// ===================== Sensors state =====================
volatile uint16_t tof_up_mm = 0;
volatile uint16_t tof_down_mm  = 0;
volatile bool     sensors_ok = false;

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

const int chServoUpL = 0;
const int chServoUpR = 1;
const int chServoDownL = 2;
const int chServoDownR = 3;
const int chServoBarUpL = 4;
const int chServoBarUpR = 5;
const int chServoBarDownL = 6;
const int chServoBarDownR = 7;

const float open_position = 0; //degrees
const float close_position = 100; //degrees
const float angle_change = 30; //degrees
const float delay_to_pose = 800; //ms

static uint16_t angleToPulse(float angle, float minAngle=0, float maxAngle=180, //for SG90
                             uint16_t minPulse=1000, uint16_t maxPulse=2000) {
  if (angle < minAngle) angle = minAngle;
  if (angle > maxAngle) angle = maxAngle;
  float t = (angle - minAngle) / (maxAngle - minAngle);
  return (uint16_t)(minPulse + t * (maxPulse - minPulse));
}

static uint16_t angleToPulse2(float angle, float minAngle=0, float maxAngle=180, //for DS3218MG
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
  unsigned int speed = motor_drive.getSpeed();
  L298N::Direction d = motor_drive.getDirection();
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
  Serial.print(F("\",\"tof\":{"
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
  // 1. Inbound commands
  if (Serial.available() > 0) {
    String payload = Serial.readStringUntil('\n');
    payload.trim();
    if (payload.length() > 0) {
      parseAndExecutePayload(payload);
    }
  }

  // 2. Periodic status export (200 ms)
  unsigned long now = millis();
  if ((long)(now - next_status_ms) >= 0) {
    next_status_ms = now + STATUS_PERIOD_MS;
    readSensors();
    emitStatusLine();
  }

  // 3. LED flash when drive is active (preserved)
  if (motor_drive.getSpeed() > 0) {
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

// ===================== Command parsing (preserved + extended) =====================
void parseAndExecutePayload(String payload) {
  int firstComma = payload.indexOf(',');
  int secondComma = payload.indexOf(',', firstComma + 1);

  if (firstComma != -1 && secondComma != -1) {
    String speedStr     = payload.substring(0, firstComma);
    String directionStr = payload.substring(firstComma + 1, secondComma);
    String commandStr   = payload.substring(secondComma + 1);

    int speedVal = speedStr.toInt();

    // Mirror direction string for status output
    strncpy(current_dir, directionStr.c_str(), sizeof(current_dir) - 1);
    current_dir[sizeof(current_dir) - 1] = '\0';

    setDriveMotors(speedVal, directionStr);

    if (commandStr != "NONE") {
      executeCommand(commandStr);
    }
  }
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

void up_detach() {
  servo_hub.writeMicroseconds(chServoUpL, angleToPulse(open_position));
  servo_hub.writeMicroseconds(chServoUpR, angleToPulse(180 - open_position));
  delay(delay_to_pose);
  setBarPosition("up_detach");
}

void down_detach() {
  servo_hub.writeMicroseconds(chServoDownL, angleToPulse(open_position));
  servo_hub.writeMicroseconds(chServoDownR, angleToPulse(180 - open_position));
  delay(delay_to_pose);
  setBarPosition("down_detach");
}

void up_attach() {
  setBarPosition("parallel");
  delay(delay_to_pose);
  servo_hub.writeMicroseconds(chServoUpL, angleToPulse(close_position));
  servo_hub.writeMicroseconds(chServoUpR, angleToPulse(180 - close_position));
}

void down_attach() {
  setBarPosition("parallel");
  delay(delay_to_pose);
  servo_hub.writeMicroseconds(chServoDownL, angleToPulse(close_position));
  servo_hub.writeMicroseconds(chServoDownR, angleToPulse(180 - close_position));
}

void both_attach() {
  setBarPosition("parallel");
  delay(delay_to_pose);
  servo_hub.writeMicroseconds(chServoUpL, angleToPulse(close_position));
  servo_hub.writeMicroseconds(chServoUpR, angleToPulse(180 - close_position));
  servo_hub.writeMicroseconds(chServoDownL, angleToPulse(close_position));
  servo_hub.writeMicroseconds(chServoDownR, angleToPulse(180 - close_position));
}

void both_detach() {
  setBarPosition("parallel");
  delay(delay_to_pose);
  servo_hub.writeMicroseconds(chServoUpL, angleToPulse(open_position));
  servo_hub.writeMicroseconds(chServoUpR, angleToPulse(180 - open_position));
  servo_hub.writeMicroseconds(chServoDownL, angleToPulse(open_position));
  servo_hub.writeMicroseconds(chServoDownR, angleToPulse(180 - open_position));
}