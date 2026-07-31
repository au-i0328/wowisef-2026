#include <Adafruit_PWMServoDriver.h>
#include <L298N.h>
#include <Servo.h>
#include <L298NX2.h>
#include <Wire.h>

// --------- Motors ---------
L298NX2 motor_drive(6, 13, 12, 5, 8, 7); // EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B
L298N motor_bar_up (3, 2, 4);
L298N motor_bar_down (9, 19, 18);
unsigned int run_time_to_pose = 1500; //to be tuned

Adafruit_PWMServoDriver servo_hub = Adafruit_PWMServoDriver(0x40);

const int chServoUpL = 0;
const int chServoUpR = 1;
const int chServoDownL = 2;
const int chServoDownR = 3;
const int chServoBarUpL = 4;
const int chServoBarUpR = 5;
const int chServoBarDownL = 6;
const int chServoBarDownR = 7;

const float open_position = 0; //ref to left servo absolute position
const float close_position = 100;
const float angle_change = 30; //change in angle for change in pose
const float delay_to_pose = 800;

static uint16_t angleToPulse(float angle, float minAngle=0, float maxAngle=180,
                             uint16_t minPulse=150, uint16_t maxPulse=600) {
  // Typical servo pulse range: ~150us..600us (tune if needed)
  if (angle < minAngle) angle = minAngle;
  if (angle > maxAngle) angle = maxAngle;

  float t = (angle - minAngle) / (maxAngle - minAngle);
  return (uint16_t)(minPulse + t * (maxPulse - minPulse));
}

static uint16_t angleToPulse2(float angle, float minAngle=0, float maxAngle=180,
                             uint16_t minPulse=150, uint16_t maxPulse=600) {
  // Typical servo pulse range: ~150us..600us (tune if needed)
  if (angle < minAngle) angle = minAngle;
  if (angle > maxAngle) angle = maxAngle;

  float t = (angle - minAngle) / (maxAngle - minAngle);
  return (uint16_t)(minPulse + t * (maxPulse - minPulse));
}

// --------- LED ---------
const int ledPin = 14;
unsigned long led_flash_timer = 0;
bool led_state = true;

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

  setBarPosition("parallel");

  for (uint8_t ch = chServoUpL; ch < chServoDownR; ch++) {
    servo_hub.writeMicroseconds(ch, angleToPulse(close_position));
  }

  //motor_drive.enable();
  //motor_bar_up.enable();
  //motor_bar_down.enable();
}

void loop() {
  // Check if incoming serial data is available
  if (Serial.available() > 0) {
    // Read payload up to newline character
    String payload = Serial.readStringUntil('\n');
    payload.trim(); // Strip carriage returns or trailing spaces

    if (payload.length() > 0) {
      parseAndExecutePayload(payload);
    }
  }

  // Flash LED when drive motor has power
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

void parseAndExecutePayload(String payload) {
  // Find comma positions
  int firstComma = payload.indexOf(',');
  int secondComma = payload.indexOf(',', firstComma + 1);

  if (firstComma != -1 && secondComma != -1) {
    // Extract individual data fields
    String speedStr     = payload.substring(0, firstComma);
    String directionStr = payload.substring(firstComma + 1, secondComma);
    String commandStr   = payload.substring(secondComma + 1);

    // Convert speed string to integer (0 - 255)
    int speedVal = speedStr.toInt();

    // 1. Update Drive Motor Outputs
    setDriveMotors(speedVal, directionStr);

    // 2. Execute Specific Command if present
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
  // Matches exact command strings sent by Python
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
  // Send Acknowledgement back to Python
      Serial.print("ACK:");
      Serial.println(cmd);
      return;
}

void setBarPosition(String pose){
    if (pose == "parallel") {
      servo_hub.writeMicroseconds(chServoBarUpL, angleToPulse2(90)); //add the actual positions after testing
      servo_hub.writeMicroseconds(chServoBarUpR, angleToPulse2(90));
      servo_hub.writeMicroseconds(chServoBarDownL, angleToPulse2(90));
      servo_hub.writeMicroseconds(chServoBarDownR, angleToPulse2(90));
    } 
    else if (pose == "up_detach") {
      servo_hub.writeMicroseconds(chServoBarUpL, angleToPulse2(90 - angle_change));
      servo_hub.writeMicroseconds(chServoBarUpR, angleToPulse2(180 - (90 - angle_change)));
      servo_hub.writeMicroseconds(chServoBarDownL, angleToPulse2(90 + angle_change));
      servo_hub.writeMicroseconds(chServoBarDownR, angleToPulse2(180 - (90 + angle_change)));
      //motor_bar_up.runFor(run_time_to_pose, L298N::FORWARD);
      //motor_bar_down.runFor(run_time_to_pose, L298N::BACKWARD);      
    } 
    else if (pose == "down_detach") {
      servo_hub.writeMicroseconds(chServoBarUpL, angleToPulse2(90 + angle_change));
      servo_hub.writeMicroseconds(chServoBarUpR, angleToPulse2(180 - (90 + angle_change)));
      servo_hub.writeMicroseconds(chServoBarDownL, angleToPulse2(90 - angle_change));
      servo_hub.writeMicroseconds(chServoBarDownR, angleToPulse2(180 - (90 - angle_change)));
      //motor_bar_up.runFor(run_time_to_pose, L298N::BACKWARD);
      //motor_bar_down.runFor(run_time_to_pose, L298N::FORWARD);      
    } 
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
    servo_hub.writeMicroseconds(chServoUpL, angleToPulse(close_position));
    servo_hub.writeMicroseconds(chServoUpR, angleToPulse(180 - close_position));   
    delay(delay_to_pose);
    setBarPosition("parallel");    
}

void down_attach() {
    servo_hub.writeMicroseconds(chServoDownL, angleToPulse(close_position));
    servo_hub.writeMicroseconds(chServoDownR, angleToPulse(180 - close_position));   
    delay(delay_to_pose);
    setBarPosition("parallel");     
}

void both_attach() {
    servo_hub.writeMicroseconds(chServoUpL, angleToPulse(close_position));
    servo_hub.writeMicroseconds(chServoUpR, angleToPulse(180 - close_position));   
    servo_hub.writeMicroseconds(chServoDownL, angleToPulse(close_position));
    servo_hub.writeMicroseconds(chServoDownR, angleToPulse(180 - close_position));   
    delay(delay_to_pose);
    setBarPosition("parallel");         
}

void both_detach() {
    servo_hub.writeMicroseconds(chServoUpL, angleToPulse(open_position));
    servo_hub.writeMicroseconds(chServoUpR, angleToPulse(180 - open_position));   
    servo_hub.writeMicroseconds(chServoDownL, angleToPulse(open_position));
    servo_hub.writeMicroseconds(chServoDownR, angleToPulse(180 - open_position));     
    delay(delay_to_pose);
    setBarPosition("parallel");       
}


