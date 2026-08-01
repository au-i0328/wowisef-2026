/**
 * ESP32 Climbing Robot (WiFi AP + Web Server + Camera)
 *
 * Hosts its own WiFi access point and serves:
 *   - WebSocket on ws://<host>/ws (commands IN, telemetry OUT)
 *   - MJPEG stream  on http://<host>/stream (live camera)
 *   - JSON snapshot on http://<host>/snapshot (one-shot telemetry)
 *
 * WebSocket message protocol:
 *   IN  (PC -> ESP32): "<speed>,<direction>,<command>\n"
 *   OUT (ESP32 -> PC): {"imu":{...},"tof":{...},"status":{...}}
 *
 * AP credentials (edit before flashing):
 *   SSID     : "ClimbingRobot"
 *   Password : "climb12345"
 *   IP       : 192.168.4.1
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VL53L0X.h>
#include <ArduinoJson.h>
#include <esp_camera.h>
#include <ESPmDNS.h>

// ====================== Camera pins (AI-Thinker ESP32-CAM) ======================
// If you're using a different module, change this block.
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM    5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

// ====================== AP Config ======================
const char* AP_SSID     = "ClimbingRobot";
const char* AP_PASSWORD = "climb12345";
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_NETMASK(255, 255, 255, 0);

// ====================== HTTP / WS ports ======================
const uint16_t HTTP_PORT = 80;
WebServer     httpServer(HTTP_PORT);
WebSocketsServer wsServer(81);

// ====================== Pin Map ======================
#define DRIVE_EN_A   25
#define DRIVE_IN1_A  26
#define DRIVE_IN2_A  27
#define DRIVE_EN_B   32
#define DRIVE_IN1_B  33
#define DRIVE_IN2_B  14
#define BAR_UP_PWM   15
#define BAR_UP_IN1   2
#define BAR_UP_IN2   4
#define BAR_DOWN_PWM 13
#define BAR_DOWN_IN1 12
#define BAR_DOWN_IN2 22
#define LED_PIN      21

#define I2C_SDA      18
#define I2C_SCL       5
#define SPI_SCLK     19
#define SPI_MISO     23
#define SPI_MOSI     17
#define SPI_CS       16
#define TOF_FRONT_XSHUT 35
#define TOF_REAR_XSHUT  34

// ====================== PWM ======================
const int PWM_FREQ       = 5000;
const int PWM_RESOLUTION = 8;
const int CH_DRIVE_A     = 0;
const int CH_DRIVE_B     = 1;
const int CH_BAR_UP      = 2;
const int CH_BAR_DOWN    = 3;

// ====================== Servo Geometry ======================
const float open_position  = 0;
const float close_position = 100;
const float angle_change   = 30;
const float delay_to_pose  = 800;
const int chServoUpL       = 0;
const int chServoUpR       = 1;
const int chServoDownL     = 2;
const int chServoDownR     = 3;
const int chServoBarUpL    = 4;
const int chServoBarUpR    = 5;
const int chServoBarDownL  = 6;
const int chServoBarDownR  = 7;

// ====================== Safety ======================
const unsigned long WATCHDOG_TIMEOUT_MS = 1000;
unsigned long last_cmd_ms = 0;
bool e_stop_active = false;

// ====================== LED ======================
const unsigned long LED_FLASH_PERIOD_MS = 150;
unsigned long led_flash_timer = 0;
bool led_state = true;

// ====================== Globals ======================
Adafruit_PWMServoDriver servoHub = Adafruit_PWMServoDriver(0x40);
Adafruit_MPU6050        mpu;
VL53L0X                 tofFront;
VL53L0X                 tofRear;

volatile float    imu_ax = 0, imu_ay = 0, imu_az = 0;
volatile float    imu_gx = 0, imu_gy = 0, imu_gz = 0;
volatile uint16_t tof_front_mm = 0;
volatile uint16_t tof_rear_mm  = 0;
volatile bool     drive_active = false;

// ====================== Servo helper ======================
static uint16_t angleToPulse(float angle, float minAngle = 0, float maxAngle = 180,
                             uint16_t minPulse = 1000, uint16_t maxPulse = 2000) {
  if (angle < minAngle) angle = minAngle;
  if (angle > maxAngle) angle = maxAngle;
  float t = (angle - minAngle) / (maxAngle - minAngle);
  return (uint16_t)(minPulse + t * (maxPulse - minPulse));
}

// ====================== Motor helpers ======================
void setDriveMotors(int speed, const String& direction) {
  if (e_stop_active) { ledcWrite(CH_DRIVE_A, 0); ledcWrite(CH_DRIVE_B, 0); drive_active = false; return; }
  bool forward = (direction == "FORWARD");
  digitalWrite(DRIVE_IN1_A, forward ? HIGH : LOW);
  digitalWrite(DRIVE_IN2_A, forward ? LOW  : HIGH);
  digitalWrite(DRIVE_IN1_B, forward ? HIGH : LOW);
  digitalWrite(DRIVE_IN2_B, forward ? LOW  : HIGH);
  ledcWrite(CH_DRIVE_A, speed);
  ledcWrite(CH_DRIVE_B, speed);
  drive_active = (speed > 0);
}

void stopAllMotors() {
  ledcWrite(CH_DRIVE_A, 0);
  ledcWrite(CH_DRIVE_B, 0);
  ledcWrite(CH_BAR_UP,  0);
  ledcWrite(CH_BAR_DOWN, 0);
  drive_active = false;
}

// ====================== Bar / climb sequences ======================
void setBarPosition(const String& pose) {
  if (pose == "parallel") {
    servoHub.writeMicroseconds(chServoBarUpL,   angleToPulse(90));
    servoHub.writeMicroseconds(chServoBarUpR,   angleToPulse(90));
    servoHub.writeMicroseconds(chServoBarDownL, angleToPulse(90));
    servoHub.writeMicroseconds(chServoBarDownR, angleToPulse(90));
  } else if (pose == "up_detach") {
    servoHub.writeMicroseconds(chServoBarUpL,   angleToPulse(90 - angle_change));
    servoHub.writeMicroseconds(chServoBarUpR,   angleToPulse(180 - (90 - angle_change)));
    servoHub.writeMicroseconds(chServoBarDownL, angleToPulse(90 + angle_change));
    servoHub.writeMicroseconds(chServoBarDownR, angleToPulse(180 - (90 + angle_change)));
  } else if (pose == "down_detach") {
    servoHub.writeMicroseconds(chServoBarUpL,   angleToPulse(90 + angle_change));
    servoHub.writeMicroseconds(chServoBarUpR,   angleToPulse(180 - (90 + angle_change)));
    servoHub.writeMicroseconds(chServoBarDownL, angleToPulse(90 - angle_change));
    servoHub.writeMicroseconds(chServoBarDownR, angleToPulse(180 - (90 - angle_change)));
  }
}
void up_attach()   { setBarPosition("parallel"); delay(delay_to_pose);
  servoHub.writeMicroseconds(chServoUpL, angleToPulse(close_position));
  servoHub.writeMicroseconds(chServoUpR, angleToPulse(180 - close_position)); }
void up_detach()   { servoHub.writeMicroseconds(chServoUpL, angleToPulse(open_position));
  servoHub.writeMicroseconds(chServoUpR, angleToPulse(180 - open_position));
  delay(delay_to_pose); setBarPosition("up_detach"); }
void down_attach() { setBarPosition("parallel"); delay(delay_to_pose);
  servoHub.writeMicroseconds(chServoDownL, angleToPulse(close_position));
  servoHub.writeMicroseconds(chServoDownR, angleToPulse(180 - close_position)); }
void down_detach() { servoHub.writeMicroseconds(chServoDownL, angleToPulse(open_position));
  servoHub.writeMicroseconds(chServoDownR, angleToPulse(180 - open_position));
  delay(delay_to_pose); setBarPosition("down_detach"); }
void both_attach() { setBarPosition("parallel"); delay(delay_to_pose);
  servoHub.writeMicroseconds(chServoUpL, angleToPulse(close_position));
  servoHub.writeMicroseconds(chServoUpR, angleToPulse(180 - close_position));
  servoHub.writeMicroseconds(chServoDownL, angleToPulse(close_position));
  servoHub.writeMicroseconds(chServoDownR, angleToPulse(180 - close_position)); }
void both_detach() { setBarPosition("parallel"); delay(delay_to_pose);
  servoHub.writeMicroseconds(chServoUpL, angleToPulse(open_position));
  servoHub.writeMicroseconds(chServoUpR, angleToPulse(180 - open_position));
  servoHub.writeMicroseconds(chServoDownL, angleToPulse(open_position));
  servoHub.writeMicroseconds(chServoDownR, angleToPulse(180 - open_position)); }

void executeCommand(const String& cmd) {
  if      (cmd == "up_attach")   up_attach();
  else if (cmd == "up_detach")   up_detach();
  else if (cmd == "down_attach") down_attach();
  else if (cmd == "down_detach") down_detach();
  else if (cmd == "both_attach") both_attach();
  else if (cmd == "both_detach") both_detach();
  else if (cmd == "estop")      { e_stop_active = true; stopAllMotors(); }
  else return;
}

void parsePayload(const String& payload) {
  int firstComma  = payload.indexOf(',');
  int secondComma = payload.indexOf(',', firstComma + 1);
  if (firstComma < 0 || secondComma < 0) return;
  int    speed     = payload.substring(0, firstComma).toInt();
  String direction = payload.substring(firstComma + 1, secondComma);
  String command   = payload.substring(secondComma + 1);
  command.trim();
  if (command == "estop") e_stop_active = true;
  setDriveMotors(speed, direction);
  if (!e_stop_active && command != "NONE" && command.length() > 0) executeCommand(command);
}

// ====================== Sensor init ======================
bool initIMU() {
  if (!mpu.begin()) return false;
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  return true;
}
bool initTOF() {
  pinMode(TOF_FRONT_XSHUT, OUTPUT);
  pinMode(TOF_REAR_XSHUT,  OUTPUT);
  digitalWrite(TOF_REAR_XSHUT,  LOW);
  digitalWrite(TOF_FRONT_XSHUT, LOW);
  digitalWrite(TOF_FRONT_XSHUT, HIGH);
  delay(50);
  if (!tofFront.init()) return false;
  tofFront.setAddress(0x30);
  delay(50);
  digitalWrite(TOF_REAR_XSHUT, HIGH);
  delay(50);
  if (!tofRear.init()) return false;
  tofRear.setAddress(0x31);
  delay(50);
  tofFront.startContinuous();
  tofRear.startContinuous();
  return true;
}

bool initCamera() {
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_4;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0       = Y2_GPIO_NUM;
  cfg.pin_d1       = Y3_GPIO_NUM;
  cfg.pin_d2       = Y4_GPIO_NUM;
  cfg.pin_d3       = Y5_GPIO_NUM;
  cfg.pin_d4       = Y6_GPIO_NUM;
  cfg.pin_d5       = Y7_GPIO_NUM;
  cfg.pin_d6       = Y8_GPIO_NUM;
  cfg.pin_d7       = Y9_GPIO_NUM;
  cfg.pin_xclk     = XCLK_GPIO_NUM;
  cfg.pin_pclk     = PCLK_GPIO_NUM;
  cfg.pin_vsync    = VSYNC_GPIO_NUM;
  cfg.pin_href     = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM;
  cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn     = PWDN_GPIO_NUM;
  cfg.pin_reset    = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.grab_mode    = CAMERA_GRAB_LATEST;

  cfg.frame_size   = FRAMESIZE_VGA;  // 640x480; drop to CIF if WiFi bandwidth is tight
  cfg.jpeg_quality = 12;             // 10-15 is a good range for streaming
  cfg.fb_count     = 2;

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
  return true;
}

// ====================== MJPEG stream ======================
// Boundary string used by browsers / VLC
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE =
  "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY =
  "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART =
  "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

void handleStream() {
  WiFiClient client = httpServer.client();
  if (!client) return;
  client.setNoDelay(true);
  client.print(F("HTTP/1.1 200 OK\r\n"
                 "Content-Type: "));
  client.print(STREAM_CONTENT_TYPE);
  client.print(F("\r\nAccess-Control-Allow-Origin: *\r\n\r\n"));

  camera_fb_t* fb = NULL;
  while (client.connected()) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera frame failed");
      continue;
    }
    client.print(STREAM_BOUNDARY);
    char buf[64];
    snprintf(buf, sizeof(buf), STREAM_PART, (unsigned)fb->len);
    client.print(buf);
    client.write(fb->buf, fb->len);
    esp_camera_fb_return(fb);

    // Yield to other tasks
    delay(10);
  }
}

void handleSnapshot() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { httpServer.send(500, "text/plain", "camera fail"); return; }
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.setContentLength(fb->len);
  httpServer.send(200, "image/jpeg", "");
  WiFiClient client = httpServer.client();
  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void handleSnapshotJson() {
  StaticJsonDocument<256> doc;
  doc["imu"]["ax"] = imu_ax; doc["imu"]["ay"] = imu_ay; doc["imu"]["az"] = imu_az;
  doc["imu"]["gx"] = imu_gx; doc["imu"]["gy"] = imu_gy; doc["imu"]["gz"] = imu_gz;
  doc["tof"]["front"] = tof_front_mm;
  doc["tof"]["rear"]  = tof_rear_mm;
  doc["status"]["drive_active"] = drive_active;
  doc["status"]["e_stop"] = e_stop_active;
  doc["status"]["uptime_ms"] = (unsigned long)millis();

  String out;
  serializeJson(doc, out);
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.send(200, "application/json", out);
}

void handleIndex() {
  const char html[] PROGMEM = R"html(
<!doctype html>
<html><head><title>Climbing Robot</title>
<style>body{font-family:system-ui;background:#111;color:#eee;text-align:center;margin:0;padding:1rem}
img{max-width:90%;border:2px solid #333;border-radius:8px}
a{color:#6cf}</style></head><body>
<h2>Climbing Robot</h2>
<img src="/stream" alt="camera stream"/><br/>
<a href="/snapshot">Snapshot</a> &middot; <a href="/snapshot.json">Telemetry JSON</a>
</body></html>
  )html";
  httpServer.send(200, "text/html", html);
}

// ====================== WebSocket ======================
void wsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: {
      IPAddress ip = wsServer.remoteIP(num);
      Serial.printf("WS client %u connected from %s\n", num, ip.toString().c_str());
      break;
    }
    case WStype_TEXT: {
      String p((const char*)payload, length);
      p.trim();
      last_cmd_ms = millis();
      e_stop_active = false;
      parsePayload(p);
      break;
    }
    case WStype_DISCONNECTED:
      Serial.printf("WS client %u disconnected\n", num);
      break;
    default: break;
  }
}

void broadcastTelemetry() {
  StaticJsonDocument<256> doc;
  doc["imu"]["ax"] = imu_ax; doc["imu"]["ay"] = imu_ay; doc["imu"]["az"] = imu_az;
  doc["imu"]["gx"] = imu_gx; doc["imu"]["gy"] = imu_gy; doc["imu"]["gz"] = imu_gz;
  doc["tof"]["front"] = tof_front_mm;
  doc["tof"]["rear"]  = tof_rear_mm;
  doc["status"]["drive_active"] = drive_active;
  doc["status"]["e_stop"] = e_stop_active;
  doc["status"]["uptime_ms"] = (unsigned long)millis();

  String out;
  serializeJson(doc, out);
  wsServer.broadcastTXT(out);
}

// ====================== Sensor task (core 0) ======================
void sensorTask(void* arg) {
  while (true) {
    sensors_event_t a, g, t;
    if (mpu.getEvent(&a, &g, &t)) {
      imu_ax = a.acceleration.x; imu_ay = a.acceleration.y; imu_az = a.acceleration.z;
      imu_gx = g.gyro.x;         imu_gy = g.gyro.y;         imu_gz = g.gyro.z;
    }
    tof_front_mm = tofFront.readRangeContinuousMillimeters();
    tof_rear_mm  = tofRear.readRangeContinuousMillimeters();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// ====================== Setup ======================
void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
  pinMode(DRIVE_IN1_A, OUTPUT); pinMode(DRIVE_IN2_A, OUTPUT);
  pinMode(DRIVE_IN1_B, OUTPUT); pinMode(DRIVE_IN2_B, OUTPUT);
  pinMode(BAR_UP_IN1,  OUTPUT); pinMode(BAR_UP_IN2,  OUTPUT);
  pinMode(BAR_DOWN_IN1,OUTPUT); pinMode(BAR_DOWN_IN2,OUTPUT);

  ledcSetup(CH_DRIVE_A,  PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(DRIVE_EN_A,  CH_DRIVE_A);
  ledcSetup(CH_DRIVE_B,  PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(DRIVE_EN_B,  CH_DRIVE_B);
  ledcSetup(CH_BAR_UP,   PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(BAR_UP_PWM,  CH_BAR_UP);
  ledcSetup(CH_BAR_DOWN, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(BAR_DOWN_PWM,CH_BAR_DOWN);

  Wire.begin(I2C_SDA, I2C_SCL);
  servoHub.begin();
  servoHub.reset();
  servoHub.setPWMFreq(50);

  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, SPI_CS);

  initIMU();
  initTOF();
  bool cam_ok = initCamera();

  for (uint8_t ch = chServoUpL; ch <= chServoDownR; ch++)
    servoHub.writeMicroseconds(ch, angleToPulse(close_position));
  setBarPosition("parallel");

  // ----- WiFi AP -----
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, AP_NETMASK);
  WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, 4);  // channel 1, max 4 clients
  Serial.print("AP started: "); Serial.print(AP_SSID);
  Serial.print(" IP "); Serial.println(WiFi.softAPIP());

  if (MDNS.begin("climbing-robot")) {
    Serial.println("mDNS: http://climbing-robot.local/");
  }

  // ----- HTTP routes -----
  httpServer.on("/",              HTTP_GET, handleIndex);
  httpServer.on("/stream",        HTTP_GET, handleStream);
  httpServer.on("/snapshot",      HTTP_GET, handleSnapshot);
  httpServer.on("/snapshot.json", HTTP_GET, handleSnapshotJson);
  httpServer.begin();

  // ----- WebSocket -----
  wsServer.begin();
  wsServer.onEvent(wsEvent);

  // Sensor task on core 0
  xTaskCreatePinnedToCore(sensorTask, "Sensor", 4096, NULL, 1, NULL, 0);

  Serial.print("Camera: "); Serial.println(cam_ok ? "OK" : "FAIL");
  last_cmd_ms = millis();
}

unsigned long last_telemetry_ms = 0;

void loop() {
  httpServer.handleClient();
  wsServer.loop();

  if (millis() - last_telemetry_ms >= 1000) {
    last_telemetry_ms = millis();
    broadcastTelemetry();
  }

  if (millis() - last_cmd_ms > WATCHDOG_TIMEOUT_MS) {
    ledcWrite(CH_DRIVE_A, 0);
    ledcWrite(CH_DRIVE_B, 0);
    drive_active = false;
  }

  if (drive_active) {
    if (millis() - led_flash_timer >= LED_FLASH_PERIOD_MS) {
      led_state = !led_state;
      digitalWrite(LED_PIN, led_state ? HIGH : LOW);
      led_flash_timer = millis();
    }
  } else {
    digitalWrite(LED_PIN, LOW);
    led_state = true;
    led_flash_timer = millis();
  }
  delay(2);
}