/*
 * ============================================================
 *  ESP32 Robot Car — MQTT Command Receiver
 * ============================================================
 *  Receives movement commands from ROS2 CommandCenterNode via
 *  MQTT topic /esp32 and drives motors accordingly.
 *
 *  Libraries required (install via Arduino Library Manager):
 *    - PubSubClient  by Nick O'Leary
 *    - ArduinoJson   by Benoit Blanchon
 *    - WiFi          (built-in ESP32)
 * ============================================================
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ============================================================
//  WIFI & MQTT CREDENTIALS
// ============================================================
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const char* MQTT_BROKER   = "192.168.1.100";   // IP of your broker (same as in ROS nodes)
const int   MQTT_PORT     = 1883;
const char* MQTT_USER     = "";                 // leave empty if no auth
const char* MQTT_PASS     = "";
const char* MQTT_CLIENT_ID = "esp32_robot_car";

// MQTT topics
const char* TOPIC_COMMAND  = "/esp32";          // commands arrive here
const char* TOPIC_STATUS   = "/esp32/status";   // ESP32 publishes its status here

// ============================================================
//  PIN CONFIGURATION  —  change only here if wiring changes
// ============================================================

// --- Motor A (LEFT motor) ---
#define PIN_MOTOR_A_IN1   27    // direction pin 1
#define PIN_MOTOR_A_IN2   26    // direction pin 2
#define PIN_MOTOR_A_EN    14    // PWM enable (speed control)

// --- Motor B (RIGHT motor) ---
#define PIN_MOTOR_B_IN3   25    // direction pin 1
#define PIN_MOTOR_B_IN4   33    // direction pin 2
#define PIN_MOTOR_B_EN    32    // PWM enable (speed control)

// --- Ultrasonic Sensor (HC-SR04) ---
#define PIN_ULTRASONIC_TRIG  5
#define PIN_ULTRASONIC_ECHO  18

// ============================================================
//  MOTOR SPEED  (0–255)
// ============================================================
const int MOTOR_SPEED = 200;    // default drive speed — tweak if motors too fast/slow

// ============================================================
//  TIMING VARIABLE
//  All turn durations are multiples of this base period (ms).
//  If turns feel wrong, change ONLY this value.
// ============================================================
const int BASE_PERIOD_MS = 400;   // 1 full period = 400 ms

/*
  Command durations derived from BASE_PERIOD_MS:
  ┌──────────────┬─────────────────────────────────────────┐
  │ Command      │ Duration                                │
  ├──────────────┼─────────────────────────────────────────┤
  │ UP           │ continuous (until next command)         │
  │ DOWN         │ continuous (until next command)         │
  │ LEFT         │ 1  × BASE_PERIOD_MS  (sharp left turn)  │
  │ RIGHT        │ 1  × BASE_PERIOD_MS  (sharp right turn) │
  │ UP_RIGHT     │ 0.5× BASE_PERIOD_MS  (gentle right)     │
  │ UP_LEFT      │ 1  × BASE_PERIOD_MS  (gentle left)      │
  │ DOWN_LEFT    │ 1.5× BASE_PERIOD_MS  (reverse left)     │
  │ DOWN_RIGHT   │ 1.5× BASE_PERIOD_MS  (reverse right)    │
  └──────────────┴─────────────────────────────────────────┘
*/

// ============================================================
//  SAFETY — ULTRASONIC STOP DISTANCE
//  If an obstacle is closer than this (cm), motors stop.
//  Change only this value to adjust sensitivity.
// ============================================================
const int STOP_DISTANCE_CM = 20;   // stop if obstacle within 20 cm

// ============================================================
//  INTERNAL — do not edit below unless you know what you're doing
// ============================================================

// PWM channels (ESP32 LEDC)
#define PWM_CHANNEL_A   0
#define PWM_CHANNEL_B   1
#define PWM_FREQ        1000    // Hz
#define PWM_RESOLUTION  8       // bits (0–255)

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

// Last successfully processed sequence number (deduplication)
int lastSeq = -1;

// Safety flag — set true when obstacle detected
bool obstacleDetected = false;

// ============================================================
//  MOTOR DRIVER — low-level helpers
// ============================================================

void motorAForward() {
  digitalWrite(PIN_MOTOR_A_IN1, HIGH);
  digitalWrite(PIN_MOTOR_A_IN2, LOW);
  ledcWrite(PWM_CHANNEL_A, MOTOR_SPEED);
}

void motorABackward() {
  digitalWrite(PIN_MOTOR_A_IN1, LOW);
  digitalWrite(PIN_MOTOR_A_IN2, HIGH);
  ledcWrite(PWM_CHANNEL_A, MOTOR_SPEED);
}

void motorAStop() {
  digitalWrite(PIN_MOTOR_A_IN1, LOW);
  digitalWrite(PIN_MOTOR_A_IN2, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
}

void motorBForward() {
  digitalWrite(PIN_MOTOR_B_IN3, HIGH);
  digitalWrite(PIN_MOTOR_B_IN4, LOW);
  ledcWrite(PWM_CHANNEL_B, MOTOR_SPEED);
}

void motorBBackward() {
  digitalWrite(PIN_MOTOR_B_IN3, LOW);
  digitalWrite(PIN_MOTOR_B_IN4, HIGH);
  ledcWrite(PWM_CHANNEL_B, MOTOR_SPEED);
}

void motorBStop() {
  digitalWrite(PIN_MOTOR_B_IN3, LOW);
  digitalWrite(PIN_MOTOR_B_IN4, LOW);
  ledcWrite(PWM_CHANNEL_B, 0);
}

void stopAllMotors() {
  motorAStop();
  motorBStop();
}

// ============================================================
//  MOVEMENT COMMANDS
// ============================================================

/*
  Motor layout (top-down view):
        FRONT
    [A-LEFT] [B-RIGHT]
        BACK
*/

// Both motors forward — drive straight
void moveUp() {
  Serial.println("[MOVE] UP — straight forward");
  motorAForward();
  motorBForward();
  // Continuous — no delay; next command will change state
}

// Both motors backward
void moveDown() {
  Serial.println("[MOVE] DOWN — straight backward");
  motorABackward();
  motorBBackward();
}

// Sharp LEFT turn — right motor forward, left motor backward
// Duration: 1 × BASE_PERIOD_MS
void moveLeft() {
  Serial.printf("[MOVE] LEFT — sharp turn (%d ms)\n", BASE_PERIOD_MS);
  motorABackward();    // left motor backward
  motorBForward();     // right motor forward
  delay(BASE_PERIOD_MS * 1);
  stopAllMotors();
}

// Sharp RIGHT turn — left motor forward, right motor backward
// Duration: 1 × BASE_PERIOD_MS
void moveRight() {
  Serial.printf("[MOVE] RIGHT — sharp turn (%d ms)\n", BASE_PERIOD_MS);
  motorAForward();     // left motor forward
  motorBBackward();    // right motor backward
  delay(BASE_PERIOD_MS * 1);
  stopAllMotors();
}

// Gentle right curve — half period
// Duration: 0.5 × BASE_PERIOD_MS
void moveUpRight() {
  int duration = BASE_PERIOD_MS / 2;
  Serial.printf("[MOVE] UP_RIGHT — gentle right (%d ms)\n", duration);
  motorAForward();     // left motor forward
  motorBBackward();    // right motor backward
  delay(duration);
  stopAllMotors();
}

// Gentle left curve — 1 full period
// Duration: 1 × BASE_PERIOD_MS
void moveUpLeft() {
  Serial.printf("[MOVE] UP_LEFT — gentle left (%d ms)\n", BASE_PERIOD_MS);
  motorABackward();    // left motor backward
  motorBForward();     // right motor forward
  delay(BASE_PERIOD_MS * 1);
  stopAllMotors();
}

// Reverse left sweep — 1.5 × period
// Duration: 1.5 × BASE_PERIOD_MS
void moveDownLeft() {
  int duration = (BASE_PERIOD_MS * 3) / 2;    // 1.5× using integer math
  Serial.printf("[MOVE] DOWN_LEFT — reverse left (%d ms)\n", duration);
  motorAForward();     // left motor forward (reverse left = swing left while going back)
  motorBBackward();    // right motor backward
  delay(duration);
  stopAllMotors();
}

// Reverse right sweep — 1.5 × period
// Duration: 1.5 × BASE_PERIOD_MS
void moveDownRight() {
  int duration = (BASE_PERIOD_MS * 3) / 2;
  Serial.printf("[MOVE] DOWN_RIGHT — reverse right (%d ms)\n", duration);
  motorABackward();    // left motor backward
  motorBForward();     // right motor forward
  delay(duration);
  stopAllMotors();
}

// ============================================================
//  COMMAND DISPATCHER
// ============================================================

void executeCommand(const String& cmd) {

  // Safety check before every move
  if (obstacleDetected) {
    Serial.println("[SAFETY] Obstacle detected — command blocked: " + cmd);
    mqttClient.publish(TOPIC_STATUS, "{\"status\":\"BLOCKED\",\"reason\":\"obstacle\"}");
    return;
  }

  // Publish acknowledgement
  String ack = "{\"status\":\"executing\",\"cmd\":\"" + cmd + "\"}";
  mqttClient.publish(TOPIC_STATUS, ack.c_str());

  if      (cmd == "UP")         moveUp();
  else if (cmd == "DOWN")       moveDown();
  else if (cmd == "LEFT")       moveLeft();
  else if (cmd == "RIGHT")      moveRight();
  else if (cmd == "UP_RIGHT")   moveUpRight();
  else if (cmd == "UP_LEFT")    moveUpLeft();
  else if (cmd == "DOWN_LEFT")  moveDownLeft();
  else if (cmd == "DOWN_RIGHT") moveDownRight();

  // "Move forward" / "Move left" / "Move right" aliases from CV node
  else if (cmd == "Move forward")  moveUp();
  else if (cmd == "Move left")     moveLeft();
  else if (cmd == "Move right")    moveRight();
  else if (cmd == "Move backward") moveDown();

  else {
    Serial.println("[WARN] Unknown command: " + cmd);
    mqttClient.publish(TOPIC_STATUS, "{\"status\":\"unknown_cmd\"}");
  }
}

// ============================================================
//  ULTRASONIC SENSOR
// ============================================================

long readDistanceCM() {
  // Send 10µs pulse
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

  // Read echo — timeout after 30 ms (≈ 5 m range)
  long duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, 30000);

  if (duration == 0) return 999;          // no echo = clear path, return large number

  long distanceCm = duration / 58;        // standard HC-SR04 formula
  return distanceCm;
}

void checkUltrasonic() {
  long dist = readDistanceCM();

  if (dist <= STOP_DISTANCE_CM) {
    if (!obstacleDetected) {              // only act on state change
      obstacleDetected = true;
      stopAllMotors();
      Serial.printf("[SAFETY] *** OBSTACLE at %ld cm — motors stopped! ***\n", dist);
      String payload = "{\"status\":\"OBSTACLE\",\"distance_cm\":" + String(dist) + "}";
      mqttClient.publish(TOPIC_STATUS, payload.c_str());
    }
  } else {
    if (obstacleDetected) {
      obstacleDetected = false;
      Serial.printf("[SAFETY] Path clear (%ld cm) — resuming.\n", dist);
      mqttClient.publish(TOPIC_STATUS, "{\"status\":\"CLEAR\"}");
    }
  }
}

// ============================================================
//  MQTT CALLBACK — called whenever a message arrives on /esp32
// ============================================================

void mqttCallback(char* topic, byte* payload, unsigned int length) {

  // Convert payload bytes to String
  String raw = "";
  for (unsigned int i = 0; i < length; i++) {
    raw += (char)payload[i];
  }

  Serial.println("[MQTT] Received on " + String(topic) + ": " + raw);

  // Parse JSON — {"seq": 1, "cmd": "RIGHT", "timestamp": 1234567.89}
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, raw);

  if (err) {
    Serial.println("[ERROR] JSON parse failed: " + String(err.c_str()));
    return;
  }

  // Extract fields
  int    seq = doc["seq"]  | -1;
  String cmd = doc["cmd"]  | "";

  if (cmd.isEmpty()) {
    Serial.println("[WARN] Empty cmd field — ignoring.");
    return;
  }

  // Sequence-based deduplication — drop already-seen messages
  if (seq != -1 && seq <= lastSeq) {
    Serial.printf("[DEDUP] seq=%d already processed (last=%d) — skipping.\n", seq, lastSeq);
    return;
  }
  lastSeq = seq;

  Serial.printf("[CMD] seq=%d  cmd='%s'\n", seq, cmd.c_str());
  executeCommand(cmd);
}

// ============================================================
//  WIFI
// ============================================================

void connectWiFi() {
  Serial.print("[WiFi] Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected — IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[WiFi] FAILED — rebooting in 5 s");
    delay(5000);
    ESP.restart();
  }
}

// ============================================================
//  MQTT CONNECTION
// ============================================================

void connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("[MQTT] Connecting to broker...");

    bool connected = (strlen(MQTT_USER) > 0)
      ? mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)
      : mqttClient.connect(MQTT_CLIENT_ID);

    if (connected) {
      Serial.println(" connected.");
      mqttClient.subscribe(TOPIC_COMMAND);
      Serial.println("[MQTT] Subscribed to " + String(TOPIC_COMMAND));
      mqttClient.publish(TOPIC_STATUS, "{\"status\":\"online\"}");
    } else {
      Serial.printf(" failed (rc=%d). Retry in 3 s\n", mqttClient.state());
      delay(3000);
    }
  }
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n========== ESP32 Robot Car Booting ==========");

  // --- Motor pins ---
  pinMode(PIN_MOTOR_A_IN1, OUTPUT);
  pinMode(PIN_MOTOR_A_IN2, OUTPUT);
  pinMode(PIN_MOTOR_B_IN3, OUTPUT);
  pinMode(PIN_MOTOR_B_IN4, OUTPUT);

  // ESP32 LEDC PWM setup
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_MOTOR_A_EN, PWM_CHANNEL_A);
  ledcAttachPin(PIN_MOTOR_B_EN, PWM_CHANNEL_B);

  stopAllMotors();
  Serial.println("[INIT] Motor driver ready.");

  // --- Ultrasonic pins ---
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);
  Serial.println("[INIT] Ultrasonic sensor ready.");

  // --- WiFi ---
  connectWiFi();

  // --- MQTT ---
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);    // large enough for JSON payload
  connectMQTT();

  Serial.println("========== Boot complete ==========\n");
}

// ============================================================
//  LOOP
// ============================================================

void loop() {
  // Reconnect if WiFi dropped
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Lost connection — reconnecting...");
    connectWiFi();
  }

  // Reconnect if MQTT broker dropped
  if (!mqttClient.connected()) {
    connectMQTT();
  }

  // Process incoming MQTT messages
  mqttClient.loop();

  // Poll ultrasonic sensor every 100 ms
  static unsigned long lastSensorCheck = 0;
  if (millis() - lastSensorCheck >= 100) {
    lastSensorCheck = millis();
    checkUltrasonic();
  }
}
