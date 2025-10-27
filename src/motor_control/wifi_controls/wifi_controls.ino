#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <WebServer.h>

// --- ESC pins (quad-X layout) ---
#define ESC1_PIN 25 //Front left
#define ESC2_PIN 26 //Front right
#define ESC3_PIN 27 //Back right
#define ESC4_PIN 14 //Back left

// --- ToF pins (ESP32) ---
#define SDA_PIN 21
#define SCL_PIN 22

// --- Constants ---
const uint32_t PWM_FREQ = 500;
const uint8_t  PWM_RES  = 16;

// Hover throttle and motion offsets (microseconds)
uint16_t baseThrottle = 1500;

// --- ToF Sensor ---
VL53L0X sensor;

// --- State Flags ---
bool motorsArmed = false;
bool obstacleDetected = false;
bool emergencyActive = false;
unsigned long obstacleStartTime = 0;

// --- Connection Watchdog ---
unsigned long lastClientTime = 0;
unsigned long armTime = 0;
bool clientConnected = false;
const unsigned long CONNECTION_TIMEOUT_MS = 2000;  // 2 seconds
const unsigned long ARM_GRACE_PERIOD_MS   = 3000;  // 3 seconds after arming

// --- Wi-Fi Access Point ---
const char* ap_ssid = "ESP32_DRONE";
const char* ap_password = "drone1234";
WebServer server(80);

// --- SPEED CONTROL ---
enum SpeedMode { SPEED_LOW = 0, SPEED_MEDIUM = 1, SPEED_HIGH = 2 };
SpeedMode currentSpeed = SPEED_MEDIUM; // default
uint16_t pitchOffset[] = {200, 350, 500};
uint16_t rollOffset[]  = {200, 350, 500};
uint16_t yawOffset[]   = {200, 350, 500};

// --- UI Mode Label ---
String currentMode = "Idle"; // simple label for web UI

// -------------------------------------------------------------------
uint32_t usToDuty(uint16_t us) {
  uint32_t period = 1000000UL / PWM_FREQ;
  if (us > period) us = period;
  const uint32_t max_duty = (1UL << PWM_RES) - 1;
  return (uint64_t)us * max_duty / period;
}

void writeESC(uint8_t pin, uint16_t us) {
  ledcWrite(pin, usToDuty(constrain(us, 1000, 2000)));
}

void setMotors(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
  writeESC(ESC1_PIN, m1);
  writeESC(ESC2_PIN, m2);
  writeESC(ESC3_PIN, m3);
  writeESC(ESC4_PIN, m4);
}

void hover() {
  setMotors(baseThrottle, baseThrottle, baseThrottle, baseThrottle);
  currentMode = "Hover";
}

void stopMotors() {
  if (emergencyActive) return;
  emergencyActive = true;
  motorsArmed = false;

  Serial.println(" Emergency landing initiated...");

  uint16_t throttle = baseThrottle;
  unsigned long lastUpdate = millis();

  while (throttle > 800) {
    setMotors(throttle, throttle, throttle, throttle);
    server.handleClient();
    if (millis() - lastUpdate >= 500) {
      Serial.printf("Throttle: %d\n", throttle);
      lastUpdate = millis();
    }
    throttle -= 10;
    delay(200);
  }

  setMotors(800, 800, 800, 800);
  delay(1500);

  ledcDetach(ESC1_PIN);
  ledcDetach(ESC2_PIN);
  ledcDetach(ESC3_PIN);
  ledcDetach(ESC4_PIN);

  Serial.println("Motors detached — full stop.");
  currentMode = "Emergency STOP";
}

void armMotors() {
  if (motorsArmed) return;

  Serial.println("Arming motors... ramping up throttle.");
  uint8_t pins[4] = {ESC1_PIN, ESC2_PIN, ESC3_PIN, ESC4_PIN};
  for (uint8_t i = 0; i < 4; i++) {
    ledcAttach(pins[i], PWM_FREQ, PWM_RES);
  }

  uint16_t throttle = 900;
  while (throttle < baseThrottle) {
    setMotors(throttle, throttle, throttle, throttle);
    Serial.printf("Throttle: %d\n", throttle);
    throttle += 10;
    delay(200);
  }

  hover();
  motorsArmed = true;
  emergencyActive = false;
  armTime = millis(); // record time of arming
  Serial.println("Motors armed and hovering.");
  currentMode = "Armed - Hover";
}

// --- Motion Functions ---
void moveForward() {
  uint16_t dp = pitchOffset[currentSpeed];
  setMotors(baseThrottle - dp, baseThrottle - dp, baseThrottle + dp, baseThrottle + dp);
  currentMode = "Moving Forward";
}
void moveBackward() {
  uint16_t dp = pitchOffset[currentSpeed];
  setMotors(baseThrottle + dp, baseThrottle + dp, baseThrottle - dp, baseThrottle - dp);
  currentMode = "Moving Backward";
}
void moveLeft() {
  uint16_t dr = rollOffset[currentSpeed];
  setMotors(baseThrottle - dr, baseThrottle + dr, baseThrottle + dr, baseThrottle - dr);
  currentMode = "Moving Left";
}
void moveRight() {
  uint16_t dr = rollOffset[currentSpeed];
  setMotors(baseThrottle + dr, baseThrottle - dr, baseThrottle - dr, baseThrottle + dr);
  currentMode = "Moving Right";
}
void rotateLeft() {
  uint16_t dy = yawOffset[currentSpeed];
  setMotors(baseThrottle - dy, baseThrottle + dy, baseThrottle - dy, baseThrottle + dy);
  currentMode = "Rotating Left";
}
void rotateRight() {
  uint16_t dy = yawOffset[currentSpeed];
  setMotors(baseThrottle + dy, baseThrottle - dy, baseThrottle + dy, baseThrottle - dy);
  currentMode = "Rotating Right";
}

// --- Pentagon path ---
void pentagonPath() {
  Serial.println("Starting pentagon path...");
  const int sides = 5;
  const int forwardTimeMs = 3000;
  const int rotateTimeMs = 1000;

  for (int i = 0; i < sides; i++) {
    if (emergencyActive || obstacleDetected) break;
    Serial.printf("Pentagon side %d: moving forward\n", i + 1);
    moveForward();
    delay(forwardTimeMs);

    Serial.printf("Pentagon side %d: rotating 72 deg\n", i + 1);
    rotateRight();
    delay(rotateTimeMs);

    hover();
    delay(500);
  }

  Serial.println("Pentagon path complete.");
  hover();
}

bool motionAllowed() {
  return (!emergencyActive && motorsArmed && !obstacleDetected);
}

// --- Connection heartbeat refresh ---
void refreshClientConnection() {
  lastClientTime = millis();
  clientConnected = true;
}

// --- Web Handlers ---
void handleArm() { refreshClientConnection(); server.send(200, "text/plain", "Arming motors..."); armMotors(); }
void handleHover() { refreshClientConnection(); if (!motionAllowed()) { server.send(200, "text/plain", "Cannot hover."); return; } hover(); server.send(200, "text/plain", "Hovering."); }
void handleForward() { refreshClientConnection(); if (!motionAllowed()) { server.send(200, "text/plain", "Blocked"); return; } moveForward(); server.send(200, "text/plain", "Forward"); }
void handleBackward() { refreshClientConnection(); if (!motionAllowed()) { server.send(200, "text/plain", "Blocked"); return; } moveBackward(); server.send(200, "text/plain", "Backward"); }
void handleLeft() { refreshClientConnection(); if (!motionAllowed()) { server.send(200, "text/plain", "Blocked"); return; } moveLeft(); server.send(200, "text/plain", "Left"); }
void handleRight() { refreshClientConnection(); if (!motionAllowed()) { server.send(200, "text/plain", "Blocked"); return; } moveRight(); server.send(200, "text/plain", "Right"); }
void handleRotLeft() { refreshClientConnection(); if (!motionAllowed()) { server.send(200, "text/plain", "Blocked"); return; } rotateLeft(); server.send(200, "text/plain", "Rotate Left"); }
void handleRotRight() { refreshClientConnection(); if (!motionAllowed()) { server.send(200, "text/plain", "Blocked"); return; } rotateRight(); server.send(200, "text/plain", "Rotate Right"); }
void handlePentagon() { refreshClientConnection(); if (!motionAllowed()) { server.send(200, "text/plain", "Blocked"); return; } server.send(200, "text/plain", "Pentagon started"); pentagonPath(); }
void handleSpeed() {
  refreshClientConnection();
  if (!server.hasArg("mode")) { server.send(400, "text/plain", "Missing mode"); return; }
  String m = server.arg("mode");
  if (m == "0") currentSpeed = SPEED_LOW;
  else if (m == "1") currentSpeed = SPEED_MEDIUM;
  else if (m == "2") currentSpeed = SPEED_HIGH;
  else { server.send(400, "text/plain", "Bad mode"); return; }
  server.send(200, "text/plain", "Speed set");
}
void handleKill() { refreshClientConnection(); server.send(200, "text/plain", "KILL activated"); stopMotors(); }

void handleStatus() {
  refreshClientConnection();
  uint16_t distance = sensor.readRangeContinuousMillimeters();
  float distance_cm = (distance / 10.0)-2.0;
  if (distance_cm < 0) distance_cm = 0;

  String json = "{";
  json += "\"motorsArmed\":"; json += (motorsArmed ? "true" : "false"); json += ",";
  json += "\"emergencyActive\":"; json += (emergencyActive ? "true" : "false"); json += ",";
  json += "\"obstacleDetected\":"; json += (obstacleDetected ? "true" : "false"); json += ",";
  json += "\"currentSpeed\":"; json += (int)currentSpeed; json += ",";
  json += "\"mode\":\""; json += currentMode; json += "\",";
  json += "\"distance_cm\":"; json += String(distance_cm, 1);
  json += "}";
  server.send(200, "application/json", json);
}

void handleRoot() {
  refreshClientConnection();
  String page =
    "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'/>"
    "<title>ESP32 Drone Control</title></head><body>"
    "<h2>ESP32 Drone Control (AP Mode)</h2>"
    "<div id='status'>Loading...</div>"
    "<div style='margin-top:10px;'>"
    "<button onclick=\"cmd('/arm')\">Arm</button> "
    "<button onclick=\"cmd('/hover')\">Hover</button> "
    "<button onclick=\"cmd('/forward')\">Forward</button> "
    "<button onclick=\"cmd('/back')\">Backward</button> "
    "<button onclick=\"cmd('/left')\">Left</button> "
    "<button onclick=\"cmd('/right')\">Right</button> "
    "<button onclick=\"cmd('/rotleft')\">Rotate Left</button> "
    "<button onclick=\"cmd('/rotright')\">Rotate Right</button> "
    "<button onclick=\"cmd('/pentagon')\">Pentagon</button>"
    "</div>"
    "<div style='margin-top:12px;'>"
    "Speed: <button onclick=\"cmd('/speed?mode=0')\">Low</button> "
    "<button onclick=\"cmd('/speed?mode=1')\">Medium</button> "
    "<button onclick=\"cmd('/speed?mode=2')\">High</button>"
    "</div>"
    "<div style='margin-top:12px;'>"
    "<button style='background:red;color:white;padding:10px' onclick=\"cmd('/kill')\">KILL</button>"
    "</div>"
    "<script>"
    "function cmd(p){var x=new XMLHttpRequest();x.open('GET',p,true);x.send();}"
    "function update(){var r=new XMLHttpRequest();r.open('GET','/status',true);"
    "r.onreadystatechange=function(){if(r.readyState==4){if(r.status==200){"
    "var s=JSON.parse(r.responseText);"
    "var html='Mode: '+s.mode+'<br>Speed: '+(s.currentSpeed==0?'LOW':(s.currentSpeed==1?'MEDIUM':'HIGH'))+"
    "'<br>Armed: '+s.motorsArmed+'<br>Emergency: '+s.emergencyActive+'<br>Obstacle: '+s.obstacleDetected+"
    "'<br>Distance (cm): '+s.distance_cm;document.getElementById('status').innerHTML=html;}"
    "setTimeout(update,500);}};r.send();}"
    "update();</script></body></html>";
  server.send(200, "text/html", page);
}

// -------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  uint8_t pins[4] = {ESC1_PIN, ESC2_PIN, ESC3_PIN, ESC4_PIN};
  for (uint8_t i = 0; i < 4; i++) {
    ledcAttach(pins[i], PWM_FREQ, PWM_RES);
  }

  Serial.println("Motors are OFF. Web controls available on AP only.");

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize VL53L0X!");
    while (1) {}
  }

  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous();

  // --- Wi-Fi Access Point setup ---
  Serial.println("\nStarting Wi-Fi Access Point...");
  WiFi.softAP(ap_ssid, ap_password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print(" Access Point started. Connect to SSID: ");
  Serial.println(ap_ssid);
  Serial.print("Password: ");
  Serial.println(ap_password);
  Serial.print("Then open in browser: http://");
  Serial.println(IP);

  // --- Web Server routes ---
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/arm", handleArm);
  server.on("/hover", handleHover);
  server.on("/forward", handleForward);
  server.on("/back", handleBackward);
  server.on("/left", handleLeft);
  server.on("/right", handleRight);
  server.on("/rotleft", handleRotLeft);
  server.on("/rotright", handleRotRight);
  server.on("/pentagon", handlePentagon);
  server.on("/speed", handleSpeed);
  server.on("/kill", handleKill);

  server.begin();
  Serial.println("Web server started.");
  Serial.println("Ready. Controls are on the Wi-Fi page only (AP mode).");

  lastClientTime = millis();  // initialize watchdog timer
}

// -------------------------------------------------------------------
void loop() {
  server.handleClient();

  uint16_t distance = sensor.readRangeContinuousMillimeters();
  float distance_cm = (distance / 10.0) - 2.0;
  if (distance_cm < 0) distance_cm = 0;
  Serial.print("Distance: "); Serial.print(distance_cm); Serial.println(" cm");

  // --- Obstacle detection ---
  if (!emergencyActive && motorsArmed) {
    if (distance_cm <= 10.0 && distance_cm > 0) {
      if (!obstacleDetected) {
        obstacleDetected = true;
        obstacleStartTime = millis();
        Serial.println(" Obstacle detected — hovering.");
        hover();
      } else if (millis() - obstacleStartTime >= 10000) {
        Serial.println(" Obstacle persists 10 s — triggering EMERGENCY STOP!");
        stopMotors();
      }
    } else {
      if (obstacleDetected) {
        obstacleDetected = false;
        Serial.println(" Obstacle cleared — control re-enabled.");
      }
    }
  }

  // --- Connection watchdog (fixed logic) ---
  if (motorsArmed && !emergencyActive) {
    unsigned long now = millis();

    // Ignore timeout for a few seconds after arming
    if (now - armTime > ARM_GRACE_PERIOD_MS) {
      if (clientConnected && (now - lastClientTime > CONNECTION_TIMEOUT_MS)) {
        Serial.println(" Web client inactive — initiating EMERGENCY STOP!");
        stopMotors();
      }
    }
  }

  if (sensor.timeoutOccurred()) Serial.println("Sensor timeout!");
  delay(100);
}

