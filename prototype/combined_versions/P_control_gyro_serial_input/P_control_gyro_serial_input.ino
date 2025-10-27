/*
  ESP32 + MPU6050  — Yaw Rate Control with Serial Input
  -----------------------------------------------------
  • Type a number in the Serial Monitor and press Enter:
      0       -> hold heading (no spin)
      1.0     -> spin right  ~57°/s   (1 rad/s)
     -2.0     -> spin left  ~115°/s  (-2 rad/s)
  • Optional commands:
      k 120   -> set kP_yaw to 120 (µs per rad/s)
      base 1500 -> set baseThrottle to 1500 µs
  Notes:
  - Positive targetYawRate = rotate RIGHT (CW) if your gyro sign matches this convention.
  - Start with small magnitudes (0.5–1.0 rad/s) indoors.
  - PROPS OFF for bench tests.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------- ESC pins (X frame: Front at top) ----------
#define ESC1_PIN 25   // Front-Left  (M1)
#define ESC2_PIN 26   // Front-Right (M2)
#define ESC3_PIN 27   // Rear-Right  (M3)
#define ESC4_PIN 14   // Rear-Left   (M4)

// ---------- LEDC PWM ----------
const uint32_t PWM_FREQ = 500;   // Hz
const uint8_t  PWM_RES  = 16;    // bits

// ---------- Throttle & limits (adjust to your ESCs) ----------
uint16_t baseThrottle = 1500;    // hover throttle (tune for your craft)
const uint16_t MIN_US = 1000;
const uint16_t MAX_US = 2000;

// ---------- Motor spin directions (VERIFY) ----------
// Typical X config (check on your airframe):
// M1 Front-Left  = CCW
// M2 Front-Right = CW
// M3 Rear-Right  = CCW
// M4 Rear-Left   = CW
const bool M1_CCW = true;
const bool M2_CCW = false;
const bool M3_CCW = true;
const bool M4_CCW = false;

// ---------- Gyro ----------
Adafruit_MPU6050 mpu;

// ---------- Control ----------
float kP_yaw = 120.0f;     // µs per (rad/s). Start ~60–120 and tune.
float targetYawRate = 0.0f; // rad/s (+ right, - left)

// ---------- Serial input buffer ----------
String line;

// Utility: microseconds -> LEDC duty (ESP32)
uint32_t usToDuty(uint16_t us) {
  uint32_t period = 1000000UL / PWM_FREQ;   // ≈2000 µs at 500 Hz
  if (us < MIN_US) us = MIN_US;
  if (us > MAX_US) us = MAX_US;
  const uint32_t max_duty = (1UL << PWM_RES) - 1;
  return (uint64_t)us * max_duty / period;
}

// Write one ESC by pin (uses ledcWrite with pin-as-channel mapping)
void writeESC(uint8_t pin, uint16_t us) {
  ledcWrite(pin, usToDuty(us));
}

// Pure yaw mix around baseThrottle.
// yawMixUs > 0 => increase CCW motors, decrease CW motors.
void mixYawAndWrite(int16_t yawMixUs) {
  int16_t m1 = baseThrottle + (M1_CCW ? +yawMixUs : -yawMixUs);
  int16_t m2 = baseThrottle + (M2_CCW ? +yawMixUs : -yawMixUs);
  int16_t m3 = baseThrottle + (M3_CCW ? +yawMixUs : -yawMixUs);
  int16_t m4 = baseThrottle + (M4_CCW ? +yawMixUs : -yawMixUs);

  writeESC(ESC1_PIN, m1);
  writeESC(ESC2_PIN, m2);
  writeESC(ESC3_PIN, m3);
  writeESC(ESC4_PIN, m4);
}

// Parse a full line from Serial:
// - If line starts with 'k ' -> set kP_yaw
// - If line starts with 'base ' -> set baseThrottle
// - Else try to parse as a float yaw rate in rad/s
void handleLine(const String& s) {
  String str = s; str.trim();
  if (str.length() == 0) return;

  // Optional commands:
  if (str.startsWith("k ")) {
    float v = str.substring(2).toFloat();
    if (v > 0.0f) {
      kP_yaw = v;
      Serial.print(F("[OK] kP_yaw = ")); Serial.println(kP_yaw, 1);
    } else {
      Serial.println(F("[ERR] usage: k <value>   e.g., k 120"));
    }
    return;
  }

  if (str.startsWith("base ")) {
    float v = str.substring(5).toFloat();
    if (v >= MIN_US && v <= MAX_US) {
      baseThrottle = (uint16_t)v;
      Serial.print(F("[OK] baseThrottle = ")); Serial.println(baseThrottle);
    } else {
      Serial.print(F("[ERR] base out of range (")); Serial.print(MIN_US);
      Serial.print(F("..")); Serial.print(MAX_US); Serial.println(F(")"));
    }
    return;
  }

  // Default: treat the entire line as a yaw-rate setpoint (rad/s)
  float r = str.toFloat();  // accepts leading +/- and decimals
  targetYawRate = r;
  Serial.print(F("[OK] targetYawRate = ")); Serial.print(targetYawRate, 3);
  Serial.println(F(" rad/s"));
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Test print");

  // --- ESCs (attach pins as LEDC "channels" on ESP32 Arduino core) ---
  uint8_t pins[4] = {ESC1_PIN, ESC2_PIN, ESC3_PIN, ESC4_PIN};
  for (uint8_t i = 0; i < 4; i++) {
    ledcAttach(pins[i], PWM_FREQ, PWM_RES);
  }

  // Arm: send idle for a moment (PROPS OFF on bench!)
  for (uint8_t i = 0; i < 4; i++) writeESC(pins[i], MIN_US);
  delay(2000);

  // --- MPU6050 ---
  Wire.begin(21, 22); // SDA=21, SCL=22 on ESP32 (adjust if needed)
  if (!mpu.begin(0x68, &Wire)) {
    if (!mpu.begin(0x69, &Wire)) {
      Serial.println(F("[FATAL] MPU6050 not found at 0x68/0x69. Check wiring!"));
      while (1) delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  Serial.println(F("\nYaw-rate control ready."));
  Serial.println(F("Type a number (rad/s) and press Enter:"));
  Serial.println(F("  0      -> hold heading"));
  Serial.println(F("  1.0    -> slow right spin (~57°/s)"));
  Serial.println(F(" -1.0    -> slow left spin (~57°/s)"));
  Serial.println(F("Commands: 'k 120' (set kP_yaw), 'base 1500'"));
}

void loop() {
  // --- Read complete lines from Serial (non-blocking) ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { handleLine(line); line = ""; }
    else { line += c; if (line.length() > 64) line = ""; }
  }

  // --- Read gyro Z (rad/s) ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float yawRate = g.gyro.z; // + means rotating right (typical MPU6050 orientation)

  // --- Simple P controller on yaw rate ---
  float err = targetYawRate - yawRate;          // rad/s
  float yawMixF = kP_yaw * err;                 // µs

  // Constrain the mixing so we don't starve motors
  const int16_t YAW_MIX_LIMIT = 200;            // µs authority on top of baseThrottle
  if (yawMixF >  YAW_MIX_LIMIT) yawMixF =  YAW_MIX_LIMIT;
  if (yawMixF < -YAW_MIX_LIMIT) yawMixF = -YAW_MIX_LIMIT;

  mixYawAndWrite((int16_t)yawMixF);

  // --- Telemetry (every ~250 ms) ---
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 250) {
    lastPrint = millis();
    Serial.print("YawRate(rad/s): ");  Serial.print(yawRate, 3);
    Serial.print("  target: ");        Serial.print(targetYawRate, 3);
    Serial.print("  mix(us): ");       Serial.print((int)yawMixF);
    Serial.print("  base(us): ");      Serial.print(baseThrottle);
    Serial.println();
  }

  delay(10); // ~100 Hz loop
}

