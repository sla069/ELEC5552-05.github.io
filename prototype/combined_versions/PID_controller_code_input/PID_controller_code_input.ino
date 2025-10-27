/*
  ESP32 + MPU6050 — Yaw Rate Control with PID (no Serial input)
  -------------------------------------------------------------
  You can directly set control parameters below:
    • targetYawRate   -> rad/s (positive = spin right, negative = spin left)
    • kP_yaw, kI_yaw, kD_yaw -> PID gains
    • baseThrottle    -> base ESC throttle (1000–2000 µs)
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------- ESC pins ----------
#define ESC1_PIN 25   // Front-Left  (M1)
#define ESC2_PIN 26   // Front-Right (M2)
#define ESC3_PIN 27   // Rear-Right  (M3)
#define ESC4_PIN 14   // Rear-Left   (M4)

// ---------- PWM setup ----------
const uint32_t PWM_FREQ = 500;   // Hz
const uint8_t  PWM_RES  = 16;    // bits

// ---------- Throttle ----------
uint16_t baseThrottle = 1500;    // hover throttle
const uint16_t MIN_US = 1000;
const uint16_t MAX_US = 2000;

// ---------- Motor spin directions ----------
const bool M1_CCW = true;
const bool M2_CCW = false;
const bool M3_CCW = true;
const bool M4_CCW = false;

// ---------- Gyro ----------
Adafruit_MPU6050 mpu;

// ---------- PID parameters ----------
float kP_yaw = 120.0f;  // proportional gain
float kI_yaw = 20.0f;   // integral gain
float kD_yaw = 5.0f;    // derivative gain

// ---------- Control targets ----------
float targetYawRate = 0.0f;   // rad/s (e.g. 1.0 = spin right, -1.0 = spin left)

// ---------- PID state ----------
float yawIntegral = 0.0f;
float lastError = 0.0f;

// Utility: microseconds -> LEDC duty (ESP32)
uint32_t usToDuty(uint16_t us) {
  uint32_t period = 1000000UL / PWM_FREQ;
  if (us < MIN_US) us = MIN_US;
  if (us > MAX_US) us = MAX_US;
  const uint32_t max_duty = (1UL << PWM_RES) - 1;
  return (uint64_t)us * max_duty / period;
}

// Write one ESC by pin
void writeESC(uint8_t pin, uint16_t us) {
  ledcWrite(pin, usToDuty(us));
}

// Mix yaw correction into motors
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

void setup() {
  Serial.begin(115200);
  delay(50);

  Serial.println(F("=== Yaw Rate PID Control (Fixed Settings) ==="));
  Serial.print(F("Target yaw rate: ")); Serial.print(targetYawRate); Serial.println(F(" rad/s"));
  Serial.print(F("Base throttle: ")); Serial.println(baseThrottle);
  Serial.print(F("PID gains: ")); Serial.print(kP_yaw); Serial.print(", ");
  Serial.print(kI_yaw); Serial.print(", "); Serial.println(kD_yaw);

  // ESC setup
  uint8_t pins[4] = {ESC1_PIN, ESC2_PIN, ESC3_PIN, ESC4_PIN};
  for (uint8_t i = 0; i < 4; i++) ledcAttach(pins[i], PWM_FREQ, PWM_RES);

  // Arm ESCs
  for (uint8_t i = 0; i < 4; i++) writeESC(pins[i], MIN_US);
  delay(2000);

  // MPU6050 setup
  Wire.begin(21, 22); // SDA=21, SCL=22
  if (!mpu.begin(0x68, &Wire)) {
    if (!mpu.begin(0x69, &Wire)) {
      Serial.println(F("[FATAL] MPU6050 not found. Check wiring!"));
      while (1) delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
}

void loop() {
  // --- Read gyro yaw rate ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float yawRate = g.gyro.z; // rad/s

  // --- PID controller ---
  float err = targetYawRate - yawRate;

  // Proportional
  float Pterm = kP_yaw * err;

  // Integral
  yawIntegral += err * 0.01f;
  yawIntegral = constrain(yawIntegral, -50.0f, 50.0f);
  float Iterm = kI_yaw * yawIntegral;

  // Derivative
  float derivative = (err - lastError) / 0.01f;
  float Dterm = kD_yaw * derivative;
  lastError = err;

  // Total PID output
  float yawMixF = Pterm + Iterm + Dterm;
  yawMixF = constrain(yawMixF, -200, 200);

  // Apply to motors
  mixYawAndWrite((int16_t)yawMixF);

  // Telemetry
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 250) {
    lastPrint = millis();
    Serial.print("YawRate(rad/s): "); Serial.print(yawRate, 3);
    Serial.print("  target: "); Serial.print(targetYawRate, 3);
    Serial.print("  P: "); Serial.print(Pterm, 1);
    Serial.print("  I: "); Serial.print(Iterm, 1);
    Serial.print("  D: "); Serial.print(Dterm, 1);
    Serial.print("  mix(us): "); Serial.print((int)yawMixF);
    Serial.print("  base(us): "); Serial.println(baseThrottle);
  }

  delay(10); // ~100 Hz loop
}
