#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

// Example for ESP32: SDA = 21, SCL = 22
#define SDA_PIN 21
#define SCL_PIN 22

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);  // explicitly set SDA/SCL pins

  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize VL53L0X!");
    while (1) {}
  }

  sensor.setMeasurementTimingBudget(20000); // 20ms per reading
  sensor.startContinuous();
}

void loop() {
  uint16_t distance = sensor.readRangeContinuousMillimeters();

  Serial.print("Distance: ");
  Serial.print(distance / 10.0);
  Serial.println(" cm");

  if (sensor.timeoutOccurred()) {
    Serial.println("Sensor timeout!");
  }

  delay(100);
}
