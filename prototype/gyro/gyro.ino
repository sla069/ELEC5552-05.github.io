#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------- MPU6050 object ----------
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("ESP32 + MPU6050 (Gyroscope only)"));

  Wire.begin(21, 22); // SDA=21, SCL=22 on ESP32

  // Try MPU6050 init (0x68 is default I2C address, 0x69 if AD0 pin is tied HIGH)
  if (!mpu.begin(0x68, &Wire)) {
    if (!mpu.begin(0x69, &Wire)) {
      Serial.println(F("MPU6050 not found at 0x68/0x69. Check wiring!"));
      while (1) delay(10);
    }
  }

  // Configure gyro settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // still needs accelerometer init
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // set gyro range
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // reduce noise

  Serial.println(F("MPU6050 ready.\n"));
}

void loop() {
  sensors_event_t a, g, temp;  
  mpu.getEvent(&a, &g, &temp);

  // --- Print only gyro data ---
  Serial.print(F("Gyro (rad/s): "));
  Serial.print("X=");
  Serial.print(g.gyro.x, 3);
  Serial.print("  Y=");
  Serial.print(g.gyro.y, 3);
  Serial.print("  Z=");
  Serial.println(g.gyro.z, 3);

  delay(250); // adjust sampling interval
}