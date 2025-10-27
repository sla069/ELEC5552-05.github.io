#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ----------- Ultrasonic (HC-SR04) pins -----------
#define TRIG_PIN 4
#define ECHO_PIN 2

// ----------- Barometer (BMP280) -----------
Adafruit_BMP280 bmp;

// ----------- Gyroscope/Accelerometer (MPU6050) -----------
Adafruit_MPU6050 mpu;

// ----------- I2C pins for ESP32 -----------
static const int I2C_SDA = 21;
static const int I2C_SCL = 22;

// ----------- Altitude calibration -----------
const float KNOWN_ALTITUDE_M = 33;   // your ground elevation
float seaLevel_hPa = 1013.25;         // recalibrated in setup()

// ----------- Helper: ultrasonic distance (cm) -----------
float readUltrasonicCM() {
  // Trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure echo pulse duration
  long duration = pulseIn(ECHO_PIN, HIGH, 25000UL); // timeout ~4.3m
  if (duration == 0) return NAN;

  // Convert to distance (cm) using 58.2 constant (≈20 °C air temp)
  return duration / 58.3;
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("\nESP32 with Ultrasonic + BMP280 + MPU6050"));

  // ----------- Ultrasonic pins -----------
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // ----------- I2C bus -----------
  Wire.begin(I2C_SDA, I2C_SCL);

  // ----------- BMP280 init -----------
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find BMP280 sensor (0x76). Try 0x77."));
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,    // temp oversampling
                  Adafruit_BMP280::SAMPLING_X16,   // pressure oversampling
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_63);

  float P_hPa_now = bmp.readPressure() / 100.0F;
  seaLevel_hPa = bmp.seaLevelForAltitude(KNOWN_ALTITUDE_M, P_hPa_now);

  // ----------- MPU6050 init -----------
  if (!mpu.begin(0x68, &Wire)) {
    if (!mpu.begin(0x69, &Wire)) {
      Serial.println(F("MPU6050 not found at 0x68/0x69."));
      while (1) delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println(F("All sensors initialized.\n"));
}

void loop() {
  // ----------- Ultrasonic -----------
  float distanceCM = readUltrasonicCM();

  // ----------- BMP280 (altitude) -----------
  float T_C   = bmp.readTemperature();
  float P_hPa = bmp.readPressure() / 100.0F;
  float altitude_m = bmp.readAltitude(seaLevel_hPa);

  // ----------- MPU6050 (gyro) -----------
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // ----------- Output -----------
  if (isnan(distanceCM)) {
    Serial.println(F("Ultrasonic: No echo / out of range"));
  } else {
    Serial.print(F("Ultrasonic Distance: "));
    Serial.print(distanceCM, 1);
    Serial.println(F(" cm"));
  }

  Serial.print(F("Barometer Altitude: "));
  Serial.print(altitude_m, 2);
  Serial.println(F(" m"));

  Serial.print(F("Gyro (°/s): X="));
  Serial.print(g.gyro.x * 57.3, 2);
  Serial.print("  Y=");
  Serial.print(g.gyro.y * 57.3, 2);
  Serial.print("  Z=");
  Serial.println(g.gyro.z * 57.3, 2);

  Serial.println(); // blank line
  delay(500);
}
