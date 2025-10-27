#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <FS.h>
#include <SPIFFS.h>

// ----------- Wi-Fi credentials -----------
const char* ssid = "Drone_Sensors";
// const char* password = "";

// ----------- Web server -----------
WebServer server(80);

// ----------- Barometer (BMP280) -----------
Adafruit_BMP280 bmp;

// ----------- Gyroscope/Accelerometer (MPU6050) -----------
Adafruit_MPU6050 mpu;

// ----------- I2C pins for ESP32 -----------
static const int I2C_SDA = 5;
static const int I2C_SCL = 6;

// ----------- Altitude calibration -----------
const float KNOWN_ALTITUDE_M = 33;   // your ground elevation
float seaLevel_hPa = 1013.25;        // recalibrated in setup()

// ----------- Global variables for web output -----------
float T_C = 0, P_hPa = 0, altitude_m = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;

// ----------- Web page handler -----------
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='1'/>"
                "<title>Drone Sensor Data</title>"
                "<style>body{font-family:Arial;text-align:center;}h1{color:#0066cc;}"
                "table{margin:auto;border-collapse:collapse;}td,th{padding:8px 16px;border:1px solid #ccc;}"
                "a.button{display:inline-block;margin-top:10px;padding:10px 20px;background:#0066cc;color:#fff;"
                "text-decoration:none;border-radius:6px;}</style>"
                "</head><body><h1>Drone Sensor Data</h1>"
                "<table><tr><th>Parameter</th><th>Value</th></tr>";
  html += "<tr><td>Temperature (degC)</td><td>" + String(T_C, 2) + "</td></tr>";
  html += "<tr><td>Pressure (hPa)</td><td>" + String(P_hPa, 2) + "</td></tr>";
  html += "<tr><td>Altitude (m)</td><td>" + String(altitude_m, 2) + "</td></tr>";
  html += "<tr><td>Gyro X (deg/s)</td><td>" + String(gyroX, 2) + "</td></tr>";
  html += "<tr><td>Gyro Y (deg/s)</td><td>" + String(gyroY, 2) + "</td></tr>";
  html += "<tr><td>Gyro Z (deg/s)</td><td>" + String(gyroZ, 2) + "</td></tr>";
  html += "</table><br><a class='button' href='/data.csv'> Download CSV</a>"
          "<p>Page refreshes every second.</p></body></html>";
  server.send(200, "text/html", html);
}

// ----------- CSV file download handler -----------
void handleDownload() {
  if (!SPIFFS.exists("/data.csv")) {
    server.send(404, "text/plain", "CSV file not found yet.");
    return;
  }
  File f = SPIFFS.open("/data.csv", "r");
  server.streamFile(f, "text/csv");
  f.close();
}

// ----------- Append one line to CSV -----------
void logToCSV() {
  File f = SPIFFS.open("/data.csv", FILE_APPEND);
  if (!f) {
    Serial.println("Failed to open CSV file for appending.");
    return;
  }
  f.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", T_C, P_hPa, altitude_m, gyroX, gyroY, gyroZ);
  f.close();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\nBMP280 + MPU6050"));

  // ----------- I2C bus -----------
  Wire.begin(I2C_SDA, I2C_SCL);

  // ----------- BMP280 init -----------
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find BMP280 sensor (0x76). Try 0x77."));
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
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

  // ----------- SPIFFS init -----------
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed!");
  } else {
    Serial.println("SPIFFS mounted successfully.");
    // Create CSV header if not present
    if (!SPIFFS.exists("/data.csv")) {
      File f = SPIFFS.open("/data.csv", FILE_WRITE);
      f.println("Temp_C,Pressure_hPa,Altitude_m,GyroX_dps,GyroY_dps,GyroZ_dps");
      f.close();
    }
  }

// ----------- Wi-Fi Access Point mode  -----------
Serial.println("Starting Access Point (no password)...");
WiFi.softAP(ssid);  // No password = open network

IPAddress IP = WiFi.softAPIP();
Serial.println("Access Point started!");
Serial.print("SSID: ");
Serial.println(ssid);
Serial.print("Access Point IP address: ");
Serial.println(IP);


  // ----------- Web server setup -----------
  server.on("/", handleRoot);
  server.on("/data.csv", handleDownload);
  server.begin();
  Serial.println("Web server started.");
}

void loop() {
  // ----------- BMP280 (altitude) -----------
  T_C = bmp.readTemperature();
  P_hPa = bmp.readPressure() / 100.0F;
  altitude_m = bmp.readAltitude(seaLevel_hPa);

  // ----------- MPU6050 (gyro) -----------
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gyroX = g.gyro.x * 57.3;
  gyroY = g.gyro.y * 57.3;
  gyroZ = g.gyro.z * 57.3;

  // ----------- Serial output (unchanged) -----------
  Serial.print(F("Barometer Altitude: "));
  Serial.print(altitude_m, 2);
  Serial.println(F(" m"));

  Serial.print(F("Gyro (°/s): X="));
  Serial.print(gyroX, 2);
  Serial.print("  Y=");
  Serial.print(gyroY, 2);
  Serial.print("  Z=");
  Serial.println(gyroZ, 2);

  Serial.println();

  // ----------- Log to CSV -----------
  logToCSV();

  // ----------- Handle web requests -----------
  server.handleClient();

  delay(500);
}
