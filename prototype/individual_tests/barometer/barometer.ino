#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

// ---------- Sensor & I2C ----------
Adafruit_BMP280 bmp;         // I2C interface
static const int I2C_SDA = 21;
static const int I2C_SCL = 22;

// ---------- Calibration (manual input) ----------
const float KNOWN_ALTITUDE_M = 33;   // <-- set your true elevation (meters)
float seaLevel_hPa = 1013.25;          // will be recalculated in setup()

// ---------- Moving Average Settings ----------
const uint16_t MA_WINDOW = 20;         // number of samples in the moving average
float pBuf_hPa[MA_WINDOW];
uint16_t pIdx = 0;
uint16_t pCount = 0;
double   pSum_hPa = 0.0;

// Update the rolling average buffer (pressure in hPa)
float updatePressureMA(float p_hPa) {
  if (pCount < MA_WINDOW) {
    // growing phase
    pBuf_hPa[pCount++] = p_hPa;
    pSum_hPa += p_hPa;
  } else {
    // steady-state: subtract the old, add the new
    pSum_hPa -= pBuf_hPa[pIdx];
    pBuf_hPa[pIdx] = p_hPa;
    pSum_hPa += p_hPa;

    pIdx++;
    if (pIdx >= MA_WINDOW) pIdx = 0;
  }
  return (float)(pSum_hPa / pCount);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("\nESP32 + BMP280 (manual altitude calibration + moving average)"));

  Wire.begin(I2C_SDA, I2C_SCL);

  // Try to initialize BMP280 (try 0x77 if your board uses that)
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor at 0x76. Try 0x77 or check wiring!"));
    while (1) delay(10);
  }

  // Sensor configuration (feel free to tweak)
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,     // temperature oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // IIR filter
                  Adafruit_BMP280::STANDBY_MS_63);  // standby

  Serial.println(F("BMP280 initialized. Calibrating sea-level pressure..."));

  // ---- Manual altitude calibration: compute sea-level pressure from your known altitude
  float P_hPa_now = bmp.readPressure() / 100.0F;             // Pa -> hPa
  seaLevel_hPa = bmp.seaLevelForAltitude(KNOWN_ALTITUDE_M, P_hPa_now);

  Serial.print(F("Known altitude: "));
  Serial.print(KNOWN_ALTITUDE_M);
  Serial.println(F(" m"));

  Serial.print(F("Calibrated sea-level pressure: "));
  Serial.print(seaLevel_hPa, 2);
  Serial.println(F(" hPa\n"));
}

void loop() {
  // Read current values
  float T_C   = bmp.readTemperature();
  float P_hPa = bmp.readPressure() / 100.0F;  // Pa -> hPa

  // Update moving average of pressure
  float P_ma_hPa = updatePressureMA(P_hPa);

  // Compute altitude from averaged pressure
  float alt_ma_m = bmp.readAltitude(seaLevel_hPa); // uses internal formula with current P
  // But we want altitude from *averaged* pressure:
  // Adafruit helper uses internal pressure, so use the standard barometric formula here:
  // h = 44330 * (1 - (P / P0)^(0.1903))
  float alt_from_ma_m = 44330.0f * (1.0f - powf(P_ma_hPa / seaLevel_hPa, 0.1903f));

  // --- Output ---
  Serial.print(F("Temp = ")); Serial.print(T_C); Serial.println(F(" °C"));

  Serial.print(F("Pressure instant = ")); Serial.print(P_hPa); Serial.println(F(" hPa"));
  Serial.print(F("Pressure MA(")); Serial.print(MA_WINDOW); Serial.print(F(") = "));
  Serial.print(P_ma_hPa); Serial.println(F(" hPa"));

  Serial.print(F("Altitude (instant) = "));
  Serial.print(alt_ma_m); Serial.println(F(" m"));

  Serial.print(F("Altitude (MA from pressure) = "));
  Serial.print(alt_from_ma_m); Serial.println(F(" m"));

  Serial.println();
  delay(250);  // sample rate; adjust to taste
}
