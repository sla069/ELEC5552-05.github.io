// ESP32 -> 4x ESCs (v3.x LEDC API).
// '1' = ALL FULL (pre-arm), '0' = ALL OFF.
// '=' / '-' adjust maxUs. 'C' = MAX->MIN calibration. 'I' = all idle (armed).

#include <Arduino.h>

#define ESC1_PIN 25
#define ESC2_PIN 26
#define ESC3_PIN 27
#define ESC4_PIN 14

// v3 API: use ledcAttach(pin, freq, res)
const uint32_t PWM_FREQ_REQ = 500;   // requested Hz
const uint8_t  PWM_RES      = 16;   // bits

// Pins (order matters for index lookups)
const uint8_t ESC_PINS[4] = {ESC1_PIN, ESC2_PIN, ESC3_PIN, ESC4_PIN};

// Each pin may end up on a different LEDC timer.
// Store actual per-pin frequency (Hz) and period (µs).
uint32_t pinFreq[4]   = {0,0,0,0};
uint32_t pinPeriodUs[4] = {20000,20000,20000,20000};

// --- Throttle limits ---
uint16_t maxUs = 2200;
const uint16_t MAX_ALLOW_US = 2200;

// Use 1000 µs as "idle/armed" (typical ESC minimum)
uint16_t IDLE_US   = 1000;
uint16_t PREARM_MS = 900;

// --- State ---
bool allSpinning = false, armedIdle = false;

// Helpers to map pin number to index 0..3
int pinToIndex(uint8_t pin){
  for (int i=0;i<4;i++) if (ESC_PINS[i]==pin) return i;
  return -1;
}

// Convert microseconds to duty for a specific pin (uses that pin's actual period)
uint32_t usToDuty(uint16_t us, int idx){
  uint32_t period = pinPeriodUs[idx];
  if (us > period) us = period;
  const uint32_t max_duty = (1UL << PWM_RES) - 1;
  return (uint64_t)us * max_duty / period;
}

void refreshPinTiming(int idx){
  // v3 API provides ledcReadFreq(pin). If your core lacks it, it returns 0 — we fall back to requested.
  uint32_t f = ledcReadFreq(ESC_PINS[idx]);
  if (f == 0) f = PWM_FREQ_REQ;
  pinFreq[idx]    = f;
  pinPeriodUs[idx]= (f > 0) ? (1000000UL / f) : 20000UL;
}

void escSignalOff(uint8_t pin){
  // Detach PWM and drive low
  ledcDetach(pin);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void escSignalOn(uint8_t pin, uint16_t us){
  // Ensure attached (idempotent; v3 will reuse existing timer/channel)
  ledcAttach(pin, PWM_FREQ_REQ, PWM_RES);
  int idx = pinToIndex(pin);
  if (idx >= 0) refreshPinTiming(idx);
  // Now write pulse in microseconds using this pin's true period
  if (idx >= 0) ledcWrite(pin, usToDuty(us, idx));
}

void allOff(){
  for (auto p: ESC_PINS) escSignalOff(p);
  allSpinning = false; armedIdle = false;
  Serial.println("ALL OFF (no signal).");
}

void allIdle(){
  for (int i=0;i<4;i++){
    escSignalOn(ESC_PINS[i], IDLE_US);
  }
  allSpinning = false; armedIdle = true;
  Serial.print("ALL IDLE (armed) @ "); Serial.print(IDLE_US); Serial.println(" us.");
}

void allPrearmThenFull(){
  if (!armedIdle){
    for (int i=0;i<4;i++){
      escSignalOn(ESC_PINS[i], IDLE_US);
    }
    delay(PREARM_MS);
  }
  // Write FULL using each pin's own period
  for (int i=0;i<4;i++){
    ledcWrite(ESC_PINS[i], usToDuty(maxUs, i));
  }
  allSpinning = true; armedIdle = false;
  Serial.print("ALL FULL @ "); Serial.print(maxUs); Serial.println(" us.");
}

void updateFullIfSpinning(){
  if (!allSpinning) return;
  for (int i=0;i<4;i++){
    ledcWrite(ESC_PINS[i], usToDuty(maxUs, i));
  }
}

void calibrateAll(){
  Serial.println("CAL: PROPS OFF! 2s MAX -> 2s MIN -> OFF.");
  // Send MAX
  for (int i=0;i<4;i++){
    escSignalOn(ESC_PINS[i], maxUs);
  }
  delay(2000);
  // Then MIN (1000 µs typical)
  const uint16_t MIN_US = 1000;
  for (int i=0;i<4;i++){
    ledcWrite(ESC_PINS[i], usToDuty(MIN_US, i));
  }
  delay(2000);
  allOff();
  Serial.println("CAL: Done.");
}

void setup(){
  Serial.begin(115200);
  delay(200);

  // Attach each pin once so we can read its actual frequency,
  // then immediately go to OFF (keeps behavior similar to your original).
  for (int i=0;i<4;i++){
    ledcAttach(ESC_PINS[i], PWM_FREQ_REQ, PWM_RES);
    refreshPinTiming(i);
    // Start safe idle for a brief moment
    ledcWrite(ESC_PINS[i], usToDuty(IDLE_US, i));
  }
  // Now OFF (no signal)
  for (auto p: ESC_PINS) escSignalOff(p);

  Serial.println();
  Serial.println("ESC test (v3 API, per-pin timing): 1=ALL FULL, 0=OFF, I=IDLE, C=Calibrate, =/- adjust maxUs.");
  Serial.print("Requested freq: "); Serial.print(PWM_FREQ_REQ); Serial.println(" Hz");
  for (int i=0;i<4;i++){
    Serial.printf("  Pin %u -> real %.3f Hz (period %lu us)\n",
                  ESC_PINS[i], (double)pinFreq[i], (unsigned long)pinPeriodUs[i]);
  }
}

void loop(){
  if (Serial.available()){
    int b = toupper(Serial.read());
    if      (b=='1') allPrearmThenFull();
    else if (b=='0') allOff();
    else if (b=='I') allIdle();
    else if (b=='='){ if (maxUs+10<=MAX_ALLOW_US) maxUs+=10; Serial.printf("maxUs=%u\n",maxUs); updateFullIfSpinning(); }
    else if (b=='-'){ if (maxUs>=1910) maxUs-=10;             Serial.printf("maxUs=%u\n",maxUs); updateFullIfSpinning(); }
    else if (b=='C') calibrateAll();
  }
  delay(5);
}
