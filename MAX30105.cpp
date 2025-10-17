#Oxygen

// Teensy 4.1 + MAX30102 on Wire1 (SDA1=17, SCL1=16)
// Fast finger detection + auto LED gain + progress counter during first measurement
// SpO2 algorithm is fed at 25 Hz (decimated from 100 Hz) for stability.

#include <Wire.h>
#include <math.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

#define I2C_BUS        Wire1
#define I2C_SPEED      I2C_SPEED_FAST
const int LED_PIN = 13;

#define DEBUG_FINGER 0   // set 1 to print waiting debug lines

MAX30105 sensor;

// ---------- Sampling / decimation ----------
const int FS_SENSOR = 100;      // sensor rate (Hz)
const int DECIM     = 4;        // 100 -> 25 Hz
const int FS_ALGO   = FS_SENSOR / DECIM; // 25 Hz
int decimCnt = 0;

// ---------- Buffers (first value ~3.2 s) ----------
const int BUF_LEN = 80;         // 80 @ 25 Hz ≈ 3.2 s
uint32_t irBuf[BUF_LEN], redBuf[BUF_LEN];

// ---------- Outputs ----------
int32_t spo2 = 0, hrAlgo = 0;
int8_t  validSPO2 = 0, validHR = 0;

// ---------- Beat detector (instant + moving avg) ----------
const byte RATE_SIZE = 4;
byte  rates[RATE_SIZE] = {0};
byte  rateSpot = 0;
long  lastBeat = 0;
float bpm_instant = 0.0f;
int   bpm_avg = 0;

// ---------- Display smoothing ----------
float bpm_display = 0.0f;
const float ALPHA = 0.30f;       // 0..1 (higher = snappier)
const int HR_MIN = 40, HR_MAX = 180;
const int SPO2_MIN = 70, SPO2_MAX = 100;

// ---------- Sensor configuration (tune if needed) ----------
byte ledBrightness = 90;   // 0..255 (~0..50mA). Raise if dim, lower if clipping
byte sampleAverage = 8;    // 1,2,4,8,16,32
byte ledMode       = 2;    // Red+IR (MAX30102)
byte sampleRate    = 100;  // Hz
int  pulseWidth    = 215;  // ns (69/118/215/411)
int  adcRange      = 4096; // nA (2048/4096/8192/16384)

// ---------- Finger detection (dynamic) ----------
struct Stats { float mean; float stddev; uint32_t minv; uint32_t maxv; };

float irBaseline = 0;             // learned with no finger

// thresholds (relaxed & practical defaults)
float IR_MULT           = 1.20f;  // mean must be > baseline * IR_MULT
float STABILITY_PCT_MAX = 0.15f;  // stddev <= 15% of mean
float RIPPLE_MIN_PCT    = 0.002f; // >= 0.2% of mean (AC/DC)
float RIPPLE_MAX_PCT    = 0.15f;  // <= 15% of mean (filter big motion)

// dwell (TIME-BASED, not pass-count)
const int   STATS_WINDOW_100HZ = 30;     // ~0.3 s stats window
const uint16_t DETECT_DWELL_MS = 900;    // need ~0.9 s continuous good quality

// ---------- Auto LED gain (keep IR DC in target band) ----------
const uint32_t IR_TARGET_MIN = 20000;
const uint32_t IR_TARGET_MAX = 80000;
const int LED_MIN = 10;
const int LED_MAX = 200;

// ---------- State machine ----------
enum State { WAIT_FOR_FINGER, PRIMING, RUNNING };
State state = WAIT_FOR_FINGER;

static inline void waitData() {
  while (sensor.available() == false) sensor.check();
}

bool readSample(uint32_t &red, uint32_t &ir) {
  waitData();
  red = sensor.getRed();
  ir  = sensor.getIR();
  sensor.nextSample();
  return true;
}

void beatDetectAt100Hz(uint32_t ir) {
  if (checkForBeat((long)ir)) {
    long now = millis();
    long dt  = now - lastBeat; lastBeat = now;
    if (dt > 0) {
      float bpm = 60.0f / (dt / 1000.0f);
      if (bpm > 20 && bpm < 255) {
        bpm_instant = bpm;
        rates[rateSpot++] = (byte)bpm; rateSpot %= RATE_SIZE;
        int sum = 0; for (byte i = 0; i < RATE_SIZE; i++) sum += rates[i];
        bpm_avg = sum / RATE_SIZE;
      }
    }
  }
}

bool readDecimated(uint32_t &r25, uint32_t &i25) {
  uint32_t r, i;
  readSample(r, i);
  beatDetectAt100Hz(i);
  if (++decimCnt >= DECIM) {
    decimCnt = 0;
    r25 = r; i25 = i;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));   // blink at ~25 Hz
    return true;
  }
  return false;
}

// Quick stats over N samples @100 Hz
Stats quickStats100(int N) {
  uint64_t sum = 0, sumsq = 0;
  uint32_t vmin = UINT32_MAX, vmax = 0;
  for (int k = 0; k < N; k++) {
    waitData();
    uint32_t ir = sensor.getIR();
    sensor.nextSample();
    sum  += ir;
    sumsq += (uint64_t)ir * ir;
    if (ir < vmin) vmin = ir;
    if (ir > vmax) vmax = ir;
  }
  float mean = (float)sum / N;
  float var  = (float)sumsq / N - mean * mean; if (var < 0) var = 0;
  float sd   = sqrtf(var);
  return { mean, sd, vmin, vmax };
}

bool fingerQualityOK(Stats &out) {
  out = quickStats100(STATS_WINDOW_100HZ);   // ~0.3 s
  if (out.mean < irBaseline * IR_MULT) return false;                // strong enough DC
  if (out.stddev > STABILITY_PCT_MAX * out.mean) return false;      // steady contact
  float ripple = (float)(out.maxv - out.minv) / (out.mean > 0 ? out.mean : 1.f);
  if (ripple < RIPPLE_MIN_PCT || ripple > RIPPLE_MAX_PCT) return false; // reasonable AC
  return true;
}

// ---------- Auto-gain helpers ----------
uint32_t measureIRMeanFast(int samples = 24) {
  uint64_t sum = 0;
  for (int i=0; i<samples; i++) {
    while (sensor.available()==false) sensor.check();
    uint32_t ir = sensor.getIR();
    sensor.nextSample();
    sum += ir;
  }
  return (uint32_t)(sum / samples);
}

void reapplyLEDConfig() {
  sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  sensor.setPulseAmplitudeRed(0x0A);
  sensor.setPulseAmplitudeIR(0x0A);
  sensor.setPulseAmplitudeGreen(0x00);
}

void autoAdjustLED() {
  uint32_t meanIR = measureIRMeanFast(24);
  int b = ledBrightness;

  if (meanIR < IR_TARGET_MIN && b < LED_MAX) {
    int step = (IR_TARGET_MIN - (int)meanIR) / 4000; if (step < 4) step = 4;
    b = min(LED_MAX, b + step);
  } else if (meanIR > IR_TARGET_MAX && b > LED_MIN) {
    int step = ((int)meanIR - IR_TARGET_MAX) / 4000; if (step < 4) step = 4;
    b = max(LED_MIN, b - step);
  } else {
    return; // already within band
  }

  ledBrightness = (byte)b;
  reapplyLEDConfig();
  (void)measureIRMeanFast(12); // settle
}

// ---------- Progress priming (with percent + spinner) ----------
void primeDecimatedBuffersWithProgress() {
  const char spinner[4] = {'|','/','-','\\'};
  int filled = 0;
  unsigned long lastPrint = 0;
  int spinIdx = 0;
  Serial.print("Measuring: 0%  ");
  while (filled < BUF_LEN) {
    uint32_t r, i;
    if (readDecimated(r, i)) {
      redBuf[filled] = r;
      irBuf[filled]  = i;
      filled++;

      // Update every 4 samples (~5%) to avoid spamming Serial
      if (filled % 4 == 0 || millis() - lastPrint > 250) {
        int pct = (filled * 100) / BUF_LEN;
        Serial.print("\rMeasuring: ");
        Serial.print(pct);
        Serial.print("%  ");
        Serial.print(spinner[spinIdx]);
        Serial.print("   ");
        spinIdx = (spinIdx + 1) & 3;
        lastPrint = millis();
      }
    }
  }
  Serial.println("\rMeasuring: 100%   done     ");
}

void computeSpO2() {
  maxim_heart_rate_and_oxygen_saturation(
    irBuf, BUF_LEN, redBuf,
    &spo2, &validSPO2, &hrAlgo, &validHR
  );
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  I2C_BUS.begin();
  if (!sensor.begin(I2C_BUS, I2C_SPEED)) {
    Serial.println("MAX30102 not found. Check 3.3V/GND and Wire1 (SDA1=17, SCL1=16).");
    while (1) { delay(10); }
  }

  reapplyLEDConfig();

  Serial.println("Calibrating ambient IR… keep finger OFF sensor.");
  delay(500);
  Stats amb = quickStats100(100);   // ~1 s ambient
  irBaseline = (amb.mean < 1) ? 1 : amb.mean;
  Serial.print("Ambient baseline IR = "); Serial.println((uint32_t)irBaseline);

  Serial.println("Place index finger on the sensor with steady pressure…");
  state = WAIT_FOR_FINGER;
}

void loop() {
  static unsigned long ok_since_ms = 0;

  switch (state) {
    case WAIT_FOR_FINGER: {
      // Manual override: press any key to force start (for testing)
      if (Serial.available()) {
        while (Serial.available()) Serial.read();
        Serial.println("Manual start → Finger detected — hold steady…");
        autoAdjustLED();
        primeDecimatedBuffersWithProgress();
        computeSpO2();
        state = PRIMING; ok_since_ms = 0;
        break;
      }

      Stats s;
      bool ok = fingerQualityOK(s);

#if DEBUG_FINGER
      static uint32_t last = 0;
      if (millis() - last > 500) {
        float ripple = (float)(s.maxv - s.minv) / (s.mean > 0 ? s.mean : 1.f);
        Serial.print("Waiting… mean=");   Serial.print((uint32_t)s.mean);
        Serial.print(" sd%=");            Serial.print(100.0f * (s.stddev / (s.mean > 0 ? s.mean : 1.f)), 1);
        Serial.print(" ripple%=");        Serial.print(100.0f * ripple, 2);
        Serial.print(" baseline=");       Serial.print((uint32_t)irBaseline);
        Serial.print(" mult=");           Serial.print(IR_MULT, 2);
        Serial.println();
        last = millis();
      }
#endif

      if (ok) {
        if (ok_since_ms == 0) ok_since_ms = millis();
        if (millis() - ok_since_ms >= DETECT_DWELL_MS) {
          Serial.println("Finger detected — hold steady…");
          autoAdjustLED();                              // ensure good IR level
          primeDecimatedBuffersWithProgress();          // <-- progress counter here
          computeSpO2();                                // first result computed
          state = PRIMING; ok_since_ms = 0;
        }
      } else {
        ok_since_ms = 0;
        uint32_t r, i; readDecimated(r, i);  // keep FIFO flowing
      }
      break;
    }

    case PRIMING:
      // First compute already done; move to steady RUNNING
      state = RUNNING;
      break;

    case RUNNING: {
      autoAdjustLED();          // Trim brightness gently as finger relaxes

      // Slide window by ~1 s (25 decimated samples)
      const int STEP = 25;
      for (int i = STEP; i < BUF_LEN; i++) {
        redBuf[i - STEP] = redBuf[i];
        irBuf[i - STEP]  = irBuf[i];
      }
      int filled = BUF_LEN - STEP;
      while (filled < BUF_LEN) {
        uint32_t r, i;
        if (readDecimated(r, i)) {
          redBuf[filled] = r; irBuf[filled] = i;
          filled++;
        }
      }

      computeSpO2();

      // Prefer algorithm HR when valid & plausible; else averaged beat detector (or last)
      bool hr_ok   = (validHR == 1)   && (hrAlgo >= HR_MIN)   && (hrAlgo <= HR_MAX);
      bool spo2_ok = (validSPO2 == 1) && (spo2  >= SPO2_MIN)  && (spo2  <= SPO2_MAX);

      int hr_raw = hr_ok ? hrAlgo : (bpm_avg > 0 ? bpm_avg : (int)(bpm_display > 0 ? bpm_display : 0));
      if (bpm_display == 0.0f && hr_raw > 0) bpm_display = hr_raw;      // initialize once non-zero
      bpm_display = ALPHA * hr_raw + (1.0f - ALPHA) * bpm_display;

      Serial.print("BPM=");
      Serial.print((int)(bpm_display + 0.5f));
      Serial.print(" (src="); Serial.print(hr_ok ? "algo" : "avg"); Serial.print("), ");

      Serial.print("SpO2=");
      if (spo2_ok) { Serial.print(spo2); Serial.print(" % (valid)"); }
      else         { Serial.print("— (invalid)"); }
      Serial.println();

      // Reacquire if finger quality drops
      Stats s;
      if (!fingerQualityOK(s)) {
        Serial.println("Finger removed/unstable. Place finger again…");
        state = WAIT_FOR_FINGER;
        ok_since_ms = 0;
      }
      break;
    }
  }
}

