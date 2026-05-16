#include <Arduino.h>
#include <Wire.h>
#include <FlexIO_t4.h>
#include <FlexSerial.h>
#include <math.h>

#include "MAX30105.h"
#include "spo2_algorithm.h"

// =====================================================
// Bio-Signal Pins (ECG & GSR)
// =====================================================
const int ecgPin = 40; // Pin 40 corresponds to ADC2_A16
const int gsrPin = 20; // Pin 20 maps to A6

// Non-blocking loop timing tracking
unsigned long lastFastSampleTime = 0;
unsigned long lastSlowSampleTime = 0;

const unsigned long fastSampleInterval = 5;    // 200 Hz for continuous crisp waveforms
const unsigned long slowSampleInterval = 2000; // Poll slow modules every 2 seconds

// =====================================================
// BP / Pressure / HR Module using FlexSerial
// =====================================================
FlexSerial moduleSerial(26, 27);
const uint8_t readCommand[6] = {0xFD, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t rx[6];

// =====================================================
// MAX30205 Temperature Sensor
// =====================================================
#define TEMP_I2C Wire1
#define TEMP_ADDR 0x48
#define TEMP_OFFSET 63.5

// =====================================================
// MAX30102 SpO2 Sensor
// =====================================================
#define SPO2_I2C Wire2
MAX30105 particleSensor;
#define MY_BUFFER_SIZE 50

uint32_t irBuffer[MY_BUFFER_SIZE];
uint32_t redBuffer[MY_BUFFER_SIZE];

int32_t spo2;
int8_t spo2Valid;
int32_t max301HeartRate;
int8_t max301HrValid;

// =====================================================
// Helper Functions
// =====================================================
void printHex(const uint8_t *data, int len) {
  for (int i = 0; i < len; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    if (i < len - 1) Serial.print(" ");
  }
}

bool readExactBytes(uint8_t *buffer, int length, unsigned long timeoutMs) {
  unsigned long startTime = millis();
  int index = 0;
  while (index < length && millis() - startTime < timeoutMs) {
    if (moduleSerial.available()) {
      buffer[index++] = moduleSerial.read();
    }
  }
  return index == length;
}

float readTempCorrected() {
  TEMP_I2C.beginTransmission(TEMP_ADDR);
  TEMP_I2C.write(0x00);
  if (TEMP_I2C.endTransmission(false) != 0) return NAN;
  if (TEMP_I2C.requestFrom((int)TEMP_ADDR, 2) != 2) return NAN;

  uint8_t msb = TEMP_I2C.read();
  uint8_t lsb = TEMP_I2C.read();
  int16_t raw = (int16_t)((msb << 8) | lsb);
  return (raw * (1.0f / 256.0f)) + TEMP_OFFSET;
}

// =====================================================
// Diagnostic Routines
// =====================================================
void readPressureModule() {
  while (moduleSerial.available()) moduleSerial.read();
  moduleSerial.write(readCommand, 6);

  bool ok = readExactBytes(rx, 6, 500); // Tightened timeout to minimize freeze frames
  if (!ok || rx[0] != 0xFD) {
    Serial.print("BP Module: Bad response or timeout: ");
    if (ok) printHex(rx, 6); else Serial.print("No full response");
    Serial.println();
  } else {
    Serial.print("BP Raw: "); printHex(rx, 6); Serial.println();
    Serial.printf("High pressure: %d | Low pressure: %d | HR: %d\n", rx[1], rx[2], rx[3]);
  }
}

void readTemperature() {
  float t = readTempCorrected();
  if (isnan(t)) {
    Serial.println("Temperature: Read error");
  } else {
    Serial.printf("Temperature: %.2f °C\n", t);
  }
}

void readMAX30102() {
  // Read samples as fast as possible without strict loop delays
  for (int i = 0; i < MY_BUFFER_SIZE; i++) {
    unsigned long startTime = millis();
    while (!particleSensor.available()) {
      particleSensor.check();
      if (millis() - startTime > 10) break; // Emergency timeout escape hatch
    }
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, MY_BUFFER_SIZE, redBuffer, &spo2, &spo2Valid, &max301HeartRate, &max301HrValid
  );

  Serial.printf("SpO2: ");
  if (spo2Valid) Serial.printf("%d %%", spo2); else Serial.printf("Invalid");
  Serial.printf(" | MAX30102 BPM: ");
  if (max301HrValid) Serial.printf("%d\n", max301HeartRate); else Serial.printf("Invalid\n");
}

// =====================================================
// Main Setup Execution
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1500);

  moduleSerial.begin(115200);
  TEMP_I2C.begin();
  TEMP_I2C.setClock(100000);
  SPO2_I2C.begin();

  if (!particleSensor.begin(SPO2_I2C, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 initialization failed!");
    while (1);
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);
  particleSensor.setPulseAmplitudeGreen(0x00);

  Serial.println("CardiaLink Core Engine Online.");
}

// =====================================================
// Non-Blocking Concurrent Main Loop
// =====================================================
void loop() {
  unsigned long currentTime = millis();

  // TASK 1: High-Speed Waves (ECG + GSR Waveform Plotter Core)
  if (currentTime - lastFastSampleTime >= fastSampleInterval) {
    lastFastSampleTime = currentTime;

    // Fetch ECG using 12-bit resolution pipeline
    analogReadResolution(12);
    int ecgValue = analogRead(ecgPin);

    // Fetch GSR using 10-bit resolution pipeline
    analogReadResolution(10);
    int gsrRaw = analogRead(gsrPin);

    // Compute GSR electrical conductance
    float resistance = 0.0;
    float conductance = 0.0;
    if (gsrRaw < 1022) {
      resistance = ((1023.0 + 2.0 * gsrRaw) * 10000.0) / (512.0 - (gsrRaw / 2.0));
      conductance = 1000000.0 / resistance;
    }

    // Format string precisely for Arduino/PlatformIO Unified Plotter parsing
    // Plot lines: ECG_Wave, Raw_Skin_GSR, and calculated Stress Conductance
    Serial.printf("ECG:%d,GSR_Raw:%d,Conductance_uS:%.2f\n", ecgValue, gsrRaw, conductance);
  }

  // TASK 2: Low-Speed Numerical Diagnostics (SpO2, Temp, BP Module)
  if (currentTime - lastSlowSampleTime >= slowSampleInterval) {
    lastSlowSampleTime = currentTime;
    
    Serial.println("\n--- [DIAGNOSTIC SNAPSHOT] ---");
    readPressureModule();
    readTemperature();
    readMAX30102();
    Serial.println("-----------------------------\n");
  }
}
