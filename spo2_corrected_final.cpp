#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define MY_BUFFER_SIZE 50

uint32_t irBuffer[MY_BUFFER_SIZE];
uint32_t redBuffer[MY_BUFFER_SIZE];

int32_t spo2;
int8_t spo2Valid;
int32_t heartRate;
int8_t hrValid;

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire1.begin();   // Teensy 4.1: SDA1=17, SCL1=16

  if (!particleSensor.begin(Wire1, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Check wiring!");
    Serial.println("Teensy 17 -> SDA");
    Serial.println("Teensy 16 -> SCL");
    Serial.println("3.3V -> VCC");
    Serial.println("GND -> GND");
    while (1);
  }

  Serial.println("Place your finger gently on the sensor...");

  particleSensor.setup();

  // lower LED current for testing
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);
  particleSensor.setPulseAmplitudeGreen(0x00);
}

void loop() {
  for (int i = 0; i < MY_BUFFER_SIZE; i++) {
    while (!particleSensor.available()) {
      particleSensor.check();
    }

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();

    particleSensor.nextSample();
  }

  Serial.print("IR=");
  Serial.print(irBuffer[MY_BUFFER_SIZE / 2]);
  Serial.print("  RED=");
  Serial.println(redBuffer[MY_BUFFER_SIZE / 2]);

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, MY_BUFFER_SIZE,
    redBuffer,
    &spo2, &spo2Valid,
    &heartRate, &hrValid
  );

  Serial.print("SpO2: ");
  if (spo2Valid) Serial.print(spo2);
  else Serial.print("Invalid");

  Serial.print("  BPM: ");
  if (hrValid) Serial.print(heartRate);
  else Serial.print("Invalid");

  Serial.println();

  delay(1000);
}