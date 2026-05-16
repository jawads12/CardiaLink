#include <Arduino.h>

const int ecgPin = 40;// Pin 40 corresponds to ADC2_A16 on Teensy 4.1

void setup() {
  // Initialize serial communication at a high speed
  // Note: Teensy 4.1 ignores the baud rate and communicates at USB speeds
  Serial.begin(115200);
  
  // Set ADC resolution (Options: 10, 12, or 16 bits). 
  // 12-bit gives a good balance of speed and detail for ECG waves.
  analogReadResolution(12);
}

void loop() {
  // Read the analog value from the ECG sensor
  int ecgValue = analogRead(ecgPin);
  
  // Plot the value clearly with a label
  Serial.print("ECG_Signal:");
  Serial.println(ecgValue);
  
  // Small delay to achieve a stable sampling rate. 
  // ~4ms to ~10ms is typically recommended to clearly visualize the waveform.
  delay(5); 
}
