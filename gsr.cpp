#include <Arduino.h>

const int gsrPin = 20; 
long gsrSum = 0;

void setup() {
  Serial.begin(115200);
  // Set to 10-bit (0-1023) to natively align with standard GSR circuit math
  analogReadResolution(10); 
}

void loop() {
  gsrSum = 0;
  for (int i = 0; i < 10; i++) {
    gsrSum += analogRead(gsrPin);
    delay(1);
  }
  int gsrAverage = gsrSum / 10;
  
  // Adjusted math for 3.3V Teensy rail logic and 10-bit resolution
  // Prevents division by zero or negative clipping numbers
  float resistance = 0.0;
  float conductance = 0.0;
  
  if (gsrAverage < 1022) { // Protects against absolute max rail saturation
    resistance = ((1023.0 + 2.0 * gsrAverage) * 10000.0) / (512.0 - (gsrAverage / 2.0));
    conductance = 1000000.0 / resistance;
  }

  Serial.print("Raw_ADC:");
  Serial.print(gsrAverage);
  Serial.print(",");
  Serial.print("Conductance_uS:");
  Serial.println(conductance);
  
  delay(20);
}
