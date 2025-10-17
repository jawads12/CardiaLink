#include <Wire.h>

#define I2C   Wire
#define ADDR  0x48
#define TEMP_OFFSET 63.5     // empirically derived offset (+61 °C)

float readTempCorrected() {
  I2C.beginTransmission(ADDR);
  I2C.write(0x00);
  if (I2C.endTransmission(false) != 0) return NAN;
  if (I2C.requestFrom((int)ADDR, 2) != 2) return NAN;

  uint8_t msb = I2C.read(), lsb = I2C.read();
  int16_t raw = (int16_t)((msb << 8) | lsb);
  float tempC = raw * (1.0f / 256.0f);   // normal MAX30205 scaling
  return tempC + TEMP_OFFSET;            // apply offset correction
}

void setup() {
  Serial.begin(115200);
  I2C.begin();
  I2C.setClock(100000);
  delay(300);
  Serial.println("MAX30205 (offset-corrected) reading…");
}

void loop() {
  float t = readTempCorrected();
  if (isnan(t)) Serial.println("Read error");
  else {
   // Serial.print("Temperature: ");
    Serial.println(t, 2);
    //Serial.println(" °C");
  }
  delay(1000);
}

