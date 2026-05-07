#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#define TFT_CS   10
#define TFT_DC   9
#define TFT_RST  8

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

int x = 0;

void setup() {
  delay(500);

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);        // 160 x 128 landscape
  tft.setSPISpeed(4000000);

  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("Teensy 4.1");

  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.setCursor(10, 40);
  tft.println("ST7735S");
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 65);
  tft.println("128 x 160 Display");
}

void loop() {
  tft.fillRect(0, 85, 160, 35, ST77XX_BLACK);

  tft.fillCircle(x, 100, 8, ST77XX_RED);

  x += 3;
  if (x > 160) {
    x = 0;
  }

  delay(40);
}