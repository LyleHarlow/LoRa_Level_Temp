//      ******************************************************************
//      *                                                                *
//      *   Minimal ILI9341 test -- bypasses TouchUserInterfaceForArduino *
//      *                                                                *
//      ******************************************************************

// Last updated: 2026-07-25 09:09 PDT

//
// Standalone diagnostic for the Airstream board's touchscreen. Talks to
// the Adafruit_ILI9341 driver directly with nothing else running (no LoRa,
// no MPU6050, no TouchUserInterfaceForArduino) to rule out any interaction
// between those and isolate whether the LCD hardware/wiring itself works
// at the most basic level: can we fill the screen a solid color at all?
//
// If this ALSO shows nothing, the problem is wiring/hardware (LCD_CS,
// LCD_DC, LCD_RST, or the SPI lines specific to the LCD connector), not
// anything in the higher-level sketch code.
//

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

const int LCD_CS_PIN = 2;
const int LCD_DC_PIN = 17;
// LCD_RESET is jumpered to the LED_RST net (GPIO16), not LCD_RST (GPIO33) --
// GPIO33 isn't actually connected to the panel's reset pin on this board.
const int LCD_RST_PIN = 16;
const int LCD_BACKLIGHT_PIN = 12;

const int SPI_SCK_PIN = 5;
const int SPI_MISO_PIN = 19;
const int SPI_MOSI_PIN = 27;

Adafruit_ILI9341 tft(LCD_CS_PIN, LCD_DC_PIN);

void fillAndReport(const char *colorName, uint16_t color)
{
  Serial.print("Filling screen: ");
  Serial.println(colorName);
  tft.fillScreen(color);
  delay(2000);
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("LCDTest starting");

  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, LOW);
  delay(20);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(150);
  Serial.println("LCD_RST pulsed");

  pinMode(LCD_BACKLIGHT_PIN, OUTPUT);
  digitalWrite(LCD_BACKLIGHT_PIN, HIGH);
  Serial.println("Backlight on");

  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  Serial.println("SPI.begin() done");

  Serial.println("Calling tft.begin()...");
  tft.begin();
  Serial.println("tft.begin() returned");

  Serial.print("tft.width()=");
  Serial.print(tft.width());
  Serial.print(" tft.height()=");
  Serial.println(tft.height());
}

void loop()
{
  fillAndReport("RED", ILI9341_RED);
  fillAndReport("GREEN", ILI9341_GREEN);
  fillAndReport("BLUE", ILI9341_BLUE);
  fillAndReport("WHITE", ILI9341_WHITE);
  fillAndReport("BLACK", ILI9341_BLACK);

  Serial.println("Drawing text...");
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(10, 10);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.println("HELLO");
  delay(3000);
}
