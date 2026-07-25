//      ******************************************************************
//      *                                                                *
//      *      Airstream v2.1 board -- tilt, temps, LoRa, touch UI       *
//      *                                                                *
//      ******************************************************************

// Last updated: 2026-07-25 09:27 PDT

//
// Heltec WiFi LoRa 32 V2. Reads MPU6050 tilt and 4 DS18B20 (OneWire) temp
// probes, shows them on the ILI9341 touchscreen, and transmits them to the
// TowVehicle board over LoRa. See ../hardware for schematics and
// C:\Users\Harlow\.claude\plans\cozy-sauteeing-whale.md for the full plan
// this was built from.
//
// This board intentionally does NOT use Heltec's Heltec_ESP32 library
// (unlike Tilt_Temp_TowVehc) -- that library's Vext handling claims
// GPIO21, which on this board is TOUCH_CS. Manual SPI/LoRa setup below
// avoids that conflict.
//
// NOTE: the tilt axis formulas below (PITCH_FROM_AXES / ROLL_FROM_AXES) were
// derived from running AxisOrientationTest.ino on the physically mounted
// board. PITCH_SIGN/ROLL_SIGN still need a quick real-world check -- see
// the comment block below.
//

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LoRa.h>
#include <TouchUserInterfaceForArduino.h>
#include <UI_Fonts.h>
#include "LoRaPacket.h"

char this_file[] = "Tilt_Temp_AirStream";
char ver[] = "ver 2.1  " __DATE__;

// ---------------------------------------------------------------------------------
//                                    Pin definitions
// ---------------------------------------------------------------------------------
// See the Confirmed Hardware section of the plan for how these were derived
// (schematic + the user's own Heltec pinout spreadsheet).

// I2C -- MPU6050 (RTC deferred, not wired up in this version)
const int SDA_PIN = 4;
const int SCL_PIN = 15;
const int MPU6050_ADDRESS = 0x69;  // ADO jumpered to 3.3V; keeps clear of PCF8523 RTC's fixed 0x68

// SPI -- shared with the onboard LoRa radio; each device has its own CS
const int SPI_SCK_PIN = 5;
const int SPI_MISO_PIN = 19;
const int SPI_MOSI_PIN = 27;

// LoRa radio (standard Heltec WiFi LoRa 32 V2 pins)
const int LORA_CS_PIN = 18;
const int LORA_RST_PIN = 14;
const int LORA_DIO0_PIN = 26;
const long LORA_BAND = 915E6;

// Touchscreen / LCD (ILI9341 + XPT2046)
const int LCD_CS_PIN = 2;
const int LCD_DC_PIN = 17;
// LCD_RESET is jumpered to the LED_RST net (GPIO16), not the LCD_RST net
// (GPIO33) -- GPIO33 isn't actually connected to the panel's reset pin on
// this board. Confirmed by the user; previously used GPIO33 here, which
// left the real reset pin floating and was very likely why the display
// never showed anything.
const int LCD_RST_PIN = 16;
const int LCD_BACKLIGHT_PIN = 12;
const int TOUCH_CS_PIN = 21;
// TOUCH_IRQ_PIN (GPIO38) is on the schematic but not currently used by the library

// Sensors. TEMP1-4 are each a DS18B20 (BOJACK 1M stainless probe) on its
// own OneWire bus, with the schematic's 3.3k resistor acting as the
// OneWire pull-up (matches the original code's OneWire-based approach on
// these same physical pins, just swapped between TEMP1/TEMP2).
const int TEMP1_PIN = 13;  // Fridge
const int TEMP2_PIN = 25;  // Freezer
const int TEMP3_PIN = 36;  // Inside Airstream
const int TEMP4_PIN = 39;  // DC Electrical Cabinet, battery bank + inverter
// AMBLIGHTSENSE_PIN = 37, disabled -- photoresistor isn't installed yet, and
// (separately) analogRead() on this pin was crashing the ADC driver anyway.
// Re-enable once the sensor is physically installed and that's investigated.

// Relay -- originally for a fridge cooling fan, no longer needed (fridge is
// now self-cooling 12VDC). Held off; reserved for a future over-temp alarm.
const int OUT2_RELAY_PIN = 32;

// I/O-1 (GPIO23) is a spare pin on the connector, unused for now

// ---------------------------------------------------------------------------------
//                          Tilt axis mapping -- derived from AxisOrientationTest.ino
// ---------------------------------------------------------------------------------
// Empirically determined on the physically (vertically) mounted board:
//  - left up/down tracks atan2(ax, ay), reading 90 deg at level -- so
//    subtracting 90 deg gives a clean "0 = level" roll value.
//  - nose up/down tracks raw Z acceleration; atan2(az, sqrt(ax^2+ay^2)) is
//    the equivalent robust angle (stays linear in degrees across the full
//    range, unlike raw Z which flattens out near +-90 deg).
//
// Sign (+/- direction) has NOT been verified yet -- raise the nose and
// confirm "Nose Up/Down" reads positive on the info screen (flip
// PITCH_SIGN to -1.0 if backwards); do the same for the left side and
// ROLL_SIGN.
#define PITCH_FROM_AXES(ax, ay, az) (atan2((az), sqrt((ax) * (ax) + (ay) * (ay))) * 180.0 / PI)  // nose up(+)/down(-)
#define ROLL_FROM_AXES(ax, ay, az) ((atan2((ax), (ay)) * 180.0 / PI) - 90.0)                      // left up(+)/down(-)
const float PITCH_SIGN = 1.0;
const float ROLL_SIGN = 1.0;

// Distance (inches) between the trailer's leveling/jack points. Left at 0
// (disabled -- falls back to showing degrees) until measured for real.
const float LEVEL_POINT_DISTANCE_INCHES = 0;

// EEPROM addresses for the persisted zero-calibration offsets (see
// TouchUserInterfaceForArduino's writeConfigurationFloat/readConfigurationFloat --
// each float uses 5 bytes, so keep these at least 5 apart)
const int EEPROM_ADDR_PITCH_OFFSET = 0;
const int EEPROM_ADDR_ROLL_OFFSET = 10;

// DS18B20 conversion timing (~750ms at default 12-bit resolution). Reads
// are done as a non-blocking request-then-collect cycle so they don't
// stall the tilt-sampling/touch loop.
const unsigned long TEMP_UPDATE_INTERVAL_MS = 2000;
const unsigned long TEMP_CONVERSION_TIME_MS = 750;

// ---------------------------------------------------------------------------------

Adafruit_MPU6050 mpu;
TouchUserInterfaceForArduino ui;

OneWire oneWireTemp1(TEMP1_PIN);
OneWire oneWireTemp2(TEMP2_PIN);
OneWire oneWireTemp3(TEMP3_PIN);
OneWire oneWireTemp4(TEMP4_PIN);
DallasTemperature dsTemp1(&oneWireTemp1);
DallasTemperature dsTemp2(&oneWireTemp2);
DallasTemperature dsTemp3(&oneWireTemp3);
DallasTemperature dsTemp4(&oneWireTemp4);

enum TempReadState
{
  TEMP_IDLE,
  TEMP_CONVERTING
};
TempReadState tempReadState = TEMP_IDLE;
unsigned long tempConversionStartTime = 0;
unsigned long lastTempUpdateTime = 0;

LoRaPacket txPacket;
float pitchOffset = 0;
float rollOffset = 0;

const int TILT_SAMPLE_COUNT = 8;
float pitchSamples[TILT_SAMPLE_COUNT];
float rollSamples[TILT_SAMPLE_COUNT];
int tiltSampleIndex = 0;

unsigned long lastTransmitTime = 0;
const unsigned long TRANSMIT_INTERVAL_MS = 2000;

BUTTON zeroLevelButton = {"Zero Level", 0, 0, 110, 32};  // position set in drawInfoScreenLayout(), once display space bounds are known

// ---------------------------------------------------------------------------------
//                                     Setup
// ---------------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println(this_file);
  Serial.println(ver);

  // ESP32's EEPROM library is flash-backed and requires begin() before use --
  // the UI library's writeConfigurationFloat/readConfigurationFloat assume
  // this has already been called (see TouchUserInterfaceForArduino.cpp).
  EEPROM.begin(64);

  pinMode(OUT2_RELAY_PIN, OUTPUT);
  digitalWrite(OUT2_RELAY_PIN, LOW);

  setupTiltSensor();
  Serial.println("Tilt sensor init done");
  setupTempSensors();
  Serial.println("Temp sensors init done");

  // Proper hardware reset pulse (LOW then HIGH) rather than just driving
  // HIGH -- the ILI9341 driver also does its own software reset since we
  // don't pass this pin to its constructor, but a clean hardware reset
  // first can't hurt and rules out a bad power-on state.
  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, LOW);
  delay(20);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(150);
  pinMode(LCD_BACKLIGHT_PIN, OUTPUT);
  digitalWrite(LCD_BACKLIGHT_PIN, HIGH);

  // Shared SPI bus (LoRa radio + LCD + touch controller), custom pins --
  // must be configured before LoRa.begin() and ui.begin(), both of which
  // use the default SPI object internally.
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  if (!LoRa.begin(LORA_BAND))
  {
    Serial.println("Starting LoRa failed!");
  }
  else
  {
    Serial.println("LoRa init OK");
  }
  // LoRa.begin() defaults to 17dBm via PA_BOOST, which draws a real current
  // spike (~100mA+) on transmit -- enough to visibly sag the shared 3.3V
  // rail and flicker the LCD without tripping a brownout reset. This link
  // only needs to reach the tow vehicle (tens of feet), so a much lower
  // power is plenty; raise it if range testing shows it's needed.
  LoRa.setTxPower(2);

  Serial.println("Calling ui.begin()...");
  ui.begin(LCD_CS_PIN, LCD_DC_PIN, TOUCH_CS_PIN, LCD_ORIENTATION_LANDSCAPE_4PIN_LEFT, UI_Font_13_Bold);
  Serial.println("ui.begin() returned");

  pitchOffset = ui.readConfigurationFloat(EEPROM_ADDR_PITCH_OFFSET, 0);
  rollOffset = ui.readConfigurationFloat(EEPROM_ADDR_ROLL_OFFSET, 0);

  for (int i = 0; i < TILT_SAMPLE_COUNT; i++)
  {
    pitchSamples[i] = 0;
    rollSamples[i] = 0;
  }

  drawInfoScreenLayout();
}

void setupTiltSensor()
{
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!mpu.begin(MPU6050_ADDRESS, &Wire))
  {
    Serial.println("MPU6050 not found at 0x69 -- check the ADO jumper and wiring");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void setupTempSensors()
{
  dsTemp1.begin();
  dsTemp2.begin();
  dsTemp3.begin();
  dsTemp4.begin();

  // non-blocking mode: requestTemperatures() returns immediately, we poll
  // for the result ourselves after TEMP_CONVERSION_TIME_MS (see updateTemperatures)
  dsTemp1.setWaitForConversion(false);
  dsTemp2.setWaitForConversion(false);
  dsTemp3.setWaitForConversion(false);
  dsTemp4.setWaitForConversion(false);
}

// ---------------------------------------------------------------------------------
//                                      Loop
// ---------------------------------------------------------------------------------

void loop()
{
  updateTilt();
  updateTemperatures();

  ui.getTouchEvents();
  if (ui.checkForButtonClicked(zeroLevelButton))
  {
    pitchOffset += txPacket.pitch;
    rollOffset += txPacket.roll;
    ui.writeConfigurationFloat(EEPROM_ADDR_PITCH_OFFSET, pitchOffset);
    ui.writeConfigurationFloat(EEPROM_ADDR_ROLL_OFFSET, rollOffset);
  }

  drawInfoScreenValues();

  if (millis() - lastTransmitTime >= TRANSMIT_INTERVAL_MS)
  {
    lastTransmitTime = millis();
    transmitPacket();
  }

  delay(50);
}

// ---------------------------------------------------------------------------------
//                                  Tilt sensing
// ---------------------------------------------------------------------------------

void updateTilt()
{
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  pitchSamples[tiltSampleIndex] = PITCH_SIGN * PITCH_FROM_AXES(ax, ay, az);
  rollSamples[tiltSampleIndex] = ROLL_SIGN * ROLL_FROM_AXES(ax, ay, az);
  tiltSampleIndex = (tiltSampleIndex + 1) % TILT_SAMPLE_COUNT;

  float pitchSum = 0;
  float rollSum = 0;
  for (int i = 0; i < TILT_SAMPLE_COUNT; i++)
  {
    pitchSum += pitchSamples[i];
    rollSum += rollSamples[i];
  }

  float pitchDegrees = (pitchSum / TILT_SAMPLE_COUNT) - pitchOffset;
  float rollDegrees = (rollSum / TILT_SAMPLE_COUNT) - rollOffset;

  if (LEVEL_POINT_DISTANCE_INCHES > 0)
  {
    txPacket.pitch = tan(pitchDegrees * PI / 180.0) * LEVEL_POINT_DISTANCE_INCHES;
    txPacket.roll = tan(rollDegrees * PI / 180.0) * LEVEL_POINT_DISTANCE_INCHES;
  }
  else
  {
    txPacket.pitch = pitchDegrees;
    txPacket.roll = rollDegrees;
  }
}

// ---------------------------------------------------------------------------------
//                                 Temperature sensing
// ---------------------------------------------------------------------------------

float readDS18B20(DallasTemperature &sensor)
{
  float f = sensor.getTempFByIndex(0);
  if (f == DEVICE_DISCONNECTED_F)
  {
    return NAN;  // probe missing or wiring fault
  }
  return f;
}

// Non-blocking: kicks off a conversion on all 4 probes every
// TEMP_UPDATE_INTERVAL_MS, then collects the results once conversion time
// has elapsed. Does not delay() -- safe to call every loop iteration.
void updateTemperatures()
{
  unsigned long now = millis();

  if (tempReadState == TEMP_IDLE && now - lastTempUpdateTime >= TEMP_UPDATE_INTERVAL_MS)
  {
    dsTemp1.requestTemperatures();
    dsTemp2.requestTemperatures();
    dsTemp3.requestTemperatures();
    dsTemp4.requestTemperatures();
    tempConversionStartTime = now;
    tempReadState = TEMP_CONVERTING;
  }
  else if (tempReadState == TEMP_CONVERTING && now - tempConversionStartTime >= TEMP_CONVERSION_TIME_MS)
  {
    txPacket.temp1 = readDS18B20(dsTemp1);
    txPacket.temp2 = readDS18B20(dsTemp2);
    txPacket.temp3 = readDS18B20(dsTemp3);
    txPacket.temp4 = readDS18B20(dsTemp4);
    lastTempUpdateTime = now;
    tempReadState = TEMP_IDLE;
  }
}

// ---------------------------------------------------------------------------------
//                                 LoRa transmit
// ---------------------------------------------------------------------------------

void transmitPacket()
{
  LoRa.beginPacket();
  LoRa.write((uint8_t *)&txPacket, sizeof(txPacket));
  LoRa.endPacket();

  Serial.print("TX  pitch=");
  Serial.print(txPacket.pitch, 1);
  Serial.print("  roll=");
  Serial.print(txPacket.roll, 1);
  Serial.print("  t1=");
  Serial.print(txPacket.temp1, 1);
  Serial.print("  t2=");
  Serial.print(txPacket.temp2, 1);
  Serial.print("  t3=");
  Serial.print(txPacket.temp3, 1);
  Serial.print("  t4=");
  Serial.println(txPacket.temp4, 1);
}

// ---------------------------------------------------------------------------------
//                                    Display
// ---------------------------------------------------------------------------------

const int VALUE_COLUMN_X = 170;
const int LINE_HEIGHT = 22;

// Set once in drawInfoScreenLayout() from ui.displaySpaceTopY (which depends
// on the title bar's actual height), reused by drawValueField() so the two
// stay in sync instead of guessing pixel offsets independently.
int firstLineY = 0;

void drawInfoScreenLayout()
{
  ui.drawTitleBar("Airstream Monitor");
  ui.clearDisplaySpace();

  firstLineY = ui.displaySpaceTopY + 6;

  const char *labels[] = {"Nose Up/Down", "Left Up/Down", "Fridge", "Freezer", "Inside AS", "DC Cabinet"};
  for (int i = 0; i < 6; i++)
  {
    ui.lcdSetCursorXY(ui.displaySpaceLeftX + 6, firstLineY + i * LINE_HEIGHT);
    ui.lcdPrint(labels[i]);
  }

  // Anchored to the bottom-right of the display space so it can't overlap
  // the label/value rows above, regardless of exact screen/title bar size.
  zeroLevelButton.centerX = ui.displaySpaceRightX - zeroLevelButton.width / 2 - 6;
  zeroLevelButton.centerY = ui.displaySpaceBottomY - zeroLevelButton.height / 2 - 6;
  ui.drawButton(zeroLevelButton);
}

void drawValueField(int lineIndex, const char *text)
{
  int y = firstLineY + lineIndex * LINE_HEIGHT;
  ui.lcdDrawFilledRectangle(VALUE_COLUMN_X, y, 120, ui.lcdGetFontHeightWithDecenders(), LCD_BLACK);
  ui.lcdSetCursorXY(VALUE_COLUMN_X, y);
  ui.lcdPrint(text);
}

void drawInfoScreenValues()
{
  char buf[24];
  const char *unit = (LEVEL_POINT_DISTANCE_INCHES > 0) ? "in" : "deg";

  snprintf(buf, sizeof(buf), "%.1f %s", txPacket.pitch, unit);
  drawValueField(0, buf);

  snprintf(buf, sizeof(buf), "%.1f %s", txPacket.roll, unit);
  drawValueField(1, buf);

  snprintf(buf, sizeof(buf), "%.1f F", txPacket.temp1);
  drawValueField(2, buf);

  snprintf(buf, sizeof(buf), "%.1f F", txPacket.temp2);
  drawValueField(3, buf);

  snprintf(buf, sizeof(buf), "%.1f F", txPacket.temp3);
  drawValueField(4, buf);

  snprintf(buf, sizeof(buf), "%.1f F", txPacket.temp4);
  drawValueField(5, buf);
}
