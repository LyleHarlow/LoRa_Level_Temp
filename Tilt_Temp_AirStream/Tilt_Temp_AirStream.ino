//      ******************************************************************
//      *                                                                *
//      *      Airstream v2.1 board -- tilt, temps, LoRa, touch UI       *
//      *                                                                *
//      ******************************************************************

//
// Heltec WiFi LoRa 32 V2. Reads MPU6050 tilt and 4 NTC-thermistor temps,
// shows them on the ILI9341 touchscreen, and transmits them to the
// TowVehicle board over LoRa. See ../hardware for schematics and
// C:\Users\Harlow\.claude\plans\cozy-sauteeing-whale.md for the full plan
// this was built from.
//
// IMPORTANT: the tilt axis mapping below is an UNVERIFIED PLACEHOLDER.
// Run AxisOrientationTest.ino on the physically (vertically) mounted board
// first and correct PITCH_FROM_AXES / ROLL_FROM_AXES / PITCH_SIGN /
// ROLL_SIGN to match what you observe. See the comment block below.
//

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
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
const int LCD_RST_PIN = 33;
const int LCD_BACKLIGHT_PIN = 12;
const int TOUCH_CS_PIN = 21;
// TOUCH_IRQ_PIN (GPIO38) is on the schematic but not currently used by the library

// Sensors. TEMP1/TEMP2 are on ESP32's ADC2, which the WiFi radio also uses --
// reads can be unreliable while WiFi is active (LoRa/SPI/I2C are unaffected).
// Not a concern today since this sketch doesn't use WiFi, but worth knowing
// if that ever changes (e.g. adding OTA updates).
const int TEMP1_PIN = 13;  // Fridge (ADC2)
const int TEMP2_PIN = 25;  // Freezer (ADC2)
const int TEMP3_PIN = 36;  // Inside Airstream (ADC1)
const int TEMP4_PIN = 39;  // DC Electrical Cabinet, battery bank + inverter (ADC1)
const int AMBLIGHTSENSE_PIN = 37;

// Relay -- originally for a fridge cooling fan, no longer needed (fridge is
// now self-cooling 12VDC). Held off; reserved for a future over-temp alarm.
const int OUT2_RELAY_PIN = 32;

// I/O-1 (GPIO23) is a spare pin on the connector, unused for now

// ---------------------------------------------------------------------------------
//                          Tilt axis mapping -- UNVERIFIED, see header comment
// ---------------------------------------------------------------------------------
// Run AxisOrientationTest.ino on the physically-mounted board, tilt the nose
// up/down and watch which of angleFromXZ/YZ/XY moves (and which direction),
// then do the same for left up/down. Update the two macros and two signs
// below to match. As shipped these are just a starting guess (XZ->pitch,
// YZ->roll) and are very likely wrong for a vertically-mounted board.
#define PITCH_FROM_AXES(ax, ay, az) (atan2((ax), (az)) * 180.0 / PI)  // nose up(+)/down(-)
#define ROLL_FROM_AXES(ax, ay, az) (atan2((ay), (az)) * 180.0 / PI)   // left up(+)/down(-)
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

// NTC thermistor conversion (3.3k series resistor, NTC to ground -- see
// schematic). NOMINAL_RESISTANCE/B_COEFFICIENT are common 10k-NTC defaults
// and are UNVERIFIED against the actual probe part; raw ADC counts are
// printed to Serial to make calibrating these easier.
const float SERIES_RESISTOR = 3300.0;
const float NOMINAL_RESISTANCE = 10000.0;
const float NOMINAL_TEMPERATURE_C = 25.0;
const float B_COEFFICIENT = 3950.0;

// ---------------------------------------------------------------------------------

Adafruit_MPU6050 mpu;
TouchUserInterfaceForArduino ui;

LoRaPacket txPacket;
float pitchOffset = 0;
float rollOffset = 0;

const int TILT_SAMPLE_COUNT = 8;
float pitchSamples[TILT_SAMPLE_COUNT];
float rollSamples[TILT_SAMPLE_COUNT];
int tiltSampleIndex = 0;

unsigned long lastTransmitTime = 0;
const unsigned long TRANSMIT_INTERVAL_MS = 2000;

BUTTON zeroLevelButton = {"Zero Level", 260, 210, 100, 40};

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

  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH);
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

  ui.begin(LCD_CS_PIN, LCD_DC_PIN, TOUCH_CS_PIN, LCD_ORIENTATION_LANDSCAPE_4PIN_LEFT, UI_Font_13_Bold);

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

float readThermistorTempF(int pin)
{
  int adc = analogRead(pin);
  float voltageRatio = (float)adc / 4095.0;

  if (voltageRatio <= 0.001 || voltageRatio >= 0.999)
  {
    return NAN;  // probe likely open or shorted
  }

  float resistance = SERIES_RESISTOR * voltageRatio / (1.0 - voltageRatio);

  float steinhart = resistance / NOMINAL_RESISTANCE;
  steinhart = log(steinhart);
  steinhart /= B_COEFFICIENT;
  steinhart += 1.0 / (NOMINAL_TEMPERATURE_C + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;  // deg C

  return steinhart * 9.0 / 5.0 + 32.0;
}

void updateTemperatures()
{
  txPacket.temp1 = readThermistorTempF(TEMP1_PIN);
  txPacket.temp2 = readThermistorTempF(TEMP2_PIN);
  txPacket.temp3 = readThermistorTempF(TEMP3_PIN);
  txPacket.temp4 = readThermistorTempF(TEMP4_PIN);
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
  Serial.print(txPacket.temp4, 1);
  Serial.print("  light=");
  Serial.println(analogRead(AMBLIGHTSENSE_PIN));
}

// ---------------------------------------------------------------------------------
//                                    Display
// ---------------------------------------------------------------------------------

const int VALUE_COLUMN_X = 170;
const int LINE_HEIGHT = 26;
const int FIRST_LINE_Y = 40;

void drawInfoScreenLayout()
{
  ui.drawTitleBar("Airstream Monitor");
  ui.clearDisplaySpace();

  const char *labels[] = {"Nose Up/Down", "Left Up/Down", "Fridge", "Freezer", "Inside AS", "DC Cabinet", "Light"};
  for (int i = 0; i < 7; i++)
  {
    ui.lcdSetCursorXY(10, FIRST_LINE_Y + i * LINE_HEIGHT);
    ui.lcdPrint(labels[i]);
  }

  ui.drawButton(zeroLevelButton);
}

void drawValueField(int lineIndex, const char *text)
{
  int y = FIRST_LINE_Y + lineIndex * LINE_HEIGHT;
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

  snprintf(buf, sizeof(buf), "%d", analogRead(AMBLIGHTSENSE_PIN));
  drawValueField(6, buf);
}
