//      ******************************************************************
//      *                                                                *
//      *      Airstream v2.1 board -- tilt, temps, LoRa, touch UI       *
//      *                                                                *
//      ******************************************************************

// Last updated: 2026-07-26 08:53 PDT

//
// Heltec WiFi LoRa 32 V2. Reads MPU6050 tilt, 4 DS18B20 (OneWire) temp
// probes, and a PCF8523 RTC; drives a relay off one of the temp probes;
// shows everything on a touchscreen menu (Level / Temps / Fan Control /
// Settings); transmits tilt+temps to the TowVehicle board over LoRa. See
// ../hardware for schematics and C:\Users\Harlow\.claude\plans\cozy-sauteeing-whale.md
// for the plan this menu system was built from.
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
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LoRa.h>
#include <TouchUserInterfaceForArduino.h>
#include <UI_Fonts.h>
#include "LoRaPacket.h"

char this_file[] = "Tilt_Temp_AirStream";
char ver[] = "ver 2.2  " __DATE__;

// ---------------------------------------------------------------------------------
//                                    Pin definitions
// ---------------------------------------------------------------------------------
// See the Confirmed Hardware section of the plan for how these were derived
// (schematic + the user's own Heltec pinout spreadsheet).

// I2C -- MPU6050 and PCF8523 RTC share this bus at different addresses.
// MPU6050_ADDRESS is jumpered to 0x69 (ADO tied to 3.3V) specifically to
// stay clear of the PCF8523's fixed 0x68 -- both devices work fine
// together as long as Wire.begin() is only called once (here) and each
// library's begin() is passed that same already-configured &Wire, rather
// than letting a library call its own Wire.begin() with default pins.
const int SDA_PIN = 4;
const int SCL_PIN = 15;
const int MPU6050_ADDRESS = 0x69;
const int PCF8523_ADDRESS = 0x68;  // fixed by the chip, not configurable

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

// Relay -- driven by updateFanControl() based on the Fan Control screen's
// settings (selected probe, min/max thresholds, optional schedule window).
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
// confirm "Nose High" shows on the Level screen; flip PITCH_SIGN to -1.0
// if backwards; do the same for the left side and ROLL_SIGN.
#define PITCH_FROM_AXES(ax, ay, az) (atan2((az), sqrt((ax) * (ax) + (ay) * (ay))) * 180.0 / PI)  // nose up(+)/down(-)
#define ROLL_FROM_AXES(ax, ay, az) ((atan2((ax), (ay)) * 180.0 / PI) - 90.0)                      // left up(+)/down(-)
const float PITCH_SIGN = 1.0;
const float ROLL_SIGN = 1.0;

// Distance (inches) between the trailer's leveling/jack points. Left at 0
// (disabled -- falls back to showing degrees) until measured for real.
const float LEVEL_POINT_DISTANCE_INCHES = 0;

// EEPROM addresses (see TouchUserInterfaceForArduino's writeConfigurationFloat/Int --
// each value uses 5 bytes, so keep these at least 5 apart; using 10 for headroom).
const int EEPROM_ADDR_PITCH_OFFSET = 0;
const int EEPROM_ADDR_ROLL_OFFSET = 10;
const int EEPROM_ADDR_FAN_PROBE = 20;        // int: 0=Fridge,1=Freezer,2=Inside AS,3=DC Cabinet
const int EEPROM_ADDR_FAN_MIN_TEMP = 30;     // float, deg F
const int EEPROM_ADDR_FAN_MAX_TEMP = 40;     // float, deg F
const int EEPROM_ADDR_FAN_TIME_ENABLE = 50;  // int: 0=schedule ignored, 1=schedule gates operation
const int EEPROM_ADDR_FAN_START_HOUR = 60;   // int, 0-23
const int EEPROM_ADDR_FAN_END_HOUR = 70;     // int, 0-23

// DS18B20 conversion timing (~750ms at default 12-bit resolution). Reads
// are done as a non-blocking request-then-collect cycle so they don't
// stall the tilt-sampling/touch loop.
const unsigned long TEMP_UPDATE_INTERVAL_MS = 2000;
const unsigned long TEMP_CONVERSION_TIME_MS = 750;

// ---------------------------------------------------------------------------------

Adafruit_MPU6050 mpu;
RTC_PCF8523 rtc;
bool rtcAvailable = false;
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

// Smoothing averages the raw accelerometer vector (not the derived angles)
// -- averaging angles directly breaks near the atan2 wraparound point.
const int TILT_SAMPLE_COUNT = 8;
float axSamples[TILT_SAMPLE_COUNT];
float aySamples[TILT_SAMPLE_COUNT];
float azSamples[TILT_SAMPLE_COUNT];
int tiltSampleIndex = 0;

unsigned long lastTransmitTime = 0;
const unsigned long TRANSMIT_INTERVAL_MS = 2000;

// Fan control state -- loaded from EEPROM in loadFanSettings(), edited on
// the Fan Control screen (each control persists immediately on change).
const char *FAN_PROBE_NAMES[4] = {"Fridge", "Freezer", "Inside AS", "DC Cabinet"};
int fanProbeIndex = 0;
float fanMinTemp = 70.0;
float fanMaxTemp = 90.0;
int fanTimeEnabled = 0;  // 0/1, also used directly as the SELECTION_BOX value
int fanStartHour = 8;
int fanEndHour = 20;
bool fanOn = false;

// ---------------------------------------------------------------------------------
//                                     Menu
// ---------------------------------------------------------------------------------

extern MENU_ITEM mainMenu[];

// Arduino's auto-prototype generation doesn't pick these up when they're
// only referenced inside the menu table initializer below -- forward
// declared explicitly instead.
void commandLevel(void);
void commandTemps(void);
void commandFanControl(void);
void commandSettings(void);

MENU_ITEM mainMenu[] = {
    {MENU_ITEM_TYPE_MAIN_MENU_HEADER, "Airstream Monitor", MENU_COLUMNS_2, mainMenu},
    {MENU_ITEM_TYPE_COMMAND, "Level", commandLevel, NULL},
    {MENU_ITEM_TYPE_COMMAND, "Temps", commandTemps, NULL},
    {MENU_ITEM_TYPE_COMMAND, "Fan Control", commandFanControl, NULL},
    {MENU_ITEM_TYPE_COMMAND, "Settings", commandSettings, NULL},
    {MENU_ITEM_TYPE_END_OF_MENU, "", NULL, NULL}};

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
  // the UI library's writeConfigurationFloat/Int/readConfigurationFloat/Int
  // assume this has already been called (see TouchUserInterfaceForArduino.cpp).
  // 128 bytes covers the tilt offsets (addr 0,10) and fan settings (20-70).
  EEPROM.begin(128);

  pinMode(OUT2_RELAY_PIN, OUTPUT);
  digitalWrite(OUT2_RELAY_PIN, LOW);

  setupTiltSensor();
  Serial.println("Tilt sensor init done");
  setupRTC();
  Serial.println("RTC init done");
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

  // IMPORTANT: these must exactly match Tilt_Temp_TowVehc's radio settings
  // or the two boards can't decode each other's packets at all -- with no
  // error on either side, just silence. This bit us once already: this
  // library (sandeepmistry/LoRa) leaves these at the SX1276 chip's
  // power-on defaults (SF7, sync word 0x12) unless set explicitly, while
  // TowVehicle's Heltec-vendored LoRa library sets different values
  // (SF11, sync word 0x34) inside its own begin(). Setting them explicitly
  // here, matching TowVehicle, rather than relying on either library's
  // implicit defaults staying the same or matching each other.
  LoRa.setSpreadingFactor(11);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setSyncWord(0x34);
  LoRa.enableCrc();

  Serial.println("Calling ui.begin()...");
  ui.begin(LCD_CS_PIN, LCD_DC_PIN, TOUCH_CS_PIN, LCD_ORIENTATION_LANDSCAPE_4PIN_LEFT, UI_Font_13_Bold);
  Serial.println("ui.begin() returned");

  pitchOffset = ui.readConfigurationFloat(EEPROM_ADDR_PITCH_OFFSET, 0);
  rollOffset = ui.readConfigurationFloat(EEPROM_ADDR_ROLL_OFFSET, 0);
  loadFanSettings();

  // backgroundUpdate() keeps sensors/LoRa/fan-control running while the
  // user is sitting at the main menu (idle, no screen open). Each screen's
  // own command function also calls it directly inside its loop, since
  // this callback only fires from displayAndExecuteMenu()'s own loop, not
  // while a command function is blocking inside its while(true).
  ui.setInMenuCallbackFunction(backgroundUpdate);

  for (int i = 0; i < TILT_SAMPLE_COUNT; i++)
  {
    axSamples[i] = 0;
    aySamples[i] = 0;
    azSamples[i] = 0;
  }
}

void setupTiltSensor()
{
  Wire.begin(SDA_PIN, SCL_PIN);

  // On a true power cycle (unlike a USB-triggered reset/reflash, where the
  // board's own power rails never actually drop -- only the ESP32 CPU
  // resets), the MPU6050 needs a brief settling time after power-up
  // before it reliably ACKs on I2C. A single immediate mpu.begin() call
  // can fail right after cold power-on; if it fails, the old code just
  // logged a warning and kept going with a never-initialized sensor
  // object, which then reads back stuck/frozen values forever -- exactly
  // "Level resets and never updates after a power cycle, but works fine
  // after reflashing" (reflash doesn't fully power-cycle the board).
  // Retry for up to ~1s rather than giving up on the first attempt.
  const int MPU_INIT_MAX_ATTEMPTS = 5;
  bool mpuOk = false;
  for (int attempt = 1; attempt <= MPU_INIT_MAX_ATTEMPTS && !mpuOk; attempt++)
  {
    mpuOk = mpu.begin(MPU6050_ADDRESS, &Wire);
    if (!mpuOk)
    {
      Serial.print("MPU6050 init attempt ");
      Serial.print(attempt);
      Serial.println(" failed, retrying...");
      delay(200);
    }
  }

  if (!mpuOk)
  {
    Serial.println("MPU6050 not found at 0x69 after retries -- check the ADO jumper and wiring");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

// Shares the Wire bus Wire.begin(SDA_PIN, SCL_PIN) already set up in
// setupTiltSensor() -- passing &Wire here reuses that, rather than the RTC
// library calling its own Wire.begin() with default pins. MPU6050 (0x69)
// and this RTC (fixed 0x68) coexist fine as two independent I2C devices.
void setupRTC()
{
  if (!rtc.begin(&Wire))
  {
    Serial.print("PCF8523 RTC not found at 0x");
    Serial.println(PCF8523_ADDRESS, HEX);
    rtcAvailable = false;
    return;
  }

  rtcAvailable = true;

  if (!rtc.initialized() || rtc.lostPower())
  {
    // lostPower() being true on every boot (time set via the Settings
    // screen works, but doesn't survive a power cycle) almost always means
    // the PCF8523 module's CR1220 backup battery is missing, dead, or not
    // seated -- the RTC has no way to keep time with main power removed.
    // Falls back to this sketch's compile time either way (not accurate,
    // but better than nothing).
    Serial.println("RTC lost power or uninitialized -- setting to compile time");
    Serial.println("  (if this happens every boot even after setting the time, check the RTC module's CR1220 battery)");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  rtc.start();

  DateTime now = rtc.now();
  Serial.print("RTC time: ");
  Serial.print(now.year());
  Serial.print('-');
  Serial.print(now.month());
  Serial.print('-');
  Serial.print(now.day());
  Serial.print(' ');
  Serial.print(now.hour());
  Serial.print(':');
  Serial.print(now.minute());
  Serial.print(':');
  Serial.println(now.second());
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

void loadFanSettings()
{
  fanProbeIndex = ui.readConfigurationInt(EEPROM_ADDR_FAN_PROBE, 0);
  fanMinTemp = ui.readConfigurationFloat(EEPROM_ADDR_FAN_MIN_TEMP, 70.0);
  fanMaxTemp = ui.readConfigurationFloat(EEPROM_ADDR_FAN_MAX_TEMP, 90.0);
  fanTimeEnabled = ui.readConfigurationInt(EEPROM_ADDR_FAN_TIME_ENABLE, 0);
  fanStartHour = ui.readConfigurationInt(EEPROM_ADDR_FAN_START_HOUR, 8);
  fanEndHour = ui.readConfigurationInt(EEPROM_ADDR_FAN_END_HOUR, 20);
}

// ---------------------------------------------------------------------------------
//                                      Loop
// ---------------------------------------------------------------------------------

// displayAndExecuteMenu() effectively never returns (matches the library's
// own Example01_SimpleMenu pattern) -- it's the entire runtime loop,
// dispatching to the command functions below and calling backgroundUpdate()
// (registered in setup()) whenever idle at a menu screen.
void loop()
{
  ui.displayAndExecuteMenu(mainMenu);
}

// ---------------------------------------------------------------------------------
//                                  Tilt sensing
// ---------------------------------------------------------------------------------

void updateTilt()
{
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  axSamples[tiltSampleIndex] = accel.acceleration.x;
  aySamples[tiltSampleIndex] = accel.acceleration.y;
  azSamples[tiltSampleIndex] = accel.acceleration.z;
  tiltSampleIndex = (tiltSampleIndex + 1) % TILT_SAMPLE_COUNT;

  float axAvg = 0, ayAvg = 0, azAvg = 0;
  for (int i = 0; i < TILT_SAMPLE_COUNT; i++)
  {
    axAvg += axSamples[i];
    ayAvg += aySamples[i];
    azAvg += azSamples[i];
  }
  axAvg /= TILT_SAMPLE_COUNT;
  ayAvg /= TILT_SAMPLE_COUNT;
  azAvg /= TILT_SAMPLE_COUNT;

  float pitchDegrees = PITCH_SIGN * PITCH_FROM_AXES(axAvg, ayAvg, azAvg) - pitchOffset;
  float rollDegrees = ROLL_SIGN * ROLL_FROM_AXES(axAvg, ayAvg, azAvg) - rollOffset;

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
//                                  Fan control
// ---------------------------------------------------------------------------------

float getFanProbeTemp()
{
  switch (fanProbeIndex)
  {
  case 0:
    return txPacket.temp1;
  case 1:
    return txPacket.temp2;
  case 2:
    return txPacket.temp3;
  default:
    return txPacket.temp4;
  }
}

// Hysteresis thermostat (on above max, off below min, unchanged in
// between) gated by an optional schedule window. Called every iteration
// via backgroundUpdate() so it keeps running regardless of which screen
// is open.
void updateFanControl()
{
  float currentTemp = getFanProbeTemp();

  if (isnan(currentTemp))
  {
    fanOn = false;  // safe default -- don't run on a disconnected probe
  }
  else
  {
    bool withinSchedule = !fanTimeEnabled;
    if (fanTimeEnabled)
    {
      if (!rtcAvailable)
      {
        withinSchedule = false;  // can't verify the schedule without a working RTC -- fail closed
      }
      else
      {
        int hour = rtc.now().hour();
        if (fanStartHour <= fanEndHour)
          withinSchedule = (hour >= fanStartHour && hour < fanEndHour);
        else
          withinSchedule = (hour >= fanStartHour || hour < fanEndHour);  // overnight window
      }
    }

    if (!withinSchedule)
    {
      fanOn = false;
    }
    else if (currentTemp >= fanMaxTemp)
    {
      fanOn = true;
    }
    else if (currentTemp <= fanMinTemp)
    {
      fanOn = false;
    }
    // else: inside the hysteresis band, leave fanOn as-is
  }

  digitalWrite(OUT2_RELAY_PIN, fanOn ? HIGH : LOW);
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
  Serial.print("  fan=");
  Serial.print(fanOn ? "ON" : "OFF");

  // Ongoing proof the RTC keeps working alongside the MPU6050 over time,
  // not just at boot -- watch this tick forward every transmit alongside
  // normal tilt/temp readings.
  if (rtcAvailable)
  {
    DateTime now = rtc.now();
    Serial.print("  rtc=");
    Serial.print(now.hour());
    Serial.print(':');
    Serial.print(now.minute());
    Serial.print(':');
    Serial.print(now.second());
  }
  Serial.println();
}

// ---------------------------------------------------------------------------------
//                              Background update
// ---------------------------------------------------------------------------------

// Everything that needs to keep running no matter which screen (if any) is
// open: sensor reads, fan control, and the periodic LoRa transmit. Called
// from ui's in-menu callback (idle at a menu) and explicitly inside every
// screen's own while(true) loop below.
void backgroundUpdate()
{
  updateTilt();
  updateTemperatures();
  updateFanControl();

  if (millis() - lastTransmitTime >= TRANSMIT_INTERVAL_MS)
  {
    lastTransmitTime = millis();
    transmitPacket();
  }
}

// ---------------------------------------------------------------------------------
//                          Shared display helpers
// ---------------------------------------------------------------------------------

// Formats a signed tilt value as a direction word + magnitude, e.g.
// "Nose High 2.3 deg" instead of a signed number -- easier to act on at a
// glance. Unit switches to inches once LEVEL_POINT_DISTANCE_INCHES is set.
void formatDirectionalValue(char *buf, size_t bufSize, float value, const char *positiveLabel, const char *negativeLabel)
{
  const char *unit = (LEVEL_POINT_DISTANCE_INCHES > 0) ? "in" : "deg";
  const char *direction = (value >= 0) ? positiveLabel : negativeLabel;
  float magnitude = (value >= 0) ? value : -value;
  snprintf(buf, bufSize, "%s %.1f %s", direction, magnitude, unit);
}

void formatTime(char *buf, size_t bufSize)
{
  if (!rtcAvailable)
  {
    snprintf(buf, bufSize, "no RTC");
    return;
  }
  DateTime now = rtc.now();
  snprintf(buf, bufSize, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
}

// Shared by commandLevel()/commandTemps(): a simple label (left column) +
// value (right column) row layout, label drawn once at screen entry,
// value redrawn every loop iteration via drawInfoValue().
const int INFO_VALUE_COLUMN_X = 140;
const int INFO_LINE_HEIGHT = 24;
int infoFirstLineY = 0;

void drawInfoLabels(const char *labels[], int count)
{
  infoFirstLineY = ui.displaySpaceTopY + 10;
  for (int i = 0; i < count; i++)
  {
    ui.lcdSetCursorXY(ui.displaySpaceLeftX + 6, infoFirstLineY + i * INFO_LINE_HEIGHT);
    ui.lcdPrint(labels[i]);
  }
}

void drawInfoValue(int rowIndex, const char *value)
{
  int y = infoFirstLineY + rowIndex * INFO_LINE_HEIGHT;
  ui.lcdDrawFilledRectangle(INFO_VALUE_COLUMN_X, y, 170, ui.lcdGetFontHeightWithDecenders(), LCD_BLACK);
  ui.lcdSetCursorXY(INFO_VALUE_COLUMN_X, y);
  ui.lcdPrint(value);
}

// ---------------------------------------------------------------------------------
//                              Screen 1: Level
// ---------------------------------------------------------------------------------

void commandLevel(void)
{
  ui.drawTitleBarWithBackButton("Level");
  ui.clearDisplaySpace();

  const char *labels[] = {"Front to Back", "Left/Right", "Time"};
  drawInfoLabels(labels, 3);

  while (true)
  {
    ui.getTouchEvents();
    backgroundUpdate();
    if (ui.checkForBackButtonClicked())
      return;

    char buf[24];
    formatDirectionalValue(buf, sizeof(buf), txPacket.pitch, "Nose High", "Nose Low");
    drawInfoValue(0, buf);
    formatDirectionalValue(buf, sizeof(buf), txPacket.roll, "Left High", "Right High");
    drawInfoValue(1, buf);
    formatTime(buf, sizeof(buf));
    drawInfoValue(2, buf);

    delay(50);
  }
}

// ---------------------------------------------------------------------------------
//                              Screen 2: Temps
// ---------------------------------------------------------------------------------

void commandTemps(void)
{
  ui.drawTitleBarWithBackButton("Temperatures");
  ui.clearDisplaySpace();

  const char *labels[] = {"Fridge", "Freezer", "Inside AS", "DC Cabinet", "Time"};
  drawInfoLabels(labels, 5);

  while (true)
  {
    ui.getTouchEvents();
    backgroundUpdate();
    if (ui.checkForBackButtonClicked())
      return;

    char buf[24];
    snprintf(buf, sizeof(buf), "%.1f F", txPacket.temp1);
    drawInfoValue(0, buf);
    snprintf(buf, sizeof(buf), "%.1f F", txPacket.temp2);
    drawInfoValue(1, buf);
    snprintf(buf, sizeof(buf), "%.1f F", txPacket.temp3);
    drawInfoValue(2, buf);
    snprintf(buf, sizeof(buf), "%.1f F", txPacket.temp4);
    drawInfoValue(3, buf);
    formatTime(buf, sizeof(buf));
    drawInfoValue(4, buf);

    delay(50);
  }
}

// ---------------------------------------------------------------------------------
//                           Screen 3: Fan Control
// ---------------------------------------------------------------------------------

// NOTE on SELECTION_BOX.width: it's the TOTAL width, split evenly across
// all choice cells ((width-3)/numberOfCells per cell, see
// getCoordsOfSelectionBoxCell() in the library) -- NOT the width of one
// cell. A 4-choice box needs real screen width to stay legible, which is
// why Probe gets its own full-width row below rather than sharing a
// 2-column layout with the other 5 widgets.
//
// Also: unused choice slots must be "" (empty string), not NULL --
// countSelectionBoxChoices() checks choice2Text[0]/choice3Text[0], which
// is a null-pointer dereference if those are NULL instead of "".
SELECTION_BOX fanProbeBox = {"Probe", 0, "Fridge", "Freezer", "Inside AS", "DC Cabinet", 0, 0, 0, 0};
SELECTION_BOX fanScheduleBox = {"Schedule", 0, "Disabled", "Enabled", "", "", 0, 0, 0, 0};
NUMBER_BOX_FLOAT fanMinTempBox = {"Min Temp", 70.0, 32.0, 150.0, 1.0, 1, 0, 0, 0, 0};
NUMBER_BOX_FLOAT fanMaxTempBox = {"Max Temp", 90.0, 32.0, 150.0, 1.0, 1, 0, 0, 0, 0};
NUMBER_BOX fanStartHourBox = {"Start Hr", 8, 0, 23, 1, 0, 0, 0, 0};
NUMBER_BOX fanEndHourBox = {"End Hr", 20, 0, 23, 1, 0, 0, 0, 0};

// 4 rows: Probe (full width, needs room for 4 text choices), Schedule
// (full width), then Min/Max Temp and Start/End Hour as 2-column rows
// (NUMBER_BOX degrades gracefully at narrower widths, unlike SELECTION_BOX).
void layoutFanControlWidgets()
{
  // drawSelectionBox()/drawNumberBox() draw the label ABOVE the box itself
  // (see TouchUserInterfaceForArduino.cpp), so each row needs headroom
  // reserved for that label, not just the box height -- query the actual
  // font metrics rather than guessing a fixed pixel value.
  int labelSpace = ui.lcdGetFontHeightWithDecentersAndLineSpacing() + 4;
  int rowHeight = ui.displaySpaceHeight / 4;
  int rowWidgetHeight = rowHeight - labelSpace - 6;  // 6px gap before the next row

  int fullWidth = ui.displaySpaceWidth - 20;
  int fullCenterX = ui.displaySpaceLeftX + ui.displaySpaceWidth / 2;

  int colWidth = ui.displaySpaceWidth / 2;
  int leftX = ui.displaySpaceLeftX + colWidth / 2;
  int rightX = ui.displaySpaceLeftX + colWidth + colWidth / 2;
  int halfWidth = colWidth - 20;

  int row1Y = ui.displaySpaceTopY + labelSpace + rowWidgetHeight / 2;
  int row2Y = ui.displaySpaceTopY + rowHeight + labelSpace + rowWidgetHeight / 2;
  int row3Y = ui.displaySpaceTopY + 2 * rowHeight + labelSpace + rowWidgetHeight / 2;
  int row4Y = ui.displaySpaceTopY + 3 * rowHeight + labelSpace + rowWidgetHeight / 2;

  fanProbeBox.centerX = fullCenterX;
  fanProbeBox.centerY = row1Y;
  fanProbeBox.width = fullWidth;
  fanProbeBox.height = rowWidgetHeight;

  fanScheduleBox.centerX = fullCenterX;
  fanScheduleBox.centerY = row2Y;
  fanScheduleBox.width = fullWidth;
  fanScheduleBox.height = rowWidgetHeight;

  fanMinTempBox.centerX = leftX;
  fanMinTempBox.centerY = row3Y;
  fanMinTempBox.width = halfWidth;
  fanMinTempBox.height = rowWidgetHeight;

  fanMaxTempBox.centerX = rightX;
  fanMaxTempBox.centerY = row3Y;
  fanMaxTempBox.width = halfWidth;
  fanMaxTempBox.height = rowWidgetHeight;

  fanStartHourBox.centerX = leftX;
  fanStartHourBox.centerY = row4Y;
  fanStartHourBox.width = halfWidth;
  fanStartHourBox.height = rowWidgetHeight;

  fanEndHourBox.centerX = rightX;
  fanEndHourBox.centerY = row4Y;
  fanEndHourBox.width = halfWidth;
  fanEndHourBox.height = rowWidgetHeight;
}

void drawFanControlTitleBar()
{
  char title[24];
  snprintf(title, sizeof(title), "Fan Control - %s", fanOn ? "ON" : "OFF");
  ui.drawTitleBarWithBackButton(title);
}

void commandFanControl(void)
{
  drawFanControlTitleBar();
  ui.clearDisplaySpace();
  layoutFanControlWidgets();

  fanProbeBox.value = fanProbeIndex;
  fanScheduleBox.value = fanTimeEnabled;
  fanMinTempBox.value = fanMinTemp;
  fanMaxTempBox.value = fanMaxTemp;
  fanStartHourBox.value = fanStartHour;
  fanEndHourBox.value = fanEndHour;

  ui.drawSelectionBox(fanProbeBox);
  ui.drawSelectionBox(fanScheduleBox);
  ui.drawNumberBox(fanMinTempBox);
  ui.drawNumberBox(fanMaxTempBox);
  ui.drawNumberBox(fanStartHourBox);
  ui.drawNumberBox(fanEndHourBox);

  bool lastFanOn = fanOn;

  while (true)
  {
    ui.getTouchEvents();
    backgroundUpdate();
    if (ui.checkForBackButtonClicked())
      return;

    if (ui.checkForSelectionBoxTouched(fanProbeBox))
    {
      fanProbeIndex = fanProbeBox.value;
      ui.writeConfigurationInt(EEPROM_ADDR_FAN_PROBE, fanProbeIndex);
    }
    if (ui.checkForSelectionBoxTouched(fanScheduleBox))
    {
      fanTimeEnabled = fanScheduleBox.value;
      ui.writeConfigurationInt(EEPROM_ADDR_FAN_TIME_ENABLE, fanTimeEnabled);
    }
    if (ui.checkForNumberBoxTouched(fanMinTempBox))
    {
      fanMinTemp = fanMinTempBox.value;
      ui.writeConfigurationFloat(EEPROM_ADDR_FAN_MIN_TEMP, fanMinTemp);
    }
    if (ui.checkForNumberBoxTouched(fanMaxTempBox))
    {
      fanMaxTemp = fanMaxTempBox.value;
      ui.writeConfigurationFloat(EEPROM_ADDR_FAN_MAX_TEMP, fanMaxTemp);
    }
    if (ui.checkForNumberBoxTouched(fanStartHourBox))
    {
      fanStartHour = fanStartHourBox.value;
      ui.writeConfigurationInt(EEPROM_ADDR_FAN_START_HOUR, fanStartHour);
    }
    if (ui.checkForNumberBoxTouched(fanEndHourBox))
    {
      fanEndHour = fanEndHourBox.value;
      ui.writeConfigurationInt(EEPROM_ADDR_FAN_END_HOUR, fanEndHour);
    }

    // Relay state can change purely from temperature drifting (not just
    // touch input), since backgroundUpdate() -> updateFanControl() runs
    // every iteration -- keep the title bar in sync either way.
    if (fanOn != lastFanOn)
    {
      lastFanOn = fanOn;
      drawFanControlTitleBar();
    }

    delay(50);
  }
}

// ---------------------------------------------------------------------------------
//                             Screen 4: Settings
// ---------------------------------------------------------------------------------

NUMBER_BOX yearBox = {"Year", 2026, 2020, 2099, 1, 0, 0, 0, 0};
NUMBER_BOX monthBox = {"Month", 1, 1, 12, 1, 0, 0, 0, 0};
NUMBER_BOX dayBox = {"Day", 1, 1, 31, 1, 0, 0, 0, 0};
NUMBER_BOX hourBox = {"Hour", 0, 0, 23, 1, 0, 0, 0, 0};
NUMBER_BOX minuteBox = {"Minute", 0, 0, 59, 1, 0, 0, 0, 0};
BUTTON saveTimeButton = {"Save Time", 0, 0, 0, 0};
BUTTON zeroLevelButton = {"Zero Level", 0, 0, 0, 0};

// 2-column x 4-row grid: Year/Month, Day/Hour, Minute/(empty), Save Time/Zero Level.
// NUMBER_BOX draws its label above the box (BUTTON draws its label inside),
// so headroom is reserved for every row for simplicity -- harmless extra
// gap above the row 4 buttons, but avoids the rows 1-3 label/box overlap
// this exact mistake caused on the Fan Control screen.
void layoutSettingsWidgets()
{
  int labelSpace = ui.lcdGetFontHeightWithDecentersAndLineSpacing() + 4;
  int colWidth = ui.displaySpaceWidth / 2;
  int rowHeight = ui.displaySpaceHeight / 4;
  int leftX = ui.displaySpaceLeftX + colWidth / 2;
  int rightX = ui.displaySpaceLeftX + colWidth + colWidth / 2;
  int widgetWidth = colWidth - 20;
  int widgetHeight = rowHeight - labelSpace - 6;

  int row1Y = ui.displaySpaceTopY + labelSpace + widgetHeight / 2;
  int row2Y = ui.displaySpaceTopY + rowHeight + labelSpace + widgetHeight / 2;
  int row3Y = ui.displaySpaceTopY + 2 * rowHeight + labelSpace + widgetHeight / 2;
  int row4Y = ui.displaySpaceTopY + 3 * rowHeight + labelSpace + widgetHeight / 2;

  yearBox.centerX = leftX;
  yearBox.centerY = row1Y;
  yearBox.width = widgetWidth;
  yearBox.height = widgetHeight;

  monthBox.centerX = rightX;
  monthBox.centerY = row1Y;
  monthBox.width = widgetWidth;
  monthBox.height = widgetHeight;

  dayBox.centerX = leftX;
  dayBox.centerY = row2Y;
  dayBox.width = widgetWidth;
  dayBox.height = widgetHeight;

  hourBox.centerX = rightX;
  hourBox.centerY = row2Y;
  hourBox.width = widgetWidth;
  hourBox.height = widgetHeight;

  minuteBox.centerX = leftX;
  minuteBox.centerY = row3Y;
  minuteBox.width = widgetWidth;
  minuteBox.height = widgetHeight;

  saveTimeButton.centerX = leftX;
  saveTimeButton.centerY = row4Y;
  saveTimeButton.width = widgetWidth;
  saveTimeButton.height = widgetHeight;

  zeroLevelButton.centerX = rightX;
  zeroLevelButton.centerY = row4Y;
  zeroLevelButton.width = widgetWidth;
  zeroLevelButton.height = widgetHeight;
}

void commandSettings(void)
{
  ui.drawTitleBarWithBackButton("Settings");
  ui.clearDisplaySpace();
  layoutSettingsWidgets();

  if (rtcAvailable)
  {
    DateTime now = rtc.now();
    yearBox.value = now.year();
    monthBox.value = now.month();
    dayBox.value = now.day();
    hourBox.value = now.hour();
    minuteBox.value = now.minute();
  }

  ui.drawNumberBox(yearBox);
  ui.drawNumberBox(monthBox);
  ui.drawNumberBox(dayBox);
  ui.drawNumberBox(hourBox);
  ui.drawNumberBox(minuteBox);
  ui.drawButton(saveTimeButton);
  ui.drawButton(zeroLevelButton);

  while (true)
  {
    ui.getTouchEvents();
    backgroundUpdate();
    if (ui.checkForBackButtonClicked())
      return;

    ui.checkForNumberBoxTouched(yearBox);
    ui.checkForNumberBoxTouched(monthBox);
    ui.checkForNumberBoxTouched(dayBox);
    ui.checkForNumberBoxTouched(hourBox);
    ui.checkForNumberBoxTouched(minuteBox);

    if (ui.checkForButtonClicked(saveTimeButton) && rtcAvailable)
    {
      rtc.adjust(DateTime(yearBox.value, monthBox.value, dayBox.value, hourBox.value, minuteBox.value, 0));
      Serial.println("RTC time saved from Settings screen");
    }

    if (ui.checkForButtonClicked(zeroLevelButton))
    {
      pitchOffset += txPacket.pitch;
      rollOffset += txPacket.roll;
      ui.writeConfigurationFloat(EEPROM_ADDR_PITCH_OFFSET, pitchOffset);
      ui.writeConfigurationFloat(EEPROM_ADDR_ROLL_OFFSET, rollOffset);
    }

    delay(50);
  }
}
