//      ******************************************************************
//      *                                                                *
//      *      Airstream v2.1 board -- tilt, temps, LoRa, touch UI       *
//      *                                                                *
//      ******************************************************************

// Last updated: 2026-07-31 21:03 PDT

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
#include <nvs_flash.h>
#include <nvs.h>
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

// Sensors. TEMP1/TEMP2 are each a DS18B20 (BOJACK 1M stainless probe) on
// its own OneWire bus, with the schematic's 3.3k resistor acting as the
// OneWire pull-up (matches the original code's OneWire-based approach on
// these same physical pins, just swapped between TEMP1/TEMP2).
// TEMP3 (GPIO36) and TEMP4 (GPIO39) were removed -- both are input-only
// pins on the ESP32 (no output driver, can't pull the OneWire bus low), so
// a DS18B20 wired to either one can never work. Confirmed on hardware: the
// probes worked fine on TEMP1/TEMP2 but not once moved to TEMP3/TEMP4.
const int TEMP1_PIN = 13;  // Fridge
const int TEMP2_PIN = 25;  // DC Cabinet, battery bank + inverter
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

// Distance (inches) between the trailer's leveling points, used to convert
// tilt angle to how many inches to raise/lower one end -- inches = tan(angle)
// * distance, valid for a rigid body regardless of where on it the sensor
// is mounted (the tilt angle is the same everywhere on the frame; only the
// horizontal distance between the two points being leveled matters). Either
// can be set to 0 to fall back to showing degrees for that axis.
//
// PITCH (nose up/down, front-to-back level): tongue jack to the center of
// the (dual) axles, 16ft measured on the trailer.
const float LEVEL_PITCH_DISTANCE_INCHES = 16.0 * 12.0;  // 192 in
// ROLL (left/right level): trailer width, 8ft measured on the trailer.
const float LEVEL_ROLL_DISTANCE_INCHES = 8.0 * 12.0;  // 96 in

// EEPROM addresses (see TouchUserInterfaceForArduino's writeConfigurationFloat/Int --
// each value uses 5 bytes, so keep these at least 5 apart; using 10 for headroom).
const int EEPROM_ADDR_PITCH_OFFSET = 0;
const int EEPROM_ADDR_ROLL_OFFSET = 10;
const int EEPROM_ADDR_FAN_PROBE = 20;        // int: 0=Fridge,1=DC Cabinet
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
bool mpuAvailable = false;
RTC_PCF8523 rtc;
bool rtcAvailable = false;
TouchUserInterfaceForArduino ui;

OneWire oneWireTemp1(TEMP1_PIN);
OneWire oneWireTemp2(TEMP2_PIN);
DallasTemperature dsTemp1(&oneWireTemp1);
DallasTemperature dsTemp2(&oneWireTemp2);

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

// Raw tilt angle in degrees, before the tan()*distance inches conversion --
// "Zero Level" needs to zero out the actual angle, not the already-converted
// inches value in txPacket.pitch/roll (a real bug once LEVEL_PITCH/ROLL_
// DISTANCE_INCHES are set: pitchOffset is subtracted from a degrees value
// inside updateTilt(), so it must stay in degrees itself).
float lastPitchDegrees = 0;
float lastRollDegrees = 0;

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
const char *FAN_PROBE_NAMES[2] = {"Fridge", "DC Cabinet"};
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

  // NVS recovery: EEPROM.begin() below reads through esp-idf's NVS layer.
  // A crash here (LoadProhibited inside nvs_get_blob/Item::Item, seen even
  // after a full flash erase) means the NVS partition itself came up
  // corrupt/mismatched -- the Arduino core's automatic nvs_flash_init() at
  // boot doesn't self-heal from that, it just leaves it broken. Detect and
  // force-erase+reinit here so EEPROM.begin() always sees a clean partition.
  esp_err_t nvsErr = nvs_flash_init();
  if (nvsErr == ESP_ERR_NVS_NO_FREE_PAGES || nvsErr == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    Serial.println("NVS partition corrupt/mismatched -- erasing and reinitializing");
    nvs_flash_erase();
    nvsErr = nvs_flash_init();
  }
  if (nvsErr != ESP_OK)
  {
    Serial.print("nvs_flash_init() failed, err=");
    Serial.println(nvsErr);
  }

  // Work around a crash in this core's EEPROM library (esp32 3.3.8): when the
  // "eeprom" NVS blob doesn't exist AT ALL yet (key_size == 0), begin()'s
  // "brand new" path -- write a throwaway "expand" key, erase it, write the
  // real "eeprom" blob, then immediately read it back -- crashes at that
  // read-back (EEPROM.cpp:117, nvs_get_blob -> Item::Item -> strncpy) with
  // LoadProhibited. Reproduced even after a full chip erase, so it's not
  // stale data -- it's this specific from-scratch path. Resizing a blob that
  // ALREADY exists (grow or shrink) goes through a different, confirmed-safe
  // path -- e.g. TouchUserInterfaceForArduino's own writeConfigurationFloat/
  // Int calls EEPROM.begin(1024) internally (its own hardcoded size, see
  // TouchUserInterfaceForArduino.cpp EEPROM_SIZE), silently regrowing this
  // same blob from 128 to 1024 bytes the first time any config is saved.
  // So only pre-create the blob when it's truly absent -- never touch it
  // (and never wipe it) if it already exists at some other size, or the
  // library's own resize on the NEXT boot would look like a fresh key again
  // and get wiped here, discarding whatever was saved (this bit us: Zero
  // Level calibration wasn't surviving a power cycle because of exactly
  // this).
  {
    nvs_handle_t rawHandle;
    if (nvs_open("eeprom", NVS_READWRITE, &rawHandle) == ESP_OK)
    {
      size_t existingSize = 0;
      esp_err_t sizeErr = nvs_get_blob(rawHandle, "eeprom", NULL, &existingSize);
      if (sizeErr == ESP_ERR_NVS_NOT_FOUND)
      {
        uint8_t blank[128];
        memset(blank, 0xFF, sizeof(blank));
        esp_err_t setErr = nvs_set_blob(rawHandle, "eeprom", blank, sizeof(blank));
        nvs_commit(rawHandle);
        Serial.print("eeprom NVS blob didn't exist -- created at 128 bytes, err=");
        Serial.println(setErr);
      }
      nvs_close(rawHandle);
    }
  }

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

// Prints every I2C address that ACKs, or "(nothing responded)" if none do.
// Diagnostic for the cold-power-cycle MPU6050 init failures -- shows
// directly whether NOTHING is on the bus yet (bus-wide power/pull-up
// issue) or whether e.g. the RTC (fixed 0x68) responds while the MPU6050
// (0x69, which depends on the ADO jumper reading a stable HIGH) doesn't.
void scanI2CBus()
{
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("  I2C device responding at 0x");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0)
  {
    Serial.println("  I2C scan: nothing responded");
  }
}

// If a previous I2C transaction was interrupted mid-stream (e.g. by the
// known esp32 core 3.3.8 I2C driver crash -- espressif/arduino-esp32#11374,
// hit in this project during first-read-after-boot MPU6050 access), a
// slave device can be left holding SDA low, waiting for clock pulses it'll
// never get. That wedges the bus for every future transaction --
// Wire.begin() only resets the ESP32's own I2C peripheral, not an external
// device still holding the line, which is why recovering from that crash
// previously required physically power-cycling the RTC/MPU6050 and why a
// second boot attempt right after a crash hung identically instead of
// retrying cleanly. Manually pulse SCL as a plain GPIO before Wire claims
// the pins -- the standard software recovery for a stuck I2C slave -- then
// issue a manual STOP condition, so the crash's own auto-reboot has an
// actual chance to succeed instead of needing physical intervention.
void recoverStuckI2CBus()
{
  pinMode(SCL_PIN, OUTPUT);
  pinMode(SDA_PIN, INPUT_PULLUP);

  for (int i = 0; i < 9; i++)
  {
    if (digitalRead(SDA_PIN) == HIGH)
      break;  // slave already released the bus
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
  }

  // Manual STOP condition: SDA low-to-high while SCL is high.
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(5);
}

void setupTiltSensor()
{
  recoverStuckI2CBus();
  Wire.begin(SDA_PIN, SCL_PIN);
  // Explicit, conservative clock -- esp32 core 3.3.8's overhauled I2C
  // driver has documented crash/instability reports on reads at default
  // settings (espressif/arduino-esp32#11374); this is the first thing worth
  // ruling out rather than assuming whatever the core's own default is.
  Wire.setClock(100000);

  // mpu.begin() was found to fail on EVERY attempt within a ~1s retry
  // window after a true power cycle (not just the first one or two),
  // which rules out "just needs a brief settling delay" as the whole
  // story -- works fine after a USB reflash (board power never actually
  // drops during that), fails every time after a real power-off/on.
  // Extended the retry window further and added I2C bus scans along the
  // way to see directly what's actually responding during a failed cold
  // boot, rather than guessing blindly again.
  const int MPU_INIT_MAX_ATTEMPTS = 15;
  const unsigned long MPU_INIT_RETRY_DELAY_MS = 300;
  bool mpuOk = false;

  for (int attempt = 1; attempt <= MPU_INIT_MAX_ATTEMPTS && !mpuOk; attempt++)
  {
    if (attempt == 1 || attempt % 5 == 0)
    {
      Serial.print("I2C scan at MPU6050 init attempt ");
      Serial.print(attempt);
      Serial.println(":");
      scanI2CBus();
    }

    mpuOk = mpu.begin(MPU6050_ADDRESS, &Wire);
    if (!mpuOk)
    {
      // scanI2CBus() above already shows 0x69 ACKs a bare address probe, so
      // begin() must be failing a deeper check -- almost certainly the
      // WHO_AM_I register readback it gates on (expects 0x68). Read that
      // register directly, bypassing Adafruit_MPU6050::begin(), to see
      // whether it's coming back wrong (real chip, bad data) or not
      // responding at all to an actual register read (vs. just an empty
      // address probe).
      Wire.beginTransmission(MPU6050_ADDRESS);
      Wire.write(0x75);  // WHO_AM_I
      uint8_t writeErr = Wire.endTransmission(false);
      uint8_t whoAmI = 0xFF;
      if (Wire.requestFrom((uint8_t)MPU6050_ADDRESS, (uint8_t)1) == 1)
        whoAmI = Wire.read();
      Serial.print("  WHO_AM_I read: 0x");
      Serial.print(whoAmI, HEX);
      Serial.print(" (expect 0x68), endTransmission code ");
      Serial.println(writeErr);

      Serial.print("MPU6050 init attempt ");
      Serial.print(attempt);
      Serial.println(" failed, retrying...");
      delay(MPU_INIT_RETRY_DELAY_MS);
    }
  }

  mpuAvailable = mpuOk;

  if (!mpuOk)
  {
    Serial.println("MPU6050 not found at 0x69 after retries -- check the ADO jumper and wiring");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // The crash seen right after a fresh boot (Guru Meditation Error,
  // IllegalInstruction, deep in esp-idf's i2c_master driver) happens on the
  // FIRST getEvent() burst-read in the main loop -- not during any of the
  // single-register writes above, which all succeed fine (this function
  // finishes and prints "Tilt sensor init done" before the crash). A short
  // settle here, after begin()/config succeed but before the main loop
  // starts hammering the bus with real reads, is a cheap way to reduce how
  // often that race gets hit.
  if (mpuOk)
    delay(100);
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

  // Log the raw booleans directly -- found (2026-07-31) that a genuinely
  // never-set PCF8523 can report initialized()=true, lostPower()=false
  // (this check's fallback below never fires) while still returning
  // garbage like month=0/day=0 from now(), so seeing these values directly
  // matters more than inferring them from which branch ran.
  Serial.print("RTC initialized()=");
  Serial.print(rtc.initialized());
  Serial.print(" lostPower()=");
  Serial.println(rtc.lostPower());

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

  // Belt-and-suspenders on top of the initialized()/lostPower() check above
  // -- confirmed on hardware (2026-07-31) that check can miss a genuinely
  // never-set RTC (month=0/day=0, neither a valid calendar value) on this
  // module. Catch that case directly and force the same compile-time
  // fallback rather than letting an obviously-impossible date through.
  if (now.month() < 1 || now.month() > 12 || now.day() < 1 || now.day() > 31)
  {
    Serial.println("RTC returned an impossible date despite initialized()/lostPower() -- forcing compile-time fallback");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    now = rtc.now();
  }

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

  // non-blocking mode: requestTemperatures() returns immediately, we poll
  // for the result ourselves after TEMP_CONVERSION_TIME_MS (see updateTemperatures)
  dsTemp1.setWaitForConversion(false);
  dsTemp2.setWaitForConversion(false);
}

// TouchUserInterfaceForArduino's ESP32 writeConfigurationInt/Float
// (TouchUserInterfaceForArduino.cpp, #if !defined(ARDUINO_ARCH_RP2040))
// call EEPROM.write() per byte but -- unlike the RP2040 branch of the same
// functions -- never call EEPROM.commit(). That means a "saved" value only
// ever reaches EEPROM's in-RAM buffer; it's never pushed to nvs_set_blob()
// at all, let alone made durable. This is why Zero Level and fan settings
// weren't sticking across a power cycle. Call EEPROM.commit() ourselves to
// actually stage the write, then nvs_commit() directly for good measure
// (nvs_set_blob() alone isn't documented as durable until nvs_commit() is
// called -- see esp-idf's nvs.h). Call this once after any
// ui.writeConfigurationInt/Float call.
void flushEepromToFlash()
{
  EEPROM.commit();
  nvs_handle_t h;
  if (nvs_open("eeprom", NVS_READWRITE, &h) == ESP_OK)
  {
    nvs_commit(h);
    nvs_close(h);
  }
}

void loadFanSettings()
{
  fanProbeIndex = ui.readConfigurationInt(EEPROM_ADDR_FAN_PROBE, 0);
  // Clamp in case a board still has 2 (Inside AS) or 3 (DC Cabinet) stored
  // from before those probes were removed.
  if (fanProbeIndex < 0 || fanProbeIndex > 1)
    fanProbeIndex = 0;
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

// Wraps an angle into (-180, 180] -- see the note in updateTilt() about why
// this is needed on top of atan2's own bounded output.
float normalizeAngle180(float angle)
{
  while (angle > 180.0)
    angle -= 360.0;
  while (angle <= -180.0)
    angle += 360.0;
  return angle;
}

void updateTilt()
{
  // Make a never-initialized sensor obvious (NAN, same pattern as a
  // disconnected DS18B20 probe) rather than silently displaying whatever
  // getEvent() happens to return from an object that never successfully
  // began -- previously this looked like a plausible but frozen reading.
  if (!mpuAvailable)
  {
    txPacket.pitch = NAN;
    txPacket.roll = NAN;
    return;
  }

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

  // ROLL_FROM_AXES's "- 90.0" shifts atan2's naturally-bounded (-180,180]
  // output to (-270,90] -- not symmetric, and subtracting rollOffset can
  // push it further outside a sane range (e.g. "252 degrees" instead of a
  // small number near the board's resting orientation, which sits close to
  // atan2's wraparound point). Normalize both angles back to (-180,180]
  // after the offset so the UI always shows a believable tilt value.
  float pitchDegrees = normalizeAngle180(PITCH_SIGN * PITCH_FROM_AXES(axAvg, ayAvg, azAvg) - pitchOffset);
  float rollDegrees = normalizeAngle180(ROLL_SIGN * ROLL_FROM_AXES(axAvg, ayAvg, azAvg) - rollOffset);
  lastPitchDegrees = pitchDegrees;
  lastRollDegrees = rollDegrees;

  txPacket.pitch = (LEVEL_PITCH_DISTANCE_INCHES > 0)
    ? tan(pitchDegrees * PI / 180.0) * LEVEL_PITCH_DISTANCE_INCHES
    : pitchDegrees;
  txPacket.roll = (LEVEL_ROLL_DISTANCE_INCHES > 0)
    ? tan(rollDegrees * PI / 180.0) * LEVEL_ROLL_DISTANCE_INCHES
    : rollDegrees;
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

// Non-blocking: kicks off a conversion on both probes every
// TEMP_UPDATE_INTERVAL_MS, then collects the results once conversion time
// has elapsed. Does not delay() -- safe to call every loop iteration.
void updateTemperatures()
{
  unsigned long now = millis();

  if (tempReadState == TEMP_IDLE && now - lastTempUpdateTime >= TEMP_UPDATE_INTERVAL_MS)
  {
    dsTemp1.requestTemperatures();
    dsTemp2.requestTemperatures();
    tempConversionStartTime = now;
    tempReadState = TEMP_CONVERTING;
  }
  else if (tempReadState == TEMP_CONVERTING && now - tempConversionStartTime >= TEMP_CONVERSION_TIME_MS)
  {
    txPacket.temp1 = readDS18B20(dsTemp1);
    txPacket.temp2 = readDS18B20(dsTemp2);
    lastTempUpdateTime = now;
    tempReadState = TEMP_IDLE;
  }
}

// ---------------------------------------------------------------------------------
//                                  Fan control
// ---------------------------------------------------------------------------------

float getFanProbeTemp()
{
  return (fanProbeIndex == 0) ? txPacket.temp1 : txPacket.temp2;
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
  // Found via the backgroundUpdate() timing diagnostic: endPacket()'s
  // default is BLOCKING -- it doesn't return until the packet has fully
  // gone out over the air (~750ms at this radio's spreading factor/
  // bandwidth), freezing the whole loop (touch polling included) for over
  // a third of every 2-second transmit interval. That's not new behavior,
  // just newly noticeable now that there's a lot more screen navigation
  // than the original single-screen UI. endPacket(true) is async -- it
  // starts the transmission and returns immediately; the radio finishes it
  // in the background well within the 2-second gap before the next call.
  // Fill in the date/time and fan status fields right before sending --
  // pitch/roll/temp1-4 are already kept current by updateTilt()/
  // updateTemperatures() every loop iteration, but these don't have a
  // dedicated "owner" function, so gather them here instead.
  if (rtcAvailable)
  {
    DateTime now = rtc.now();
    txPacket.year = now.year();
    txPacket.month = now.month();
    txPacket.day = now.day();
    txPacket.hour = now.hour();
    txPacket.minute = now.minute();
  }
  else
  {
    txPacket.year = 0;
    txPacket.month = 0;
    txPacket.day = 0;
    txPacket.hour = 0;
    txPacket.minute = 0;
  }
  txPacket.fanOn = fanOn ? 1 : 0;
  txPacket.fanProbeIndex = fanProbeIndex;
  txPacket.fanOffTemp = fanMinTemp;
  txPacket.fanOnTemp = fanMaxTemp;
  txPacket.fanStartHour = fanStartHour;
  txPacket.fanEndHour = fanEndHour;

  LoRa.beginPacket();
  LoRa.write((uint8_t *)&txPacket, sizeof(txPacket));
  LoRa.endPacket(true);

  // Diagnostic for tracking down the TowVehicle-V3 date/time field
  // corruption -- hex dump of exactly what's going out over the air, byte
  // for byte, so it can be compared against what the receiver actually
  // reads. Remove once that's resolved.
  Serial.print("TX raw bytes (");
  Serial.print(sizeof(txPacket));
  Serial.print("): ");
  {
    uint8_t *raw = (uint8_t *)&txPacket;
    for (size_t i = 0; i < sizeof(txPacket); i++)
    {
      if (raw[i] < 0x10)
        Serial.print('0');
      Serial.print(raw[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
  }

  Serial.print("TX  pitch=");
  Serial.print(txPacket.pitch, 1);
  Serial.print("  roll=");
  Serial.print(txPacket.roll, 1);
  Serial.print("  t1=");
  Serial.print(txPacket.temp1, 1);
  Serial.print("  t2=");
  Serial.print(txPacket.temp2, 1);
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
  // Diagnostic for the "everything feels slower" report -- this runs on
  // every loop iteration of every screen, so timing it here will catch
  // whatever's actually slow (sensor read, transmit, RTC I2C) without
  // spamming Serial during normal-speed iterations.
  unsigned long startMs = millis();

  updateTilt();
  updateTemperatures();
  updateFanControl();

  if (millis() - lastTransmitTime >= TRANSMIT_INTERVAL_MS)
  {
    lastTransmitTime = millis();
    transmitPacket();
  }

  unsigned long elapsedMs = millis() - startMs;
  if (elapsedMs > 100)
  {
    Serial.print("backgroundUpdate() took ");
    Serial.print(elapsedMs);
    Serial.println(" ms (slow)");
  }
}

// ---------------------------------------------------------------------------------
//                          Shared display helpers
// ---------------------------------------------------------------------------------

// Shared 12-hour <-> 24-hour conversion, used by both the Set Time screen
// and Schedule Fan Control's Start/End Hr -- storage (EEPROM, RTC) and the
// schedule-window comparison in updateFanControl() all stay in 24-hour;
// only the on-screen entry widgets are 12-hour + AM/PM.
int hour12From24(int hour24)
{
  int hour12 = hour24 % 12;
  return (hour12 == 0) ? 12 : hour12;
}

int ampmFrom24(int hour24)
{
  return (hour24 >= 12) ? 1 : 0;
}

int hour24From12AndAmPm(int hour12, int ampm)
{
  int hour24 = hour12 % 12;
  if (ampm == 1)
    hour24 += 12;
  return hour24;
}

// Formats a signed tilt value as a direction word + magnitude, e.g.
// "Nose High 2.3 in" instead of a signed number -- easier to act on at a
// glance. unit is "in" or "deg" depending on whether the caller's axis has
// its LEVEL_PITCH/ROLL_DISTANCE_INCHES set (see updateTilt()).
void formatDirectionalValue(char *buf, size_t bufSize, float value, const char *positiveLabel, const char *negativeLabel, const char *unit)
{
  const char *direction = (value >= 0) ? positiveLabel : negativeLabel;
  float magnitude = (value >= 0) ? value : -value;
  snprintf(buf, bufSize, "%s %.1f %s", direction, magnitude, unit);
}

// 12-hour clock with AM/PM, matching the Set Time screen's entry format,
// plus the date (MM/DD/YYYY) since Level/Temps have the room for it.
void formatDateTime(char *buf, size_t bufSize)
{
  if (!rtcAvailable)
  {
    snprintf(buf, bufSize, "no RTC");
    return;
  }
  DateTime now = rtc.now();
  int hour24 = now.hour();
  const char *ampm = (hour24 >= 12) ? "PM" : "AM";
  int hour12 = hour24 % 12;
  if (hour12 == 0)
    hour12 = 12;
  snprintf(buf, bufSize, "%02d/%02d/%04d %d:%02d %s", now.month(), now.day(), now.year(), hour12, now.minute(), ampm);
}

// Shared by commandLevel()/commandTemps(): a simple label (left column) +
// value (right column) row layout, label drawn once at screen entry,
// value redrawn every loop iteration via drawInfoValue(). Line height and
// value column position are computed fresh each time from the CURRENT font
// (caller must ui.lcdSetFont(...) before calling) and row count, so each
// screen can use as large a font as its row count leaves room for.
int infoFirstLineY = 0;
int infoLineHeight = 24;
int infoValueColumnX = 140;

void drawInfoLabels(const char *labels[], int count, int lineHeight)
{
  infoLineHeight = lineHeight;
  infoFirstLineY = ui.displaySpaceTopY + 10;

  int maxLabelWidth = 0;
  for (int i = 0; i < count; i++)
  {
    int w = ui.lcdStringWidthInPixels(labels[i]);
    if (w > maxLabelWidth)
      maxLabelWidth = w;
  }
  infoValueColumnX = ui.displaySpaceLeftX + 6 + maxLabelWidth + 8;

  for (int i = 0; i < count; i++)
  {
    ui.lcdSetCursorXY(ui.displaySpaceLeftX + 6, infoFirstLineY + i * infoLineHeight);
    ui.lcdPrint(labels[i]);
  }
}

void drawInfoValue(int rowIndex, const char *value)
{
  int y = infoFirstLineY + rowIndex * infoLineHeight;
  int eraseWidth = ui.displaySpaceLeftX + ui.displaySpaceWidth - infoValueColumnX - 6;
  ui.lcdDrawFilledRectangle(infoValueColumnX, y, eraseWidth, ui.lcdGetFontHeightWithDecenders(), LCD_BLUE);
  ui.lcdSetCursorXY(infoValueColumnX, y);
  ui.lcdPrint(value);
}

// ---------------------------------------------------------------------------------
//                              Screen 1: Level
// ---------------------------------------------------------------------------------

void commandLevel(void)
{
  // Title bar font is a library-wide setting (used by every screen's title
  // and the main menu), so bump it just for this screen and restore the
  // default before returning -- otherwise it'd leak into whatever screen
  // comes next.
  ui.setTitleBarFont(UI_Font_16_Bold);
  ui.drawTitleBarWithBackButton("Level Status");
  ui.clearDisplaySpace(LCD_BLUE);

  // Only 3 rows on this screen -- plenty of vertical room, so use the
  // largest available font.
  ui.lcdSetFont(UI_Font_16_Bold);
  ui.lcdSetFontColor(LCD_WHITE);
  const char *labels[] = {"Front/Back", "Left/Right", "Date/Time"};
  drawInfoLabels(labels, 3, ui.displaySpaceHeight / 3);

  while (true)
  {
    ui.getTouchEvents();
    backgroundUpdate();
    if (ui.checkForBackButtonClicked())
    {
      ui.setTitleBarFont(UI_Font_13_Bold);
      return;
    }

    char buf[24];
    formatDirectionalValue(buf, sizeof(buf), txPacket.pitch, "Nose High", "Nose Low",
      (LEVEL_PITCH_DISTANCE_INCHES > 0) ? "in" : "deg");
    drawInfoValue(0, buf);
    // Roll sign here describes the LOW side, since that's the one that
    // needs a block/shim under it -- you can't lower the high side.
    formatDirectionalValue(buf, sizeof(buf), txPacket.roll, "Right Low", "Left Low",
      (LEVEL_ROLL_DISTANCE_INCHES > 0) ? "in" : "deg");
    drawInfoValue(1, buf);
    formatDateTime(buf, sizeof(buf));
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
  ui.clearDisplaySpace(LCD_BLUE);

  ui.lcdSetFont(UI_Font_16_Bold);
  ui.lcdSetFontColor(LCD_WHITE);
  const char *labels[] = {"Fridge", "DC Cabinet", "Date/Time"};
  drawInfoLabels(labels, 3, ui.displaySpaceHeight / 3);

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
    formatDateTime(buf, sizeof(buf));
    drawInfoValue(2, buf);

    delay(50);
  }
}

// ---------------------------------------------------------------------------------
//                           Screen 3: Fan Control
// ---------------------------------------------------------------------------------
//
// Split across 4 screens so each has room for its own bigger back button:
// Fan Control (this) has the Schedule selector + links to Probe, Temp Range,
// and Schedule Hours; those are their own screens with their own back
// button (returning here). Forward-declared since they're only referenced
// from commandFanControl() below, which is defined first.
void commandFanProbe(void);
void commandFanTempRange(void);
void commandFanScheduleHours(void);

SELECTION_BOX fanScheduleBox = {"Schedule", 0, "Disabled", "Enabled", "", "", 0, 0, 0, 0};
BUTTON fanProbeLinkButton = {"Select Temperature Probe", 0, 0, 0, 0};
BUTTON fanTempRangeLinkButton = {"Temp Range", 0, 0, 0, 0};
BUTTON fanScheduleHoursLinkButton = {"Schedule Hours", 0, 0, 0, 0};

// Draws a yellow border around the selected cell of a SELECTION_BOX,
// matching the highlight style used on the Probe screen's buttons -- the
// library's own drawSelectionBoxCell() only tints the selected cell's fill
// color, not distinct enough at a glance. Replicates the cell-geometry math
// from getCoordsOfSelectionBoxCell() (private in the library) since it only
// needs box.width/numberOfCells -- see the SELECTION_BOX.width note above.
void drawSelectionBoxHighlight(SELECTION_BOX &box, int numberOfCells)
{
  int cellWidth = (box.width - 3) / numberOfCells;
  int overallWidth = cellWidth * numberOfCells;
  int x = (box.centerX - overallWidth / 2) + (box.value * cellWidth);
  int y = box.centerY - (box.height - 3) / 2;
  int h = box.height - 3;
  ui.lcdDrawRectangle(x, y, cellWidth, h, LCD_YELLOW);
  ui.lcdDrawRectangle(x + 1, y + 1, cellWidth - 2, h - 2, LCD_YELLOW);
}

// 4 rows: Schedule (full width, only 2 choices so it fits fine inline),
// then Probe link, then the two range/hours link buttons. Probe is its own
// screen (see commandFanProbe() below) rather than an inline SELECTION_BOX
// here, left as-is from when there were 4 probe choices (now just Fridge/
// DC Cabinet, but the dedicated screen still works fine).
void layoutFanControlWidgets()
{
  // drawSelectionBox() draws the label ABOVE the box itself (see
  // TouchUserInterfaceForArduino.cpp), so the selector row needs headroom
  // reserved for that label -- query the actual font metrics rather than
  // guessing a fixed pixel value. BUTTON draws its label inside, so the
  // link-button rows don't need that extra space.
  int labelSpace = ui.lcdGetFontHeightWithDecentersAndLineSpacing() + 4;
  int rowHeight = ui.displaySpaceHeight / 4;
  int selectionRowHeight = rowHeight - labelSpace - 6;
  int buttonRowHeight = rowHeight - 20;

  int fullWidth = ui.displaySpaceWidth - 20;
  int fullCenterX = ui.displaySpaceLeftX + ui.displaySpaceWidth / 2;

  int row1Y = ui.displaySpaceTopY + labelSpace + selectionRowHeight / 2;
  int row2Y = ui.displaySpaceTopY + rowHeight + rowHeight / 2;
  int row3Y = ui.displaySpaceTopY + 2 * rowHeight + rowHeight / 2;
  int row4Y = ui.displaySpaceTopY + 3 * rowHeight + rowHeight / 2;

  fanScheduleBox.centerX = fullCenterX;
  fanScheduleBox.centerY = row1Y;
  fanScheduleBox.width = fullWidth;
  fanScheduleBox.height = selectionRowHeight;

  fanProbeLinkButton.centerX = fullCenterX;
  fanProbeLinkButton.centerY = row2Y;
  fanProbeLinkButton.width = fullWidth;
  fanProbeLinkButton.height = buttonRowHeight;

  fanTempRangeLinkButton.centerX = fullCenterX;
  fanTempRangeLinkButton.centerY = row3Y;
  fanTempRangeLinkButton.width = fullWidth;
  fanTempRangeLinkButton.height = buttonRowHeight;

  fanScheduleHoursLinkButton.centerX = fullCenterX;
  fanScheduleHoursLinkButton.centerY = row4Y;
  fanScheduleHoursLinkButton.width = fullWidth;
  fanScheduleHoursLinkButton.height = buttonRowHeight;
}

void drawFanControlTitleBar()
{
  char title[32];
  snprintf(title, sizeof(title), "Fan Control Circuit - %s", fanOn ? "ON" : "OFF");
  ui.drawTitleBarWithBackButton(title);
}

// Redraws the whole Fan Control screen -- called at screen entry, and again
// after returning from any link screen since those overwrite the display.
void drawFanControlScreen()
{
  ui.lcdSetFont(UI_Font_13_Bold);
  drawFanControlTitleBar();
  ui.clearDisplaySpace();
  layoutFanControlWidgets();

  fanScheduleBox.value = fanTimeEnabled;

  ui.drawSelectionBox(fanScheduleBox);
  drawSelectionBoxHighlight(fanScheduleBox, 2);
  ui.drawButton(fanProbeLinkButton);
  ui.drawButton(fanTempRangeLinkButton);
  ui.drawButton(fanScheduleHoursLinkButton);
}

void commandFanControl(void)
{
  drawFanControlScreen();
  bool lastFanOn = fanOn;

  while (true)
  {
    ui.getTouchEvents();
    backgroundUpdate();
    if (ui.checkForBackButtonClicked())
      return;

    if (ui.checkForSelectionBoxTouched(fanScheduleBox))
    {
      fanTimeEnabled = fanScheduleBox.value;
      ui.writeConfigurationInt(EEPROM_ADDR_FAN_TIME_ENABLE, fanTimeEnabled);
      flushEepromToFlash();
      drawSelectionBoxHighlight(fanScheduleBox, 2);
    }

    if (ui.checkForButtonClicked(fanProbeLinkButton))
    {
      commandFanProbe();
      drawFanControlScreen();
      lastFanOn = fanOn;
    }
    if (ui.checkForButtonClicked(fanTempRangeLinkButton))
    {
      commandFanTempRange();
      drawFanControlScreen();
      lastFanOn = fanOn;
    }
    if (ui.checkForButtonClicked(fanScheduleHoursLinkButton))
    {
      commandFanScheduleHours();
      drawFanControlScreen();
      lastFanOn = fanOn;
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
//                       Screen 3a: Fan Control -- Probe
// ---------------------------------------------------------------------------------
//
// 2 full-width stacked buttons instead of a single-row SELECTION_BOX -- see
// the note above layoutFanControlWidgets() for why (this predates temp3/4
// removal, back when there were 4 probe choices; kept as its own screen
// rather than folded back into an inline selector since 2 stacked buttons
// still reads fine). drawButton()'s buttonSelectedFlg draws the currently-
// selected probe highlighted.

BUTTON fanProbeButton0 = {"Fridge", 0, 0, 0, 0};
BUTTON fanProbeButton1 = {"DC Cabinet", 0, 0, 0, 0};

void layoutFanProbeWidgets()
{
  int rowHeight = ui.displaySpaceHeight / 2;
  int centerX = ui.displaySpaceLeftX + ui.displaySpaceWidth / 2;
  int widgetWidth = ui.displaySpaceWidth - 40;
  int widgetHeight = rowHeight - 20;

  fanProbeButton0.centerX = centerX;
  fanProbeButton0.centerY = ui.displaySpaceTopY + rowHeight / 2;
  fanProbeButton0.width = widgetWidth;
  fanProbeButton0.height = widgetHeight;

  fanProbeButton1.centerX = centerX;
  fanProbeButton1.centerY = ui.displaySpaceTopY + rowHeight + rowHeight / 2;
  fanProbeButton1.width = widgetWidth;
  fanProbeButton1.height = widgetHeight;
}

// The 2-arg drawButton(BUTTON&, selectedFlg) overload that would normally
// highlight the selected choice is private in this library -- draw a
// yellow border around the selected probe's button ourselves instead.
void drawFanProbeButtons()
{
  ui.drawButton(fanProbeButton0);
  ui.drawButton(fanProbeButton1);

  BUTTON *selected = (fanProbeIndex == 1) ? &fanProbeButton1 : &fanProbeButton0;

  int x1 = selected->centerX - selected->width / 2;
  int y1 = selected->centerY - selected->height / 2;
  ui.lcdDrawRectangle(x1, y1, selected->width, selected->height, LCD_YELLOW);
  ui.lcdDrawRectangle(x1 + 1, y1 + 1, selected->width - 2, selected->height - 2, LCD_YELLOW);
}

void commandFanProbe(void)
{
  ui.lcdSetFont(UI_Font_13_Bold);
  ui.drawTitleBarWithBackButton("Select Temp Probe for Fan Circuit");
  ui.clearDisplaySpace();
  layoutFanProbeWidgets();
  drawFanProbeButtons();

  while (true)
  {
    ui.getTouchEvents();
    backgroundUpdate();
    if (ui.checkForBackButtonClicked())
      return;

    int newIndex = -1;
    if (ui.checkForButtonClicked(fanProbeButton0)) newIndex = 0;
    if (ui.checkForButtonClicked(fanProbeButton1)) newIndex = 1;

    if (newIndex >= 0 && newIndex != fanProbeIndex)
    {
      fanProbeIndex = newIndex;
      ui.writeConfigurationInt(EEPROM_ADDR_FAN_PROBE, fanProbeIndex);
      flushEepromToFlash();
      drawFanProbeButtons();
    }

    delay(50);
  }
}

// ---------------------------------------------------------------------------------
//                    Screen 3a: Fan Control -- Temp Range
// ---------------------------------------------------------------------------------

NUMBER_BOX_FLOAT fanMinTempBox = {"Fan Off Temp", 70.0, 32.0, 150.0, 1.0, 1, 0, 0, 0, 0};
NUMBER_BOX_FLOAT fanMaxTempBox = {"Fan On Temp", 90.0, 32.0, 150.0, 1.0, 1, 0, 0, 0, 0};

// Only 2 widgets on this screen -- a single row, generously sized.
void layoutFanTempRangeWidgets()
{
  int labelSpace = ui.lcdGetFontHeightWithDecentersAndLineSpacing() + 4;
  int colWidth = ui.displaySpaceWidth / 2;
  int widgetHeight = ui.displaySpaceHeight - labelSpace - 30;
  int leftX = ui.displaySpaceLeftX + colWidth / 2;
  int rightX = ui.displaySpaceLeftX + colWidth + colWidth / 2;
  int widgetWidth = colWidth - 20;
  int rowY = ui.displaySpaceTopY + labelSpace + widgetHeight / 2 + 10;

  fanMinTempBox.centerX = leftX;
  fanMinTempBox.centerY = rowY;
  fanMinTempBox.width = widgetWidth;
  fanMinTempBox.height = widgetHeight;

  fanMaxTempBox.centerX = rightX;
  fanMaxTempBox.centerY = rowY;
  fanMaxTempBox.width = widgetWidth;
  fanMaxTempBox.height = widgetHeight;
}

void commandFanTempRange(void)
{
  ui.lcdSetFont(UI_Font_13_Bold);
  ui.drawTitleBarWithBackButton("Temp Range");
  ui.clearDisplaySpace();
  layoutFanTempRangeWidgets();

  fanMinTempBox.value = fanMinTemp;
  fanMaxTempBox.value = fanMaxTemp;

  ui.drawNumberBox(fanMinTempBox);
  ui.drawNumberBox(fanMaxTempBox);

  while (true)
  {
    ui.getTouchEvents();
    backgroundUpdate();
    if (ui.checkForBackButtonClicked())
      return;

    if (ui.checkForNumberBoxTouched(fanMinTempBox))
    {
      fanMinTemp = fanMinTempBox.value;
      ui.writeConfigurationFloat(EEPROM_ADDR_FAN_MIN_TEMP, fanMinTemp);
      flushEepromToFlash();
    }
    if (ui.checkForNumberBoxTouched(fanMaxTempBox))
    {
      fanMaxTemp = fanMaxTempBox.value;
      ui.writeConfigurationFloat(EEPROM_ADDR_FAN_MAX_TEMP, fanMaxTemp);
      flushEepromToFlash();
    }

    delay(50);
  }
}

// ---------------------------------------------------------------------------------
//                  Screen 3b: Fan Control -- Schedule Hours
// ---------------------------------------------------------------------------------

// Hour is kept in 12-hour form (1-12) with a separate AM/PM selector, same
// pattern as the Set Time screen -- fanStartHour/fanEndHour themselves stay
// 24-hour (that's what's persisted to EEPROM and compared against
// rtc.now().hour() in updateFanControl()'s schedule-window check).
NUMBER_BOX fanStartHourBox = {"Start Hr", 8, 1, 12, 1, 0, 0, 0, 0};
NUMBER_BOX fanEndHourBox = {"End Hr", 8, 1, 12, 1, 0, 0, 0, 0};
SELECTION_BOX fanStartAmPmBox = {"AM/PM", 0, "AM", "PM", "", "", 0, 0, 0, 0};
SELECTION_BOX fanEndAmPmBox = {"AM/PM", 0, "AM", "PM", "", "", 0, 0, 0, 0};

// 2x2 grid: Start/End Hr on top, their AM/PM selectors below.
void layoutFanScheduleHoursWidgets()
{
  int labelSpace = ui.lcdGetFontHeightWithDecentersAndLineSpacing() + 4;
  int rowHeight = ui.displaySpaceHeight / 2;
  int colWidth = ui.displaySpaceWidth / 2;
  int widgetHeight = rowHeight - labelSpace - 6;

  int leftX = ui.displaySpaceLeftX + colWidth / 2;
  int rightX = ui.displaySpaceLeftX + colWidth + colWidth / 2;
  int widgetWidth = colWidth - 20;

  int row1Y = ui.displaySpaceTopY + labelSpace + widgetHeight / 2;
  int row2Y = ui.displaySpaceTopY + rowHeight + labelSpace + widgetHeight / 2;

  fanStartHourBox.centerX = leftX;
  fanStartHourBox.centerY = row1Y;
  fanStartHourBox.width = widgetWidth;
  fanStartHourBox.height = widgetHeight;

  fanEndHourBox.centerX = rightX;
  fanEndHourBox.centerY = row1Y;
  fanEndHourBox.width = widgetWidth;
  fanEndHourBox.height = widgetHeight;

  fanStartAmPmBox.centerX = leftX;
  fanStartAmPmBox.centerY = row2Y;
  fanStartAmPmBox.width = widgetWidth;
  fanStartAmPmBox.height = widgetHeight;

  fanEndAmPmBox.centerX = rightX;
  fanEndAmPmBox.centerY = row2Y;
  fanEndAmPmBox.width = widgetWidth;
  fanEndAmPmBox.height = widgetHeight;
}

void commandFanScheduleHours(void)
{
  ui.lcdSetFont(UI_Font_13_Bold);
  ui.drawTitleBarWithBackButton("Schedule Fan Control");
  ui.clearDisplaySpace();
  layoutFanScheduleHoursWidgets();

  fanStartHourBox.value = hour12From24(fanStartHour);
  fanStartAmPmBox.value = ampmFrom24(fanStartHour);
  fanEndHourBox.value = hour12From24(fanEndHour);
  fanEndAmPmBox.value = ampmFrom24(fanEndHour);

  ui.drawNumberBox(fanStartHourBox);
  ui.drawNumberBox(fanEndHourBox);
  ui.drawSelectionBox(fanStartAmPmBox);
  ui.drawSelectionBox(fanEndAmPmBox);

  while (true)
  {
    ui.getTouchEvents();
    backgroundUpdate();
    if (ui.checkForBackButtonClicked())
      return;

    boolean startChanged = ui.checkForNumberBoxTouched(fanStartHourBox);
    startChanged |= ui.checkForSelectionBoxTouched(fanStartAmPmBox);
    if (startChanged)
    {
      fanStartHour = hour24From12AndAmPm(fanStartHourBox.value, fanStartAmPmBox.value);
      ui.writeConfigurationInt(EEPROM_ADDR_FAN_START_HOUR, fanStartHour);
      flushEepromToFlash();
    }

    boolean endChanged = ui.checkForNumberBoxTouched(fanEndHourBox);
    endChanged |= ui.checkForSelectionBoxTouched(fanEndAmPmBox);
    if (endChanged)
    {
      fanEndHour = hour24From12AndAmPm(fanEndHourBox.value, fanEndAmPmBox.value);
      ui.writeConfigurationInt(EEPROM_ADDR_FAN_END_HOUR, fanEndHour);
      flushEepromToFlash();
    }

    delay(50);
  }
}

// ---------------------------------------------------------------------------------
//                             Screen 4: Settings
// ---------------------------------------------------------------------------------
//
// Split across 3 screens for the same reason as Fan Control: Settings (this)
// has Zero Level + links to Date and Time, each their own screen with their
// own back button (returning here).
void commandSettingsDate(void);
void commandSettingsTime(void);

BUTTON zeroLevelButton = {"Zero Level", 0, 0, 0, 0};
BUTTON settingsDateLinkButton = {"Date", 0, 0, 0, 0};
BUTTON settingsTimeLinkButton = {"Time", 0, 0, 0, 0};

// 3 rows, all full-width buttons -- lots of room since this screen only
// has links + one action now.
void layoutSettingsWidgets()
{
  int rowHeight = ui.displaySpaceHeight / 3;
  int centerX = ui.displaySpaceLeftX + ui.displaySpaceWidth / 2;
  int widgetWidth = ui.displaySpaceWidth - 40;
  int widgetHeight = rowHeight - 20;

  zeroLevelButton.centerX = centerX;
  zeroLevelButton.centerY = ui.displaySpaceTopY + rowHeight / 2;
  zeroLevelButton.width = widgetWidth;
  zeroLevelButton.height = widgetHeight;

  settingsDateLinkButton.centerX = centerX;
  settingsDateLinkButton.centerY = ui.displaySpaceTopY + rowHeight + rowHeight / 2;
  settingsDateLinkButton.width = widgetWidth;
  settingsDateLinkButton.height = widgetHeight;

  settingsTimeLinkButton.centerX = centerX;
  settingsTimeLinkButton.centerY = ui.displaySpaceTopY + 2 * rowHeight + rowHeight / 2;
  settingsTimeLinkButton.width = widgetWidth;
  settingsTimeLinkButton.height = widgetHeight;
}

// Redraws the whole Settings screen -- called at screen entry, and again
// after returning from either link screen since those overwrite the display.
void drawSettingsScreen()
{
  ui.lcdSetFont(UI_Font_13_Bold);
  ui.drawTitleBarWithBackButton("Settings");
  ui.clearDisplaySpace();
  layoutSettingsWidgets();

  ui.drawButton(zeroLevelButton);
  ui.drawButton(settingsDateLinkButton);
  ui.drawButton(settingsTimeLinkButton);
}

void commandSettings(void)
{
  drawSettingsScreen();

  while (true)
  {
    ui.getTouchEvents();
    backgroundUpdate();
    if (ui.checkForBackButtonClicked())
      return;

    if (ui.checkForButtonClicked(zeroLevelButton))
    {
      // Must use the raw pre-conversion degree values here, not
      // txPacket.pitch/roll -- those are in inches once LEVEL_PITCH/ROLL_
      // DISTANCE_INCHES are set, but pitchOffset/rollOffset are subtracted
      // from a degrees value inside updateTilt().
      pitchOffset += lastPitchDegrees;
      rollOffset += lastRollDegrees;
      ui.writeConfigurationFloat(EEPROM_ADDR_PITCH_OFFSET, pitchOffset);
      ui.writeConfigurationFloat(EEPROM_ADDR_ROLL_OFFSET, rollOffset);
      flushEepromToFlash();
    }

    if (ui.checkForButtonClicked(settingsDateLinkButton))
    {
      commandSettingsDate();
      drawSettingsScreen();
    }
    if (ui.checkForButtonClicked(settingsTimeLinkButton))
    {
      commandSettingsTime();
      drawSettingsScreen();
    }

    delay(50);
  }
}

// ---------------------------------------------------------------------------------
//                          Screen 4a: Settings -- Date
// ---------------------------------------------------------------------------------

NUMBER_BOX yearBox = {"Year", 2026, 2020, 2099, 1, 0, 0, 0, 0};
NUMBER_BOX monthBox = {"Month", 1, 1, 12, 1, 0, 0, 0, 0};
NUMBER_BOX dayBox = {"Day", 1, 1, 31, 1, 0, 0, 0, 0};
BUTTON saveDateButton = {"Save Date", 0, 0, 0, 0};

// Month/Day/Year across one row, Save Date below -- 3-column layout with
// no label-space needed on the button row (BUTTON draws its label inside).
void layoutSettingsDateWidgets()
{
  int labelSpace = ui.lcdGetFontHeightWithDecentersAndLineSpacing() + 4;
  int rowHeight = ui.displaySpaceHeight / 2;
  int colWidth = ui.displaySpaceWidth / 3;
  int widgetHeight = rowHeight - labelSpace - 6;

  int col1X = ui.displaySpaceLeftX + colWidth / 2;
  int col2X = ui.displaySpaceLeftX + colWidth + colWidth / 2;
  int col3X = ui.displaySpaceLeftX + 2 * colWidth + colWidth / 2;
  int widgetWidth = colWidth - 20;

  int row1Y = ui.displaySpaceTopY + labelSpace + widgetHeight / 2;
  int row2Y = ui.displaySpaceTopY + rowHeight + (rowHeight - 20) / 2;
  int saveButtonHeight = 50;

  monthBox.centerX = col1X;
  monthBox.centerY = row1Y;
  monthBox.width = widgetWidth;
  monthBox.height = widgetHeight;

  dayBox.centerX = col2X;
  dayBox.centerY = row1Y;
  dayBox.width = widgetWidth;
  dayBox.height = widgetHeight;

  yearBox.centerX = col3X;
  yearBox.centerY = row1Y;
  yearBox.width = widgetWidth;
  yearBox.height = widgetHeight;

  saveDateButton.centerX = ui.displaySpaceLeftX + ui.displaySpaceWidth / 2;
  saveDateButton.centerY = row2Y;
  saveDateButton.width = ui.displaySpaceWidth - 60;
  saveDateButton.height = saveButtonHeight;
}

void commandSettingsDate(void)
{
  ui.lcdSetFont(UI_Font_13_Bold);
  ui.drawTitleBarWithBackButton("Set Date");
  ui.clearDisplaySpace();
  layoutSettingsDateWidgets();

  if (rtcAvailable)
  {
    // A glitched I2C read (e.g. right after boot, while the MPU6050 on the
    // same bus is still retrying) can hand back garbage bytes -- clamp to
    // each box's own valid range rather than displaying/allowing something
    // like "month 160" that NUMBER_BOX would never let you dial in by hand.
    DateTime now = rtc.now();
    int y = now.year();
    int mo = now.month();
    int d = now.day();
    yearBox.value = (y >= yearBox.minimumValue && y <= yearBox.maximumValue) ? y : yearBox.value;
    monthBox.value = (mo >= monthBox.minimumValue && mo <= monthBox.maximumValue) ? mo : monthBox.value;
    dayBox.value = (d >= dayBox.minimumValue && d <= dayBox.maximumValue) ? d : dayBox.value;
  }

  ui.drawNumberBox(monthBox);
  ui.drawNumberBox(dayBox);
  ui.drawNumberBox(yearBox);
  ui.drawButton(saveDateButton);

  while (true)
  {
    ui.getTouchEvents();
    backgroundUpdate();
    if (ui.checkForBackButtonClicked())
      return;

    ui.checkForNumberBoxTouched(yearBox);
    ui.checkForNumberBoxTouched(monthBox);
    ui.checkForNumberBoxTouched(dayBox);

    if (ui.checkForButtonClicked(saveDateButton) && rtcAvailable)
    {
      DateTime current = rtc.now();
      rtc.adjust(DateTime(yearBox.value, monthBox.value, dayBox.value, current.hour(), current.minute(), current.second()));
      Serial.println("RTC date saved from Date screen");
    }

    delay(50);
  }
}

// ---------------------------------------------------------------------------------
//                          Screen 4b: Settings -- Time
// ---------------------------------------------------------------------------------

// Hour is kept in 12-hour form (1-12) with a separate AM/PM selector --
// converted to/from the RTC's 24-hour DateTime at load/save time.
NUMBER_BOX hourBox = {"Hour", 12, 1, 12, 1, 0, 0, 0, 0};
NUMBER_BOX minuteBox = {"Minute", 0, 0, 59, 1, 0, 0, 0, 0};
SELECTION_BOX ampmBox = {"AM/PM", 0, "AM", "PM", "", "", 0, 0, 0, 0};
BUTTON saveTimeButton = {"Save Time", 0, 0, 0, 0};

void layoutSettingsTimeWidgets()
{
  int labelSpace = ui.lcdGetFontHeightWithDecentersAndLineSpacing() + 4;
  int rowHeight = ui.displaySpaceHeight / 3;
  int colWidth = ui.displaySpaceWidth / 2;
  int widgetHeight = rowHeight - labelSpace - 6;

  int col1X = ui.displaySpaceLeftX + colWidth / 2;
  int col2X = ui.displaySpaceLeftX + colWidth + colWidth / 2;
  int widgetWidth = colWidth - 20;
  int fullWidth = ui.displaySpaceWidth - 20;
  int fullCenterX = ui.displaySpaceLeftX + ui.displaySpaceWidth / 2;

  int row1Y = ui.displaySpaceTopY + labelSpace + widgetHeight / 2;
  int row2Y = ui.displaySpaceTopY + rowHeight + labelSpace + widgetHeight / 2;
  int row3Y = ui.displaySpaceTopY + 2 * rowHeight + rowHeight / 2;

  hourBox.centerX = col1X;
  hourBox.centerY = row1Y;
  hourBox.width = widgetWidth;
  hourBox.height = widgetHeight;

  minuteBox.centerX = col2X;
  minuteBox.centerY = row1Y;
  minuteBox.width = widgetWidth;
  minuteBox.height = widgetHeight;

  ampmBox.centerX = fullCenterX;
  ampmBox.centerY = row2Y;
  ampmBox.width = fullWidth;
  ampmBox.height = widgetHeight;

  saveTimeButton.centerX = fullCenterX;
  saveTimeButton.centerY = row3Y;
  saveTimeButton.width = fullWidth;
  saveTimeButton.height = 50;
}

void commandSettingsTime(void)
{
  ui.lcdSetFont(UI_Font_13_Bold);
  ui.drawTitleBarWithBackButton("Set Time");
  ui.clearDisplaySpace();
  layoutSettingsTimeWidgets();

  if (rtcAvailable)
  {
    DateTime now = rtc.now();
    hourBox.value = hour12From24(now.hour());
    ampmBox.value = ampmFrom24(now.hour());
    minuteBox.value = now.minute();
  }

  ui.drawNumberBox(hourBox);
  ui.drawNumberBox(minuteBox);
  ui.drawSelectionBox(ampmBox);
  ui.drawButton(saveTimeButton);

  while (true)
  {
    ui.getTouchEvents();
    backgroundUpdate();
    if (ui.checkForBackButtonClicked())
      return;

    ui.checkForNumberBoxTouched(hourBox);
    ui.checkForNumberBoxTouched(minuteBox);
    ui.checkForSelectionBoxTouched(ampmBox);

    if (ui.checkForButtonClicked(saveTimeButton) && rtcAvailable)
    {
      int hour24 = hour24From12AndAmPm(hourBox.value, ampmBox.value);
      DateTime current = rtc.now();
      rtc.adjust(DateTime(current.year(), current.month(), current.day(), hour24, minuteBox.value, 0));
      Serial.println("RTC time saved from Time screen");
    }

    delay(50);
  }
}
