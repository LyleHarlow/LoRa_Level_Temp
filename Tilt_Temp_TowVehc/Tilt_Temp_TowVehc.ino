//      ******************************************************************
//      *                                                                *
//      *   TowVehicle board -- buttons, LEDs, OLED, LoRa receive        *
//      *                                                                *
//      ******************************************************************

// Last updated: 2026-07-31 10:35 PDT

//
// Heltec WiFi LoRa 32 V2, built on Heltec's own board package + Heltec_ESP32
// library (https://github.com/HelTecAutomation/Heltec_ESP32) rather than
// raw ESP32 core + separate display/LoRa libraries. Heltec.begin() powers
// up the module's Vext rail, inits the built-in OLED, and inits the LoRa
// radio using the board package's own pin definitions -- no manual
// SPI/Wire/pin setup needed for those.
//
// Board package: https://resource.heltec.cn/download/package_heltec_esp32_index.json
// Select board "Heltec WiFi LoRa 32(V2)".
//
// Receives tilt/temp data from the Airstream board over LoRa and shows it
// on the built-in OLED. Buttons cycle between screens; see ../hardware for
// schematics and C:\Users\Harlow\.claude\plans\cozy-sauteeing-whale.md for
// the full plan this was built from.
//
// NOTE: the Airstream board intentionally does NOT use this library --
// Heltec's board package fixes Vext to GPIO21, which on the Airstream
// board is TOUCH_CS. Heltec.begin() unconditionally drives Vext, which
// would conflict with the touchscreen's chip-select. TowVehicle doesn't
// use GPIO21 for anything, so no conflict here.
//

#include "heltec.h"
#include "LoRaPacket.h"

#if defined(WIFI_LORA_32_V3)
// driver/gpio.h exists in this core (confirmed on disk) but doesn't
// resolve via #include here -- likely something about how the
// precompiled Heltec_ESP32_Dev-Boards library exposes its include paths
// to sketch code. Declare what's needed directly instead; esp_err_t/
// gpio_num_t are already available transitively via heltec.h.
extern "C" esp_err_t gpio_reset_pin(gpio_num_t gpio_num);
// gpio_drive_cap_t itself is already available transitively (via
// hal/gpio_types.h, pulled in by Arduino.h) -- only the function
// declaration was missing.
extern "C" esp_err_t gpio_set_drive_capability(gpio_num_t gpio_num, gpio_drive_cap_t strength);
#endif

// Defined up here (rather than down near where `buttons[]` is populated)
// because the conditional #include <RadioLib.h> below shifts where
// Arduino's auto-generated function prototypes get inserted -- same class
// of gotcha as mainMenu[] in Tilt_Temp_AirStream.ino: buttonPressed()'s
// auto-prototype ended up placed before this struct was visible.
struct DebouncedButton
{
  int pin;
  bool wasDown;
};

char this_file[] = "Tilt_Temp_TowVehc";
char ver[] = "ver 2.2  " __DATE__;

const long LORA_BAND = 915E6;

// ---------------------------------------------------------------------------------
//                                    V3 LoRa radio (RadioLib)
// ---------------------------------------------------------------------------------
// V3's radio chip is a genuinely different chip (SX1262, not V2's SX127x)
// on its own dedicated pins -- separate from the general-purpose header
// covered by the pin-mapping table below, and internal to the module (see
// the separate "LoRa" box in Heltec's V3 pinout diagram). Heltec.begin()'s
// automatic LoRa init only knows the classic SX127x API (Heltec_ESP32_Dev-
// Boards' bundled lora/LoRa.cpp always calls the SX127x-style begin(),
// which can't talk to an SX1262 -- it fails every time and hangs forever
// in an infinite while(1) inside that library). So LoRaEnable is left
// false in Heltec.begin() for V3, and the radio is driven directly here
// via RadioLib instead.
#if defined(WIFI_LORA_32_V3)
#include <RadioLib.h>

const int LORA_NSS_PIN = 8;
const int LORA_SCK_PIN = 9;
const int LORA_MOSI_PIN = 10;
const int LORA_MISO_PIN = 11;
const int LORA_RST_PIN = 12;
const int LORA_BUSY_PIN = 13;
const int LORA_DIO1_PIN = 14;

SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN);

volatile bool loraPacketReceivedFlag = false;

void IRAM_ATTR onLoraDio1Rise()
{
  loraPacketReceivedFlag = true;
}
#endif

// ---------------------------------------------------------------------------------
//                                    Pin definitions
// ---------------------------------------------------------------------------------
// OLED, LoRa radio, and Vext pins come from the board package (via
// Heltec.begin()) and don't need to be defined here -- those peripherals
// are wired internally on the module itself (not through this carrier
// PCB's header), so the board package's own V2/V3-specific config handles
// them correctly regardless of which module is populated.
//
// These 6 signals ARE wired through this carrier PCB's header into fixed
// physical pin positions, so which GPIO is actually present depends on
// which module occupies that socket. WIFI_LORA_32_V2/V3 are defined by the
// board package itself (boards.txt build.defines) based on Tools > Board,
// so selecting the right board in the IDE picks the right pins here
// automatically -- no manual toggle needed.
//
// V3 pin numbers below were initially derived by matching this carrier
// board's schematic (U6 pin numbers, which correspond 1:1 to the V2
// module's published edge pinout) against the V3 module's published
// pinout at the SAME physical header positions -- that derivation got
// LED2 wrong (40 instead of 39), since confirmed and corrected directly
// on hardware; the other 5 matched. Three of these six land on pins with
// real caveats on V3 (see inline comments) -- worth keeping in mind if
// anything acts up.
#if defined(WIFI_LORA_32_V3)
// All 6 confirmed directly on hardware (2026-07-31) -- the derived V2->V3
// header-position mapping had LED2 wrong (40->39); LED1 and the 4 buttons
// matched the derivation.
const int BUTTON1_PIN = 20;  // previous screen
const int BUTTON2_PIN = 42;  // next screen -- CAVEAT: this is MTMS, a JTAG debug pin
const int BUTTON3_PIN = 45;  // mute/acknowledge alert -- CAVEAT: S3 boot-strapping pin (flash voltage select)
const int BUTTON4_PIN = 5;   // spare (logged to Serial only for now)

const int LED1_PIN = 26;  // heartbeat: brief flash on each received packet -- CAVEAT: labeled SPICS1, possibly reserved for PSRAM/flash on some V3 module variants
const int LED2_PIN = 39;  // alert: solid on when the LoRa link is lost -- CAVEAT: this is MTCK, a JTAG debug pin
#else
const int BUTTON1_PIN = 17;  // previous screen
const int BUTTON2_PIN = 39;  // next screen
const int BUTTON3_PIN = 38;  // mute/acknowledge alert
const int BUTTON4_PIN = 12;  // spare (logged to Serial only for now)

const int LED1_PIN = 2;   // heartbeat: brief flash on each received packet
const int LED2_PIN = 32;  // alert: solid on when the LoRa link is lost
#endif

// ---------------------------------------------------------------------------------

LoRaPacket rxPacket;
unsigned long lastPacketTime = 0;
const unsigned long LINK_LOST_TIMEOUT_MS = 30000;
bool alertMuted = false;

enum Screen
{
  SCREEN_LEVEL = 0,
  SCREEN_TEMPS = 1,
  SCREEN_LINK = 2,
  SCREEN_FAN = 3,
  SCREEN_COUNT = 4
};
int currentScreen = SCREEN_LEVEL;

// Matches Airstream's Fan Control probe order (EEPROM_ADDR_FAN_PROBE).
const char *FAN_PROBE_NAMES[4] = {"Fridge", "Freezer", "Inside AS", "DC Cabinet"};

// simple debounce: pin, and whether it was down last time we checked
DebouncedButton buttons[4] = {
    {BUTTON1_PIN, false},
    {BUTTON2_PIN, false},
    {BUTTON3_PIN, false},
    {BUTTON4_PIN, false},
};

// ---------------------------------------------------------------------------------
//                                     Setup
// ---------------------------------------------------------------------------------

void setup()
{
#if defined(WIFI_LORA_32_V3)
  // Display + Vext + Serial only -- LoRaEnable is false because the radio
  // is driven directly via RadioLib below instead (see the note above the
  // V3 LoRa radio section for why Heltec.begin()'s own LoRa init can't be
  // used here).
  Heltec.begin(true, false, true, true, LORA_BAND);

  SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);
  // Parameters chosen to exactly match Tilt_Temp_AirStream's radio config
  // (still on V2 -- see LORA_BAND and setSpreadingFactor/etc. in the #else
  // branch below): 915MHz, 125kHz bandwidth, spreading factor 11, sync
  // word 0x34, CRC on. Coding rate 5 (4/5) matches what BOTH sketches'
  // vendored classic LoRa.h libraries actually use today -- neither one
  // explicitly calls setCodingRate4(), so they've been running at the
  // SX127x's power-on-reset default of 4/5 this whole time. RadioLib
  // doesn't share that hardware default (its own default is 4/7), so it
  // has to be requested explicitly here to match, or the two boards won't
  // be able to decode each other's packets -- silently, with no error on
  // either side (this bit us once already, with a different setting).
  int radioState = radio.begin(LORA_BAND / 1E6, 125.0, 11, 5, 0x34, 17, 8);
  if (radioState != RADIOLIB_ERR_NONE)
  {
    Serial.print("SX1262 radio.begin() failed, code ");
    Serial.println(radioState);
  }
  radio.setCRC(true);

  // Confirmed via byte-for-byte TX/RX comparison: the PHY header decoded
  // correctly on every packet (right length every time) but the payload
  // came back as near-total garbage -- LDRO (Low Data Rate Optimization)
  // changes how the payload specifically gets bit-interleaved, and
  // classic LoRa.h's setLdoFlag() has an integer-truncation quirk at
  // SF11/125kHz: symbolDuration = 1000 / (125000 / 2048) truncates to
  // 1000/61 = 16 (integer division, twice), and the check is strictly
  // "> 16", so LDRO ends up DISABLED on both Airstream and the V2
  // TowVehicle. RadioLib's own auto-LDRO uses floating point instead
  // (2048/125.0 = 16.384 >= 16.0 -> true) and enables it, which corrupts
  // every payload without touching the header. Force it off to match.
  radio.forceLDRO(false);

  radio.setDio1Action(onLoraDio1Rise);
  radio.startReceive();
#else
  // Heltec.begin(DisplayEnable, LoRaEnable, SerialEnable, PABOOST, BAND) --
  // handles Serial.begin(), Vext power-up, OLED init, and LoRa radio init
  // (SPI + pins) all in one call, using the board package's own pin
  // definitions for this board variant.
  Heltec.begin(true, true, true, true, LORA_BAND);

  // IMPORTANT: these must exactly match Tilt_Temp_AirStream's radio
  // settings or the two boards can't decode each other's packets at all --
  // with no error on either side, just silence (this bit us once already).
  // Heltec.begin() already sets these same values internally today, but
  // setting them explicitly here means this sketch doesn't silently break
  // if that library's defaults ever change.
  Heltec.LoRa.setSpreadingFactor(11);
  Heltec.LoRa.setSignalBandwidth(125E3);
  Heltec.LoRa.setSyncWord(0x34);
  Heltec.LoRa.enableCrc();
#endif

  Serial.println();
  Serial.println(this_file);
  Serial.println(ver);

  // BUTTON2/BUTTON3 use INPUT (not INPUT_PULLUP) because on the V2 module
  // those header positions are input-only pins with no internal pull-up --
  // the PCB has external 10K pull-ups (R3, R4) wired to those specific
  // header positions instead, regardless of which module is populated. On
  // V3 those positions land on pins that DO support an internal pull-up,
  // but INPUT still works fine there since the external resistor is a
  // fixed PCB component, not something that depends on module choice.
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(BUTTON3_PIN, INPUT);
  pinMode(BUTTON4_PIN, INPUT_PULLUP);

#if defined(WIFI_LORA_32_V3)
  // LED2 (GPIO39, confirmed on hardware -- the derived pin table had this
  // wrong as GPIO40, which is why earlier fix attempts didn't help) glows
  // dimly even when explicitly driven LOW, regardless of USB connection
  // state (ruling out the USB-Serial-JTAG peripheral actively driving it --
  // that would only show up while USB is attached). GPIO39 is MTCK, one of
  // the ESP32-S3's JTAG pins; gpio_reset_pin() forces the IO mux back to
  // plain GPIO in case something still has a residual claim on it, and
  // gpio_set_drive_capability() maxes out the output drive strength in case
  // the dim glow is a weak-pull-vs-weak-drive fight this pin's default
  // (weaker) drive strength was losing. BUTTON2 (GPIO42, MTMS) shares this
  // same class of risk -- revisit if it ever misbehaves the same way.
  gpio_reset_pin((gpio_num_t)LED2_PIN);
  gpio_set_drive_capability((gpio_num_t)LED2_PIN, GPIO_DRIVE_CAP_3);
#endif
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);

  memset(&rxPacket, 0, sizeof(rxPacket));

  delay(1000);
  drawScreen();
}

// ---------------------------------------------------------------------------------
//                                      Loop
// ---------------------------------------------------------------------------------

void loop()
{
  receiveLoRaPacket();
  handleButtons();
  updateAlertLED();
  drawScreen();
  delay(50);
}

// ---------------------------------------------------------------------------------
//                                 LoRa receive
// ---------------------------------------------------------------------------------

void receiveLoRaPacket()
{
#if defined(WIFI_LORA_32_V3)
  if (!loraPacketReceivedFlag)
  {
    return;  // nothing arrived, not an error -- don't spam Serial
  }
  loraPacketReceivedFlag = false;

  size_t packetSize = radio.getPacketLength();
  if (packetSize != sizeof(rxPacket))
  {
    // Something arrived but isn't a valid LoRaPacket -- radio params
    // mismatched between boards, interference, or a version skew between
    // the two sketches' LoRaPacket.h. Worth knowing about, unlike the
    // silent "nothing arrived" case above.
    Serial.print("RX: got ");
    Serial.print(packetSize);
    Serial.print(" bytes, expected ");
    Serial.println(sizeof(rxPacket));
    radio.startReceive();
    return;
  }

  // RX_DONE (the only IRQ in RadioLib's default receive mask) fires
  // whenever the radio finishes clocking in a packet-shaped signal --
  // REGARDLESS of whether its CRC actually validates. readData() checks
  // CRC internally and returns an error for it, but until now nothing here
  // checked that return value, so a CRC-failed (corrupted) packet was
  // silently accepted and displayed as if it were valid. This is very
  // likely why specific fields (not random ones) were consistently wrong:
  // LoRa's interleaving/FEC can leave some byte regions of a corrupted
  // packet closer to correct than others, not a uniform random scramble.
  int16_t readState = radio.readData((uint8_t *)&rxPacket, sizeof(rxPacket));
  radio.startReceive();

  // Diagnostic for tracking down the date/time field corruption -- hex
  // dump of exactly what was received, byte for byte, to compare against
  // the transmit-side dump, REGARDLESS of whether CRC passed (printed
  // before the CRC check below returns early, on purpose -- we need to see
  // the bytes precisely when CRC failed, not just when it didn't). Remove
  // once resolved.
  Serial.print("RX raw bytes (");
  Serial.print(sizeof(rxPacket));
  Serial.print(", readData() code ");
  Serial.print(readState);
  Serial.print("): ");
  {
    uint8_t *raw = (uint8_t *)&rxPacket;
    for (size_t i = 0; i < sizeof(rxPacket); i++)
    {
      if (raw[i] < 0x10)
        Serial.print('0');
      Serial.print(raw[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
  }

  if (readState != RADIOLIB_ERR_NONE)
  {
    Serial.print("RX: readData() failed, code ");
    Serial.println(readState);
    return;
  }
#else
  int packetSize = Heltec.LoRa.parsePacket();
  if (packetSize == 0)
  {
    return;  // nothing arrived, not an error -- don't spam Serial
  }
  if (packetSize != sizeof(rxPacket))
  {
    // Something arrived but isn't a valid LoRaPacket -- radio params
    // mismatched between boards, interference, or a version skew between
    // the two sketches' LoRaPacket.h. Worth knowing about, unlike the
    // silent "nothing arrived" case above.
    Serial.print("RX: got ");
    Serial.print(packetSize);
    Serial.print(" bytes, expected ");
    Serial.println(sizeof(rxPacket));
    return;
  }

  Heltec.LoRa.readBytes((uint8_t *)&rxPacket, sizeof(rxPacket));
#endif

  lastPacketTime = millis();
  alertMuted = false;  // a fresh, valid packet clears any prior mute

  Serial.print("RX  pitch=");
  Serial.print(rxPacket.pitch, 1);
  Serial.print("  roll=");
  Serial.print(rxPacket.roll, 1);
  Serial.print("  t1=");
  Serial.print(rxPacket.temp1, 1);
  Serial.print("  t2=");
  Serial.print(rxPacket.temp2, 1);
  Serial.print("  t3=");
  Serial.print(rxPacket.temp3, 1);
  Serial.print("  t4=");
  Serial.println(rxPacket.temp4, 1);

  digitalWrite(LED1_PIN, HIGH);
  delay(30);
  digitalWrite(LED1_PIN, LOW);
}

bool isLinkLost()
{
  return (lastPacketTime == 0) || (millis() - lastPacketTime > LINK_LOST_TIMEOUT_MS);
}

void updateAlertLED()
{
  digitalWrite(LED2_PIN, (isLinkLost() && !alertMuted) ? HIGH : LOW);
}

// ---------------------------------------------------------------------------------
//                                    Buttons
// ---------------------------------------------------------------------------------

// Returns true once on the press transition (debounced), like the old
// TestIO.ino pattern but table-driven for all 4 buttons.
bool buttonPressed(DebouncedButton &b)
{
  bool isDown = (digitalRead(b.pin) == LOW);
  bool justPressed = isDown && !b.wasDown;
  b.wasDown = isDown;
  return justPressed;
}

void handleButtons()
{
  if (buttonPressed(buttons[0]))
  {
    currentScreen = (currentScreen + SCREEN_COUNT - 1) % SCREEN_COUNT;
  }
  if (buttonPressed(buttons[1]))
  {
    currentScreen = (currentScreen + 1) % SCREEN_COUNT;
  }
  if (buttonPressed(buttons[2]))
  {
    alertMuted = true;
  }
  if (buttonPressed(buttons[3]))
  {
    Serial.println("Button4 pressed (spare, no action assigned yet)");
  }
}

// ---------------------------------------------------------------------------------
//                                    Display
// ---------------------------------------------------------------------------------

// Formats a signed tilt value as a direction word + magnitude, e.g.
// "Nose High 2.3 deg" instead of a signed number -- matches
// Tilt_Temp_AirStream's touchscreen wording. Unit is hardcoded to degrees
// for now since rxPacket doesn't carry which unit Airstream used -- when
// Airstream's LEVEL_POINT_DISTANCE_INCHES gets set to switch to inches,
// this needs updating too (or LoRaPacket needs a unit flag added).
void formatDirectionalValue(char *buf, size_t bufSize, float value, const char *positiveLabel, const char *negativeLabel)
{
  const char *direction = (value >= 0) ? positiveLabel : negativeLabel;
  float magnitude = (value >= 0) ? value : -value;
  snprintf(buf, bufSize, "%s %.1f deg", direction, magnitude);
}

// Compact "8AM"/"11PM"-style hour, matching the Airstream's 12-hour Set
// Time/Schedule Fan Control screens -- this tiny 128x64 OLED doesn't have
// room for a full "8:00 AM".
void formatHour12Compact(char *buf, size_t bufSize, int hour24)
{
  int hour12 = hour24 % 12;
  if (hour12 == 0)
    hour12 = 12;
  snprintf(buf, bufSize, "%d%s", hour12, (hour24 >= 12) ? "PM" : "AM");
}

void drawScreen()
{
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);

  char line[32];

  switch (currentScreen)
  {
  case SCREEN_LEVEL:
    // Label above value (rather than side-by-side, which doesn't fit this
    // 128px-wide screen at a readable size) -- matches the Airstream's own
    // "Front/Back"/"Left/Right" row labels and roll wording (the low side
    // needs a shim, so that's the side named). No date/time here -- this
    // screen is hard enough to read already without extra clutter.
    Heltec.display->drawString(0, 0, "Front/Back");
    formatDirectionalValue(line, sizeof(line), rxPacket.pitch, "Nose High", "Nose Low");
    Heltec.display->drawString(0, 13, line);
    Heltec.display->drawString(0, 32, "Left/Right");
    formatDirectionalValue(line, sizeof(line), rxPacket.roll, "Right Low", "Left Low");
    Heltec.display->drawString(0, 45, line);
    break;

  case SCREEN_TEMPS:
    Heltec.display->drawString(0, 0, "Temps (F)");
    snprintf(line, sizeof(line), "Fridge  %.1f", rxPacket.temp1);
    Heltec.display->drawString(0, 14, line);
    snprintf(line, sizeof(line), "Freezer %.1f", rxPacket.temp2);
    Heltec.display->drawString(0, 27, line);
    snprintf(line, sizeof(line), "Inside  %.1f", rxPacket.temp3);
    Heltec.display->drawString(0, 40, line);
    snprintf(line, sizeof(line), "DC Cab  %.1f", rxPacket.temp4);
    Heltec.display->drawString(0, 53, line);
    break;

  case SCREEN_LINK:
    Heltec.display->drawString(0, 0, "LoRa Link");
    if (rxPacket.year == 0)
    {
      Heltec.display->drawString(0, 13, "No RTC data");
    }
    else
    {
      snprintf(line, sizeof(line), "%d/%d/%d %d:%02d%s", rxPacket.month, rxPacket.day, rxPacket.year,
        (rxPacket.hour % 12 == 0) ? 12 : rxPacket.hour % 12, rxPacket.minute, (rxPacket.hour >= 12) ? "PM" : "AM");
      Heltec.display->drawString(0, 13, line);
    }
    if (lastPacketTime == 0)
    {
      Heltec.display->drawString(0, 26, "No packet yet");
    }
    else
    {
      snprintf(line, sizeof(line), "Last: %lus ago", (millis() - lastPacketTime) / 1000);
      Heltec.display->drawString(0, 26, line);
      Heltec.display->drawString(0, 39, isLinkLost() ? "LINK LOST" : "OK");
    }
#if defined(WIFI_LORA_32_V3)
    snprintf(line, sizeof(line), "RSSI: %.0f", radio.getRSSI());
#else
    snprintf(line, sizeof(line), "RSSI: %d", Heltec.LoRa.packetRssi());
#endif
    Heltec.display->drawString(0, 52, line);
    break;

  case SCREEN_FAN:
  {
    Heltec.display->drawString(0, 0, "Fan Control");
    Heltec.display->drawString(0, 13, rxPacket.fanOn ? "Fan: ON" : "Fan: OFF");
    int probeIndex = rxPacket.fanProbeIndex;
    if (probeIndex < 0 || probeIndex > 3)
      probeIndex = 0;
    snprintf(line, sizeof(line), "Probe: %s", FAN_PROBE_NAMES[probeIndex]);
    Heltec.display->drawString(0, 26, line);
    snprintf(line, sizeof(line), "Off:%.0f  On:%.0f", rxPacket.fanOffTemp, rxPacket.fanOnTemp);
    Heltec.display->drawString(0, 39, line);
    char startBuf[8], endBuf[8];
    formatHour12Compact(startBuf, sizeof(startBuf), rxPacket.fanStartHour);
    formatHour12Compact(endBuf, sizeof(endBuf), rxPacket.fanEndHour);
    snprintf(line, sizeof(line), "Sched: %s-%s", startBuf, endBuf);
    Heltec.display->drawString(0, 52, line);
    break;
  }
  }

  Heltec.display->display();
}
