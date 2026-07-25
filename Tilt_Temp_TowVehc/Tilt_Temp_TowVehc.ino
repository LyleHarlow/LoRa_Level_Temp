//      ******************************************************************
//      *                                                                *
//      *   TowVehicle board -- buttons, LEDs, OLED, LoRa receive        *
//      *                                                                *
//      ******************************************************************

// Last updated: 2026-07-25 10:52 PDT

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

char this_file[] = "Tilt_Temp_TowVehc";
char ver[] = "ver 2.2  " __DATE__;

const long LORA_BAND = 915E6;

// ---------------------------------------------------------------------------------
//                                    Pin definitions
// ---------------------------------------------------------------------------------
// OLED, LoRa radio, and Vext pins come from the board package (via
// Heltec.begin()) and don't need to be defined here. These are the custom
// pins specific to this carrier board -- see the Confirmed Hardware
// section of the plan for how they were derived from the schematic.

const int BUTTON1_PIN = 17;  // previous screen
const int BUTTON2_PIN = 39;  // next screen
const int BUTTON3_PIN = 38;  // mute/acknowledge alert
const int BUTTON4_PIN = 12;  // spare (logged to Serial only for now)

const int LED1_PIN = 2;   // heartbeat: brief flash on each received packet
const int LED2_PIN = 32;  // alert: solid on when the LoRa link is lost

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
  SCREEN_COUNT = 3
};
int currentScreen = SCREEN_LEVEL;

// simple debounce: pin, and whether it was down last time we checked
struct DebouncedButton
{
  int pin;
  bool wasDown;
};
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

  Serial.println();
  Serial.println(this_file);
  Serial.println(ver);

  pinMode(BUTTON1_PIN, INPUT_PULLUP);  // GPIO17 supports an internal pull-up
  pinMode(BUTTON2_PIN, INPUT);         // GPIO39 is input-only, no internal pull-up -- board has an external 10K pull-up (R3)
  pinMode(BUTTON3_PIN, INPUT);         // GPIO38 is input-only, no internal pull-up -- board has an external 10K pull-up (R4)
  pinMode(BUTTON4_PIN, INPUT_PULLUP);  // GPIO12 supports an internal pull-up
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

void drawScreen()
{
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);

  char line[32];

  switch (currentScreen)
  {
  case SCREEN_LEVEL:
    Heltec.display->drawString(0, 0, "Level");
    formatDirectionalValue(line, sizeof(line), rxPacket.pitch, "Nose High", "Nose Low");
    Heltec.display->drawString(0, 20, line);
    formatDirectionalValue(line, sizeof(line), rxPacket.roll, "Left High", "Right High");
    Heltec.display->drawString(0, 40, line);
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
    if (lastPacketTime == 0)
    {
      Heltec.display->drawString(0, 16, "No packet yet");
    }
    else
    {
      snprintf(line, sizeof(line), "Last: %lus ago", (millis() - lastPacketTime) / 1000);
      Heltec.display->drawString(0, 16, line);
      Heltec.display->drawString(0, 29, isLinkLost() ? "LINK LOST" : "OK");
    }
    snprintf(line, sizeof(line), "RSSI: %d", Heltec.LoRa.packetRssi());
    Heltec.display->drawString(0, 42, line);
    break;
  }

  Heltec.display->display();
}
