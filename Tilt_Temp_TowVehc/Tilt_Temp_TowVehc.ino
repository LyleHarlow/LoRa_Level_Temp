//      ******************************************************************
//      *                                                                *
//      *   TowVehicle board -- buttons, LEDs, OLED, LoRa receive        *
//      *                                                                *
//      ******************************************************************

//
// Heltec WiFi LoRa 32 V2. Receives tilt/temp data from the Airstream board
// over LoRa and shows it on the built-in OLED. Buttons cycle between
// screens; see ../hardware for schematics and
// C:\Users\Harlow\.claude\plans\cozy-sauteeing-whale.md for the full plan
// this was built from.
//

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <string.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "LoRaPacket.h"

char this_file[] = "Tilt_Temp_TowVehc";
char ver[] = "ver 2.1  " __DATE__;

// ---------------------------------------------------------------------------------
//                                    Pin definitions
// ---------------------------------------------------------------------------------
// See the Confirmed Hardware section of the plan for how these were derived
// from the corrected schematic.

const int BUTTON1_PIN = 17;  // previous screen
const int BUTTON2_PIN = 39;  // next screen
const int BUTTON3_PIN = 38;  // mute/acknowledge alert
const int BUTTON4_PIN = 12;  // spare (logged to Serial only for now)

const int LED1_PIN = 2;   // heartbeat: brief flash on each received packet
const int LED2_PIN = 32;  // alert: solid on when the LoRa link is lost

// Built-in OLED (standard Heltec WiFi LoRa 32 V2 wiring)
const int OLED_SDA_PIN = 4;
const int OLED_SCL_PIN = 15;
const int OLED_RST_PIN = 16;
const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;

// LoRa radio (standard Heltec WiFi LoRa 32 V2 pins)
const int SPI_SCK_PIN = 5;
const int SPI_MISO_PIN = 19;
const int SPI_MOSI_PIN = 27;
const int LORA_CS_PIN = 18;
const int LORA_RST_PIN = 14;
const int LORA_DIO0_PIN = 26;
const long LORA_BAND = 915E6;

// ---------------------------------------------------------------------------------

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST_PIN);

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
  Serial.begin(115200);
  delay(100);
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

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("SSD1306 allocation failed");
    while (1)
      delay(1000);
  }
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(this_file);
  display.println(ver);
  display.display();

  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  if (!LoRa.begin(LORA_BAND))
  {
    Serial.println("Starting LoRa failed!");
    display.println("LoRa init failed!");
    display.display();
  }

  memset(&rxPacket, 0, sizeof(rxPacket));

  delay(1500);
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
  int packetSize = LoRa.parsePacket();
  if (packetSize != sizeof(rxPacket))
  {
    return;
  }

  LoRa.readBytes((uint8_t *)&rxPacket, sizeof(rxPacket));
  lastPacketTime = millis();
  alertMuted = false;  // a fresh, valid packet clears any prior mute

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

void drawScreen()
{
  display.clearDisplay();
  display.setCursor(0, 0);

  switch (currentScreen)
  {
  case SCREEN_LEVEL:
    display.println("Level");
    display.print("Nose: ");
    display.print(rxPacket.pitch, 1);
    display.println();
    display.print("Left: ");
    display.print(rxPacket.roll, 1);
    break;

  case SCREEN_TEMPS:
    display.println("Temps (F)");
    display.print("Fridge  ");
    display.println(rxPacket.temp1, 1);
    display.print("Freezer ");
    display.println(rxPacket.temp2, 1);
    display.print("Inside  ");
    display.println(rxPacket.temp3, 1);
    display.print("DC Cab  ");
    display.println(rxPacket.temp4, 1);
    break;

  case SCREEN_LINK:
    display.println("LoRa Link");
    if (lastPacketTime == 0)
    {
      display.println("No packet yet");
    }
    else
    {
      display.print("Last packet: ");
      display.print((millis() - lastPacketTime) / 1000);
      display.println("s ago");
      display.println(isLinkLost() ? "LINK LOST" : "OK");
    }
    display.print("RSSI: ");
    display.println(LoRa.packetRssi());
    break;
  }

  display.display();
}
