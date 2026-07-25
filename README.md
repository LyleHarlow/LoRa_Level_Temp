# LoRa_Level_Temp

Code and Hardware Repo for LoRa Level and Temp Project

Two Heltec WiFi LoRa 32 V2 boards for monitoring an Airstream travel trailer:

- **Airstream** (`Tilt_Temp_AirStream/`) rides on the trailer. Reads tilt (MPU6050) for
  leveling and 4 temperature probes (fridge, freezer, inside the trailer, DC electrical
  cabinet), shows them on a touchscreen, and transmits them over LoRa.
- **TowVehicle** (`Tilt_Temp_TowVehc/`) rides in the cab. Receives that data over LoRa and
  shows it on its built-in OLED; 4 buttons cycle screens and mute the link-lost alert.

Schematics for both boards are in `hardware/`. `AxisOrientationTest/` is a standalone
diagnostic sketch for empirically determining the MPU6050's axis orientation once it's
physically mounted (see the comment block at the top of that file, and the corresponding
note in `Tilt_Temp_AirStream.ino`, before trusting the tilt readings).

## Setup

1. In the Arduino IDE, add the ESP32 board package: **File > Preferences > Additional Board
   Manager URLs**, add `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`,
   then install **esp32 by Espressif Systems** via Boards Manager. Select a Heltec WiFi LoRa
   32 (V2) / "ESP32 Dev Module" board.
2. Install these via **Library Manager** (Sketch > Include Library > Manage Libraries):
   - `Adafruit MPU6050`
   - `Adafruit Unified Sensor` (MPU6050 dependency)
   - `Adafruit GFX Library`
   - `Adafruit ILI9341`
   - `Adafruit SSD1306`
   - `XPT2046_Touchscreen` (by Paul Stoffregen)
   - `LoRa` (by Sandeep Mistry)
3. `TouchUserInterfaceForArduino` (by Stan Reifel) isn't in the Library Manager index, so
   it's vendored in this repo under `libraries/TouchUserInterfaceForArduino/`. Copy that
   folder into your Arduino `libraries/` directory (or point your `sketchbook` at this
   repo's root, since Arduino IDE also looks in `<sketchbook>/libraries/`).
   Source: https://github.com/Stan-Reifel/TouchUserInterfaceForArduino

## Known hardware notes

- MPU6050 address is 0x69 (ADO jumpered to 3.3V) so it doesn't collide with the PCF8523
  RTC's fixed 0x68 address -- this was the root cause of an earlier "RTC and MPU6050 don't
  work when both are plugged in" issue.
- The Airstream board's LCD/touchscreen SPI bus is intentionally shared with the onboard
  LoRa radio's SPI pins (SCK/MISO/MOSI), each device has its own CS pin.
- NTC thermistor conversion constants in `Tilt_Temp_AirStream.ino` (`NOMINAL_RESISTANCE`,
  `B_COEFFICIENT`) are placeholders and need calibrating against the actual probe part.
