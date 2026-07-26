# LoRa_Level_Temp

Code and Hardware Repo for LoRa Level and Temp Project

Two Heltec WiFi LoRa 32 V2 boards for monitoring an Airstream travel trailer:

- **Airstream** (`Tilt_Temp_AirStream/`) rides on the trailer. Reads tilt (MPU6050) for
  leveling, 4 DS18B20 temperature probes (fridge, freezer, inside the trailer, DC
  electrical cabinet), and a PCF8523 RTC, shows them on a touchscreen, and transmits them
  over LoRa.
- **TowVehicle** (`Tilt_Temp_TowVehc/`) rides in the cab. Receives that data over LoRa and
  shows it on its built-in OLED; 4 buttons cycle screens and mute the link-lost alert.

Schematics for both boards are in `hardware/`. `AxisOrientationTest/` is a standalone
diagnostic sketch for empirically determining the MPU6050's axis orientation once it's
physically mounted (see the comment block at the top of that file, and the corresponding
note in `Tilt_Temp_AirStream.ino`, before trusting the tilt readings).

## Setup

1. In the Arduino IDE, add Heltec's own board package: **File > Preferences > Additional
   Board Manager URLs**, add `https://resource.heltec.cn/download/package_heltec_esp32_index.json`,
   then install **Heltec ESP32 Series Dev-boards** via Boards Manager. Select board
   **"Heltec WiFi LoRa 32(V2)"** for both sketches.
2. Install these via **Library Manager** (Sketch > Include Library > Manage Libraries):
   - `Heltec ESP32 Dev-Boards` (by Heltec Automation) -- used by `Tilt_Temp_TowVehc` only
     (see Known hardware notes below for why `Tilt_Temp_AirStream` doesn't use it)
   - `Adafruit MPU6050`
   - `Adafruit Unified Sensor` (MPU6050 dependency)
   - `RTClib` (by Adafruit)
   - `Adafruit GFX Library`
   - `Adafruit ILI9341`
   - `XPT2046_Touchscreen` (by Paul Stoffregen)
   - `OneWire`
   - `DallasTemperature` (by Miles Burton et al.)
   - `LoRa` (by Sandeep Mistry) -- used by `Tilt_Temp_AirStream` only; `Tilt_Temp_TowVehc`
     uses the LoRa radio driver bundled in `Heltec ESP32 Dev-Boards` instead
3. `TouchUserInterfaceForArduino` (by Stan Reifel) isn't in the Library Manager index, so
   it's vendored in this repo under `libraries/TouchUserInterfaceForArduino/`. Copy that
   folder into your Arduino `libraries/` directory (or point your `sketchbook` at this
   repo's root, since Arduino IDE also looks in `<sketchbook>/libraries/`).
   Source: https://github.com/Stan-Reifel/TouchUserInterfaceForArduino

## Test-compiling without the Arduino IDE

Arduino IDE 2.x bundles its own `arduino-cli` at
`C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe`, and
already has a config file at `C:\Users\Harlow\.arduinoIDE\arduino-cli.yaml` with
`directories.user` pointed at this repo's root (same sketchbook location the IDE itself
uses) -- so it sees the same board packages and `libraries/` folder with no extra setup.
Compile either sketch with:
```
"C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" ^
  --config-file "C:\Users\Harlow\.arduinoIDE\arduino-cli.yaml" ^
  compile --fqbn Heltec-esp32:esp32:heltec_wifi_lora_32_V2 ^
  "Tilt_Temp_AirStream"
```
(swap the sketch folder for `Tilt_Temp_TowVehc`, `AxisOrientationTest`, or `LCDTest`). First
compile builds the whole ESP32 core and takes a few minutes; subsequent ones are much
faster. A standalone `arduino-cli` downloaded fresh (e.g. to a temp folder) may get blocked
by Windows Application Control policy -- the IDE's own bundled copy runs fine since it's
already an installed, trusted binary.

## Known hardware notes

- MPU6050 address is 0x69 (ADO jumpered to 3.3V) so it doesn't collide with the PCF8523
  RTC's fixed 0x68 address -- this was part of the root cause of an earlier "RTC and MPU6050
  don't work when both are plugged in" issue (the rest was likely the old code testing
  against a PCF8563-oriented library/example for a different chip than the actual PCF8523
  hardware). Both now share one I2C bus fine: `Wire.begin(SDA_PIN, SCL_PIN)` is called once
  in `setupTiltSensor()`, and each library's `begin()` is passed that same `&Wire` rather
  than being allowed to call its own `Wire.begin()` with default pins.
- The Airstream board's LCD/touchscreen SPI bus is intentionally shared with the onboard
  LoRa radio's SPI pins (SCK/MISO/MOSI), each device has its own CS pin.
- `Tilt_Temp_AirStream` intentionally does NOT use the `Heltec ESP32 Dev-Boards` library.
  That library's `Heltec.begin()` unconditionally drives the board's Vext pin (GPIO21) to
  power the onboard OLED -- but on the Airstream board GPIO21 is TOUCH_CS for the external
  touchscreen, so using it would stall touch input. `Tilt_Temp_TowVehc` doesn't use GPIO21
  for anything, so it uses the library normally (and gets proper Vext/OLED power-up for free,
  which the previous version of this sketch never explicitly handled).
- Temp probes are BOJACK DS18B20 (1M stainless, OneWire) -- one per pin, each with its own
  bus and pull-up resistor (the schematic's 3.3k), not a shared bus.

## Known bugs in Heltec ESP32 Dev-Boards v2.1.7 (needed for Tilt_Temp_TowVehc)

Three real upstream bugs in this library needed local patches to work with this project's
toolchain (GCC 14, from Heltec's ESP32 core 3.3.8 board package) and board (WiFi LoRa 32 V2).

**This library is gitignored** (Library Manager installs aren't tracked in this repo), so
both fixes below need to be reapplied any time the library is freshly installed/updated.

### 1. Compile error: `SpiInOut` implicit declaration

`src/driver/sx1276.c` calls `SpiInOut()` without a prototype in scope (the equivalent
`sx1262-board.c` has one; `sx1276.c` is just missing it upstream). Older GCC only warned
about this; GCC 14 makes it a hard error by default.

Add this line near the top of `sx1276.c` (after the existing `extern void lora_printf(...)`
declaration works as a landmark):
```c
extern uint8_t SpiInOut(Spi_t *obj, uint8_t outData);
```

### 2. Crash on `Heltec.begin()`: null-pointer dereference in `display->init()`

`src/heltec.h` defines the macro `Class_WIFI_LORA` (all-caps LORA) for WiFi LoRa 32 boards,
but `src/heltec.cpp`'s `Heltec_ESP32` constructor checks for a differently-cased
`Class_Wifi_LoRa` -- which is never actually defined, since preprocessor macros are
case-sensitive. That means `display` (the `SSD1306Wire*`) is never allocated, while
`Heltec_Screen` (which gates whether `begin()` calls `display->init()`) correctly checks
`Class_WIFI_LORA` and does fire -- calling `init()` through a null pointer, crashing with
`EXCVADDR: 0x00000014` (Guru Meditation, LoadProhibited).

In `heltec.cpp`, change the constructor's `#if` from:
```c
#if defined( Class_Wifi_Kit ) || defined( Class_Wifi_LoRa )
```
to:
```c
#if defined( Class_Wifi_Kit ) || defined( Class_WIFI_LORA )
```

### 3. Blank OLED, no errors anywhere: wrong constructor argument order

Even after fix #2, the OLED stayed completely blank (not even a brief flash of Heltec's
own boot splash), despite Serial showing every init call "succeeding." Cause:
`SSD1306Wire`'s constructor in `src/HT_SSD1306Wire.h` is
`(uint8_t address, uint32_t freq, int sda, int scl, DISPLAY_GEOMETRY g, int8_t rst)`, but
`heltec.cpp`'s constructor calls it as `SSD1306Wire(0x3c, SDA_OLED, SCL_OLED, RST_OLED,
GEOMETRY_128_64)` -- 5 positional args matching an older 5-arg signature (before `freq` was
apparently inserted as the 2nd parameter). That silently puts `SDA_OLED` into `freq`,
`SCL_OLED` into `sda`, and `RST_OLED` into `scl` -- so `Wire.begin()` runs on the wrong
physical pins entirely at a ~4Hz clock, while the real reset pin never gets passed at all.
Nothing in this path checks I2C ACKs, so every call reports success while nothing ever
reaches the actual display.

In `heltec.cpp`, change both `SSD1306Wire` constructor calls from:
```c
display = new SSD1306Wire(0x3c, SDA_OLED, SCL_OLED, RST_OLED, GEOMETRY_128_64);
...
display = new SSD1306Wire(0x3c, SDA_OLED, SCL_OLED, RST_OLED, GEOMETRY_64_32);
```
to:
```c
display = new SSD1306Wire(0x3c, 400000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
...
display = new SSD1306Wire(0x3c, 400000, SDA_OLED, SCL_OLED, GEOMETRY_64_32, RST_OLED);
```
