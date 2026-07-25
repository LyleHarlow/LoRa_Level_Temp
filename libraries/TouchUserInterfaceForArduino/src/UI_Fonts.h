

#ifndef UI_Fonts_h
#define UI_Fonts_h

#include <Arduino.h>

// Some ESP32 core versions don't expose PROGMEM to plain .c translation
// units (Arduino.h's pgmspace.h pull-in appears to be C++-only there),
// even though .cpp files see it fine. PROGMEM is a no-op on ESP32 anyway
// (flash is memory-mapped, unlike AVR), so an empty fallback is safe.
#ifndef PROGMEM
#define PROGMEM
#endif


#if defined(ARDUINO_ARCH_RP2040)
  extern const byte UI_Font_9[];
#else
  extern const PROGMEM byte UI_Font_9[];
#endif


#if defined(ARDUINO_ARCH_RP2040)
  extern const byte UI_Font_10[];
#else
  extern const PROGMEM byte UI_Font_10[];
#endif


#if defined(ARDUINO_ARCH_RP2040)
  extern const byte UI_Font_10_Bold[];
#else
  extern const PROGMEM byte UI_Font_10_Bold[];
#endif


#if defined(ARDUINO_ARCH_RP2040)
  extern const byte UI_Font_11[];
#else
  extern const PROGMEM byte UI_Font_11[];
#endif


#if defined(ARDUINO_ARCH_RP2040)
  extern const byte UI_Font_11_Bold[];
#else
  extern const PROGMEM byte UI_Font_11_Bold[];
#endif


#if defined(ARDUINO_ARCH_RP2040)
  extern const byte UI_Font_12_Bold[];
#else
  extern const PROGMEM byte UI_Font_12_Bold[];
#endif


#if defined(ARDUINO_ARCH_RP2040)
  extern const byte UI_Font_13[];
#else
  extern const PROGMEM byte UI_Font_13[];
#endif


#if defined(ARDUINO_ARCH_RP2040)
  extern const byte UI_Font_13_Bold[];
#else
  extern const PROGMEM byte UI_Font_13_Bold[];
#endif


#if defined(ARDUINO_ARCH_RP2040)
  extern const byte UI_Font_14[];
#else
  extern const PROGMEM byte UI_Font_14[];
#endif


#if defined(ARDUINO_ARCH_RP2040)
  extern const byte UI_Font_14_Bold[];
#else
  extern const PROGMEM byte UI_Font_14_Bold[];
#endif


#if defined(ARDUINO_ARCH_RP2040)
  extern const byte UI_Font_15[];
#else
  extern const PROGMEM byte UI_Font_15[];
#endif


#if defined(ARDUINO_ARCH_RP2040)
  extern const byte UI_Font_15_Bold[];
#else
  extern const PROGMEM byte UI_Font_15_Bold[];
#endif


#if defined(ARDUINO_ARCH_RP2040)
  extern const byte UI_Font_16_Bold[];
#else
  extern const PROGMEM byte UI_Font_16_Bold[];
#endif



#endif
