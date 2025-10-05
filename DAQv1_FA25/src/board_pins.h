// board_pins.h
#pragma once
#include <stdint.h>
#include <stddef.h>

// ---------- Pins (prefer constants over macros) ----------
inline constexpr int DRDY_PIN = 14;
inline constexpr int MOSIp     = 5;
inline constexpr int MISOp     = 41;
inline constexpr int SCLK     = 13;
inline constexpr int CS       = 37;
inline constexpr int START    = 43;

// Optional legacy aliases if some code still uses these old names.
// Undef first to avoid redefinition warnings if included elsewhere.
#ifdef DRDY
#undef DRDY
#endif
#define DRDY DRDY_PIN

// ---------- ADC config ----------
inline uint8_t neg_pin = 0x0A;              // AINCOM
inline float   vRef    = 2.5f;
inline float   adcScale = 2147483648.0f;     // 2^31, signed full-scale

// ---------- Channels ----------
inline uint8_t channels[] = {2, 3};
inline size_t  num_channels = sizeof(channels) / sizeof(channels[0]);

/*
Old (Module 8) defines were duplicate/conflicting:
  #define DOUT 41 // MISO
  #define DIN 5   // MOSI
  #define SCLK 13
  #define CS 37
  #define START 43
Use the constants above instead. If you must keep them for legacy code:

#ifdef DOUT
#undef DOUT
#endif
#define DOUT MISO

#ifdef DIN
#undef DIN
#endif
#define DIN MOSI
*/
