// adc.h
// Minimal interface for fast multi-channel ADS1263 reads on ESP32.

#ifndef ADC_H_
#define ADC_H_

#include <Arduino.h>
#include <ADS126X.h>
#include <SPI.h>

namespace adc {

// Call once at startup. Sets up SPI, pins, resets the ADC, waits for DRDY.
// Returns true if the ADC responds.
bool init();

// Programs reference, PGA or bypass, data rate, and run mode for fast scans
// 38.4 kSPS, Sinc5, CHOP off, pulse mode. Returns true on success.
bool configure();

// Read one channel once, blocking until DRDY.
// Returns signed 32-bit raw ADC code. On error, returns INT32_MIN.
int32_t read(uint8_t channel);

// Fixed-size version. Size inferred at compile time.
template<size_t N>
void read_batch(const uint8_t (&channels)[N], int32_t (&results)[N]) {
  for (size_t i = 0; i < N; ++i) {
    results[i] = read(channels[i]);
  }
}

// Dynamic-size fallback.
void read_batch(const uint8_t* channels, size_t n, int32_t* results);

} // namespace adc

#endif // ADC_H_
