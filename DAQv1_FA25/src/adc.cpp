// adc.cpp — matches the minimal adc.h API

#include "adc.h"
#include <Arduino.h>
#include <SPI.h>
#include <ADS126X.h>

// Pin definitions (adapt to your board)
static const int PIN_SCLK  = 13;
static const int PIN_MISO  = 41;
static const int PIN_MOSI  = 5;
static const int PIN_CS    = 37;
static const int PIN_DRDY  = 14;
static const int PIN_START = 43;

// ADS1263 commands / registers we need
static const uint8_t CMD_RREG   = 0x20;
static const uint8_t CMD_WREG   = 0x40;
static const uint8_t CMD_START1 = 0x08;
static const uint8_t REG_MODE0  = 0x03;

static const uint8_t MODE0_RUNMODE = 1 << 4; // 0=continuous, 1=pulse
static const uint8_t MODE0_CHOP    = 0x03;   // bits 1:0

// Driver instance
static ADS126X g_adc;

// Simple SPI read/write helpers for registers
static inline uint8_t readReg(uint8_t reg) {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(CMD_RREG | (reg & 0x1F));
  SPI.transfer(0x00);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return v;
}
static inline void writeReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(CMD_WREG | (reg & 0x1F));
  SPI.transfer(0x00);
  SPI.transfer(val);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}
static inline bool wait_drdy_low(uint32_t timeout_us) {
  uint32_t t0 = micros();
  while (digitalRead(PIN_DRDY) == HIGH) {
    if ((micros() - t0) > timeout_us) return false;
  }
  return true;
}

namespace adc {

bool init() {
  pinMode(PIN_CS, OUTPUT);    digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_DRDY, INPUT);
  pinMode(PIN_START, OUTPUT); digitalWrite(PIN_START, LOW);

  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  g_adc.begin(PIN_CS);

  // Basic sanity read: if SPI is up, we should be able to read MODE0
  (void)readReg(REG_MODE0);
  return true;
}

bool configure() {
  // Reference and rate for fast scanning
  g_adc.enableInternalReference();          // use 2.5 V internal ref
  g_adc.setRate(ADS126X_RATE_38400);        // ADC1 38.4 kSPS
  // g_adc.bypassPGA(); // uncomment if your sensor range needs PGA bypass

  // Put ADC1 in pulse (one-shot) mode, CHOP off
  uint8_t m0 = readReg(REG_MODE0);
  m0 = (m0 | MODE0_RUNMODE) & ~MODE0_CHOP;  // RUNMODE=1, CHOP=00
  writeReg(REG_MODE0, m0);

  return true;
}

int32_t read(uint8_t channel) {
  // Select channel vs AINCOM (0x0A)
  g_adc.setMux(channel, 0x0A);

  // Optional: allow external RC to settle if needed
  // delayMicroseconds(20);

  // One-shot conversion in pulse mode: START1, wait DRDY, read data
  // Using library helper ensures START1 is issued
  g_adc.startADC1();

  if (!wait_drdy_low(50000)) { // 50 ms timeout
    return INT32_MIN;
  }

  // Read conversion result
  // If your lib has a direct "readData()", prefer that; readADC1 will also work
  return static_cast<int32_t>(g_adc.readData());
}

void read_batch(const uint8_t* channels, size_t n, int32_t* results) {
  for (size_t i = 0; i < n; ++i) {
    results[i] = read(channels[i]);
  }
}

} // namespace adc
