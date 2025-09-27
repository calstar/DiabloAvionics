#include <ADS126X.h>
#include <SPI.h>
#include <Arduino.h>
#include "board_pins.h"
#include "adcs.h"

// 1 = ASCII voltages for the Python calibration script
// 0 = packed binary records (original behavior)
#define TEXT_OUTPUT 0
#define PT_NUM_START 0
#define NUM_PTS 10

// --- Buffer settings ---
static const size_t PKT_MAX = 1000;   // packet size in bytes
static const uint32_t FLUSH_MS = 10;  // flush interval in ms

static uint8_t pktA[PKT_MAX];
static uint8_t pktB[PKT_MAX];
static uint8_t* active = pktA;
static uint8_t* standby = pktB;
static size_t used = 0;
static uint32_t lastFlushMs = 0;
inline void flushPacketIfNeeded(bool force = false);

#pragma pack(push, 1)
struct SampleRecord {
  uint32_t t_us;
  uint8_t  channel;
  int32_t  volt_reader;        // Raw ADC code (signed)
  float    voltage;            // Volts
  uint32_t read_time_us;
  float    samples_per_second;
  uint32_t sent_us;
};
#pragma pack(pop)

static constexpr float VREF_VOLTS = 2.5f;            // using internal 2.5 V
static constexpr float LSB_PER_V  = 2147483648.0f;   // 2^31, gain = 1

void setup() {
  Serial.begin(115200);
  SPI.begin(SCLK, MISO, MOSI, CS);

  if (!adcs::init()) { while (1) { delay(1000); } }                
  if (!adcs::configure()) { while (1) { delay(1000); } }           
  
}

void loop() {
  // Build the channel list [PT_NUM_START .. PT_NUM_START+NUM_PTS)
  uint8_t chs[NUM_PTS];
  for (uint8_t i = 0; i < NUM_PTS; ++i) chs[i] = PT_NUM_START + i;

  // One sweep
  adcs::AllResults sweep = adcs::read_all(chs, NUM_PTS);        

  if (TEXT_OUTPUT) {
    // print volts separated by spaces
    for (size_t i = 0; i < sweep.count; ++i) {
      const auto &s = sweep.samples[i];
      
      float volts = (float)s.result.raw * vRef / adcScale;
      Serial.print(volts, 6);
      if (i + 1 < sweep.count) Serial.print(' ');
    }
    Serial.print("\r\n");
  } else {
    // pack records
    for (size_t i = 0; i < sweep.count; ++i) {
      const auto &s = sweep.samples[i];

      // Time stamp per-sample: approximate with (total sweep start + per-read time isn’t stored),
      // use micros() at pack time for simplicity. If you want exact per-sample start, add it to ReadResult.
      uint32_t t_us = micros();

      float volts = (float)s.result.raw * VREF_VOLTS / LSB_PER_V;
      float sps   = s.result.read_time ? (1000000.0f / (float)s.result.read_time) : 0.0f;

      SampleRecord rec;
      rec.t_us              = t_us;
      rec.channel           = s.channel;
      rec.volt_reader       = s.result.raw;
      rec.voltage           = volts;
      rec.read_time_us      = s.result.read_time;
      rec.samples_per_second= sps;
      rec.sent_us           = 0;

      // ensure space, flush if needed
      if (used + sizeof(rec) > PKT_MAX) flushPacketIfNeeded(true);
      memcpy(active + used, &rec, sizeof(rec));
      used += sizeof(rec);

      // flush by time window
      flushPacketIfNeeded(false);
    }
  }
}
