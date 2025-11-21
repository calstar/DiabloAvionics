#pragma once
#include <Arduino.h>

class NoiseAnalyzer {
public:
    static const size_t WINDOW_SIZE = 100;   // last 100 samples

    NoiseAnalyzer();

    // push a new raw ADC code (signed or unsigned, doesn't matter)
    void addSample(int32_t code);

    // true once we've collected WINDOW_SIZE samples
    bool ready() const { return _count >= WINDOW_SIZE; }

    // metric 1: RMS noise in LSB (i.e. std dev of codes)
    float rmsNoiseLsb() const;

    // metric 2: noise-free bits, given ADC resolution (in bits)
    float noiseFreeBits(uint8_t adcBits) const;

private:
    int32_t _buf[WINDOW_SIZE];
    size_t  _count;
    size_t  _index;
};
