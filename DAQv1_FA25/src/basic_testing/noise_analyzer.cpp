#include "noise_analyzer.h"
#include <math.h>

NoiseAnalyzer::NoiseAnalyzer()
    : _count(0), _index(0) {
    for (size_t i = 0; i < WINDOW_SIZE; ++i) {
        _buf[i] = 0;
    }
}

void NoiseAnalyzer::addSample(int32_t code) {
    _buf[_index] = code;
    _index = (_index + 1) % WINDOW_SIZE;
    if (_count < WINDOW_SIZE) _count++;
}

float NoiseAnalyzer::rmsNoiseLsb() const {
    if (_count < 2) return NAN;

    // mean of the window
    double mean = 0.0;
    for (size_t i = 0; i < _count; ++i) {
        mean += (double)_buf[i];
    }
    mean /= (double)_count;

    // variance = mean of squared deviation from mean
    double var = 0.0;
    for (size_t i = 0; i < _count; ++i) {
        double d = (double)_buf[i] - mean;
        var += d * d;
    }
    var /= (double)(_count - 1);   // unbiased estimate

    return (float)sqrt(var);       // in LSBs, because input is codes
}

float NoiseAnalyzer::noiseFreeBits(uint8_t adcBits) const {
    if (!ready()) return NAN;

    float rms = rmsNoiseLsb();
    if (!isfinite(rms) || rms <= 0.0f) {
        return adcBits;  // effectively no measurable noise
    }

    // Approximate peak-to-peak noise for Gaussian: ~6.6 × RMS
    // Full-scale range in codes = 2^adcBits
    // N_noise_free = log2(FS / noise_pp) = B - log2(6.6 * RMS_LSB)
    double nfb = (double)adcBits - log(6.6 * (double)rms) / log(2.0);

    if (nfb < 0.0) nfb = 0.0;
    return (float)nfb;
}
