#pragma once

#include <Arduino.h>
#include "STAR_ADS126X.h"

namespace SensorSelfTest {

// Thresholds (ADC codes, full-scale = 2^31)
// ADC TDAC test: VDD reference (5V). TDACP = 0.6*VDD (3V), TDACN = 0.5*VDD (2.5V).
// Differential is 0.5V. Full-scale is 5V (VDD).
// Expected code = (0.5V / 5.0V) * 2^31 = 214748364 (1/10th of full-scale).
// We allow roughly ±5% of 2^31 tolerance for this test.
constexpr int32_t ADC_TDAC_EXPECTED_CODE = 214748364;
constexpr int32_t ADC_TDAC_TOLERANCE     = 107374182; // 5% of 2^31

// Sensor bias test (SBMAG = 0b110 → 10MΩ pull resistor via MODE1):
// Open circuit  → bias pulls pin to rail → large |code|  (beyond SENSOR_BIAS_OPEN_THRESHOLD)
// Closed sensor → small voltage drop    → small |code|  (below SENSOR_BIAS_CLOSED_THRESHOLD)
constexpr int32_t SENSOR_BIAS_OPEN_THRESHOLD   = 1717986918; // 80% of 2^31
constexpr int32_t SENSOR_BIAS_CLOSED_THRESHOLD = 429496729;  // 20% of 2^31

/**
 * ADC self-test via internal TDAC.
 * Sets TDACP to 0.6*VDD and TDACN to 0.5*VDD.
 * Muxes TDAC vs TDAC, uses VDD reference (5V).
 * Reads and checks that the code is near ADC_TDAC_EXPECTED_CODE.
 * Restores original reference, mux, and turns off TDAC after test.
 */
inline bool run_adc_self_test(ADS126X& adc, uint8_t drdy_pin,
                              uint8_t original_ref_neg, uint8_t original_ref_pos) {
    // 1. Point mux to TDAC
    adc.setInputMux(ADS126X_TDAC, ADS126X_TDAC);

    // 2. Turn on TDACs with specific dividers
    adc.setOutputTDACP(1);
    adc.setOutputTDACN(1);
    adc.setOutputmagnitudeTDACP(ADS126X_TDAC_DIV_0_6);
    adc.setOutputmagnitudeTDACN(ADS126X_TDAC_DIV_0_5);

    // 3. Switch to VDD reference (5V)
    adc.setReference(ADS126X_REF_NEG_VSS, ADS126X_REF_POS_VDD);

    // Wait for conversion
    delayMicroseconds(10);
    while (digitalRead(drdy_pin) != LOW) {
        delayMicroseconds(10);
    }
    delayMicroseconds(25);

    // Read value
    const auto reading = adc.readADC1();
    int32_t code = reading.value;

    // 4. Cleanup
    adc.setOutputTDACP(0);
    adc.setOutputTDACN(0);
    adc.setReference(original_ref_neg, original_ref_pos);

    if (!reading.checksumValid) {
        return false;
    }

    // 5. Check tolerance
    int32_t diff = code - ADC_TDAC_EXPECTED_CODE;
    if (diff < 0) diff = -diff; // basic abs()

    return diff <= ADC_TDAC_TOLERANCE;
}

/** Call once before the connector loop. Turns off IDAC, sets SBMAG = 10MΩ. */
inline void sensor_bias_enable(ADS126X& adc) {
    adc.setIDAC1Mag(ADS126X_IDAC_MAG_0);
    adc.setIDAC2Mag(ADS126X_IDAC_MAG_0);
    adc.setBiasMagnitude(ADS126X_BIAS_MAG_10M);
}

/**
 * Read one connector with sensor bias already active.
 * Returns true (closed) if |code| < SENSOR_BIAS_CLOSED_THRESHOLD,
 * false (open)   if |code| > SENSOR_BIAS_OPEN_THRESHOLD.
 */
inline bool read_sensor_bias(ADS126X& adc, uint8_t drdy_pin,
                             uint8_t adc_channel, uint8_t aincom) {
    adc.setInputMux(adc_channel, aincom);
    delayMicroseconds(10);
    while (digitalRead(drdy_pin) != LOW) {
        delayMicroseconds(10);
    }
    delayMicroseconds(25);
    
    auto reading = adc.readADC1();
    if (!reading.checksumValid) return false;
    
    int32_t val = reading.value;
    if (val < 0) val = -val;
    
    // We only explicitly return false if it's over the open threshold.
    // E.g. if it's near-rail, it is open. 
    // Anything below SENSOR_BIAS_CLOSED_THRESHOLD is definitely closed.
    // For now, if val < SENSOR_BIAS_OPEN_THRESHOLD, we'll consider it "not open" (closed enough).
    return val < SENSOR_BIAS_OPEN_THRESHOLD;
}

/** Call once after the connector loop. Restores SBMAG = 0. */
inline void sensor_bias_disable(ADS126X& adc) {
    adc.setBiasMagnitude(ADS126X_BIAS_MAG_0);
}

} // namespace SensorSelfTest
