#pragma once

#include <Arduino.h>
#include "STAR_ADS126X.h"

namespace SensorSelfTest {

// Thresholds (ADC codes, full-scale = 2^31)
// ADC TDAC test: VDD reference. TDACP = 0.6*AVDD, TDACN = 0.5*AVDD.
// Differential = 0.1*AVDD. Reference = AVDD. So code = 0.1 * 2^31.
// We allow ±1% of the expected code.
constexpr int32_t ADC_TDAC_EXPECTED_CODE = 214748364;           // 0.1 * 2^31
constexpr int32_t ADC_TDAC_TOLERANCE     = 2147484;            // 1% of expected code

// Sensor bias test (SBMAG = 0b110 → 10MΩ pull resistor via MODE1):
// Open circuit  → bias pulls pin to rail → large |code|
// Connected     → sensor loads the bias  → small |code|
// Three-state thresholds:
//   |code| < 20% FS  → CONNECTED (sensor clearly loads the bias)
//   |code| > 80% FS  → DISCONNECTED (pin pulled to rail)
//   between          → AMBIGUOUS
constexpr int32_t SENSOR_BIAS_CLOSED_THRESHOLD = 429496729;  // 20% of 2^31
constexpr int32_t SENSOR_BIAS_OPEN_THRESHOLD   = 1717986918; // 80% of 2^31

enum class BiasResult : uint8_t {
  CONNECTED    = 0,  // |code| < closed threshold — sensor is present
  AMBIGUOUS    = 1,  // between thresholds — uncertain
  DISCONNECTED = 2,  // |code| > open threshold — no sensor
};

/**
 * ADC self-test via internal TDAC.
 * Sets TDACP to 0.6*VDD and TDACN to 0.5*VDD.
 * Muxes TDAC vs TDAC, uses VDD reference (5V).
 * Reads and checks that the code is near ADC_TDAC_EXPECTED_CODE.
 * Restores original reference, mux, and turns off TDAC after test.
 */
inline bool run_adc_self_test(ADS126X& adc, uint8_t drdy_pin,
                              uint8_t original_ref_neg, uint8_t original_ref_pos) {
    // 1. Bypass PGA (datasheet recommends this for TDAC tests)
    adc.bypassPGA();

    // 2. Point mux to TDAC
    adc.setInputMux(ADS126X_TDAC, ADS126X_TDAC);

    // 3. Turn on TDACs with specific dividers
    adc.setOutputTDACP(1);
    adc.setOutputTDACN(1);
    adc.setOutputmagnitudeTDACP(ADS126X_TDAC_DIV_0_6);
    adc.setOutputmagnitudeTDACN(ADS126X_TDAC_DIV_0_5);

    // 4. Switch to VDD reference
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

    // 5. Cleanup: restore PGA, TDAC off, reference
    adc.enablePGA();
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
 * Returns CONNECTED if |code| < SENSOR_BIAS_CLOSED_THRESHOLD,
 * DISCONNECTED if |code| >= SENSOR_BIAS_OPEN_THRESHOLD,
 * AMBIGUOUS otherwise (or on checksum failure).
 */
inline BiasResult read_sensor_bias(ADS126X& adc, uint8_t drdy_pin,
                                   uint8_t adc_channel, uint8_t aincom) {
    adc.setInputMux(adc_channel, aincom);
    delayMicroseconds(10);
    while (digitalRead(drdy_pin) != LOW) {
        delayMicroseconds(10);
    }
    delayMicroseconds(25);
    
    auto reading = adc.readADC1();
    if (!reading.checksumValid) return BiasResult::AMBIGUOUS;
    
    int32_t val = reading.value;
    if (val < 0) val = -val;
    
    if (val < SENSOR_BIAS_CLOSED_THRESHOLD) return BiasResult::CONNECTED;
    if (val >= SENSOR_BIAS_OPEN_THRESHOLD)  return BiasResult::DISCONNECTED;
    return BiasResult::AMBIGUOUS;
}

/** Call once after the connector loop. Restores SBMAG = 0. */
inline void sensor_bias_disable(ADS126X& adc) {
    adc.setBiasMagnitude(ADS126X_BIAS_MAG_0);
}

} // namespace SensorSelfTest
