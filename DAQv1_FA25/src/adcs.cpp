// adc.cpp
// Abstraction for adc library

#include "adcs.h"

#include <Arduino.h>
#include "board_pins.h"

static ADS126X ads;

namespace adcs {
// -----------------------------------------------------------------------------
// init
// -----------------------------------------------------------------------------
bool init() {
    // Pins
    pinMode(START, OUTPUT);
    pinMode(DRDY, INPUT);
    digitalWrite(START, LOW); // START will stay low forever as START1 cmd over SPI will be used instead

    ads.begin(CS);
    ads.enableInternalReference(); // 2.5V
    delay(50); // Delay for settling

    return true;
}

// -----------------------------------------------------------------------------
// configure
//   Default: 38.4 kSPS, SINCn ignored at this rate, CHOP off, pulse mode
//   Also runs self-offset calibration at the end.
// -----------------------------------------------------------------------------
bool configure(uint8_t negativeReference,
               uint8_t positiveReference,
               bool bypassPGA,
               uint8_t gainPGA,
               uint8_t chopMode,
               uint8_t sincFilter,
               uint8_t rate 
               )   
{
    
    if (negativeReference != REF_SKIP && positiveReference != REF_SKIP) {
        ads.setReference(negativeReference, positiveReference);
    }
    if (bypassPGA) {
        ads.bypassPGA(); // COMMENT OUT
    }
    else if (!bypassPGA && gainPGA) {
        ads.enablePGA();
        ads.setGain(gainPGA);
    }
    else {
        ads.bypassPGA();
    }
    ads.setChopMode(chopMode);


    if (sincFilter != FILT_SKIP) {
        ads.setFilter(sincFilter);
    } 
    ads.setRate(rate);
    ads.setPulseMode();
    ads.calibrateSelfOffsetADC1();

    return true;
}

ADS126X& device() {
    return ads;
}


void setChannel(uint8_t channel) {
    ads.readADC1(channel, neg_pin);
}



ReadResult read(uint8_t channel) {
    uint32_t read_start = micros();
    adcs::setChannel(channel);
    uint32_t conv_start = micros();
    ads.startADC1();

    // Data Ready
    uint32_t t_wait = millis();
    while (digitalRead(DRDY_PIN) == HIGH) {
        if (millis() - t_wait > 100) {       // 100 ms timeout guard
        return {INT32_MIN, 0u, 0u, false};
        }
    }
    uint32_t conv_time = micros() - conv_start;
    
    
    long raw = ads.readADC1(channel, neg_pin);
    


    uint32_t read_time = micros() - read_start;

    return ReadResult{raw, read_time, conv_time, true};
}

AllResults readAll(const uint8_t* channels, size_t num_channels) {
    AllResults results{};
    const uint32_t read_start = micros();

    const size_t results_size = (num_channels <= MAX_CHANNELS) ? num_channels : MAX_CHANNELS;
    results.count = results_size;

    // Sweep channels

    for (size_t i = 0; i < results_size; ++i) {
        const uint8_t ch = channels[i];
        results.samples[i].channel = ch;
        results.samples[i].result = read(ch);
        if (!results.samples[i].result.ok) {
            results.failures++;
        }
    }

    results.total_time = micros() - read_start;

    return results;
}



}


