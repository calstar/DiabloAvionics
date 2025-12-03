#pragma once
#include "STAR_ADS126X.h"

// From page 64 of ADS126X datasheet
// SINC5 not supported (requires refactor)
constexpr uint8_t baseSettlePulses(uint8_t f)
{
    switch (f) {
        case ADS126X_FIR:   return 1;
        case ADS126X_SINC1: return 1;
        case ADS126X_SINC2: return 2;
        case ADS126X_SINC3: return 3;
        case ADS126X_SINC4: return 4;
        default:            return 1;    // safe fallback
    }
}

constexpr uint8_t settlePulses(uint8_t f,
                               bool chop_enabled = false,
                               bool idac_rotation_enabled = false)
{
    uint8_t n = baseSettlePulses(f);

    // Either one being enabled double conversion time
    // It seems unlikely that either will be used, but shrug 
    if (chop_enabled)          n *= 2;
    if (idac_rotation_enabled) n *= 2;

    return n;
}
