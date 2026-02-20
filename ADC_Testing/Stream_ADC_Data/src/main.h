#include "STAR_ADS126X.h"

#define READINGS_PER_CONNECTOR 5  // Readings per connector per chunk (allows settling)
#define FILTER ADS126X_SINC4
#define DATA_RATE ADS126X_RATE_7200

// ADC Reference Configuration
// Selects which voltage the ADS126X uses as its reference for ADC conversions.
// 0 = Internal 2.5V reference (codes convert as: voltage = code * 2.5 / 2^31)
// 1 = VDD (power supply, typically 3.3V or 5V). Receiver must know VDD to convert codes to volts.
// Note: Raw codes are sent over the wire; reference is NOT in the packet.
#define USE_VDD_REFERENCE 1
