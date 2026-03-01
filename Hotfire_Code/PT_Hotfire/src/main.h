#pragma once

#include "sense_config.h"

//-----------------------------------------------------------------------------
// PT-specific: ADC scan (readings per connector per chunk)
// Channel list comes from SENSOR_CONFIG; default when config has 0 sensors
//-----------------------------------------------------------------------------
#define READINGS_PER_MUX                5

// Default channels when legacy/empty config (connectors 1-10)
static const uint8_t PT_DEFAULT_SENSOR_IDS[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
#define PT_DEFAULT_NUM_SENSORS (sizeof(PT_DEFAULT_SENSOR_IDS) / sizeof(PT_DEFAULT_SENSOR_IDS[0]))
