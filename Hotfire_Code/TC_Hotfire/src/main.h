#pragma once

#include "sense_config.h"

//-----------------------------------------------------------------------------
// TC-specific: ADC scan (readings per connector per chunk)
// Channel list comes from SENSOR_CONFIG
//-----------------------------------------------------------------------------
#define READINGS_PER_CONNECTOR          1

static const uint8_t TC_DEFAULT_SENSOR_IDS[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
#define TC_DEFAULT_NUM_SENSORS (sizeof(TC_DEFAULT_SENSOR_IDS) / sizeof(TC_DEFAULT_SENSOR_IDS[0]))
