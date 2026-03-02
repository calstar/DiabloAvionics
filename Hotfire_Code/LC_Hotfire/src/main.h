#pragma once

#include "sense_config.h"

//-----------------------------------------------------------------------------
// LC-specific: ADC scan (readings per connector per chunk)
// Channel list comes from SENSOR_CONFIG. LC ADC1 only: connectors 1,2,3,6,7.
//-----------------------------------------------------------------------------
#define READINGS_PER_CONNECTOR          1

static const uint8_t LC_DEFAULT_SENSOR_IDS[] = { 1, 2, 3, 6, 7 };
#define LC_DEFAULT_NUM_SENSORS (sizeof(LC_DEFAULT_SENSOR_IDS) / sizeof(LC_DEFAULT_SENSOR_IDS[0]))
