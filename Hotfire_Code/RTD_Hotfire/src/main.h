#pragma once

#include "sense_config.h"

//-----------------------------------------------------------------------------
// RTD-specific: ADC scan (allow settling; match RTD_Testing)
// Channel list comes from SENSOR_CONFIG
//-----------------------------------------------------------------------------
#define READINGS_PER_CONNECTOR          5

static const uint8_t RTD_DEFAULT_SENSOR_IDS[] = { 1, 2 };
#define RTD_DEFAULT_NUM_SENSORS (sizeof(RTD_DEFAULT_SENSOR_IDS) / sizeof(RTD_DEFAULT_SENSOR_IDS[0]))
