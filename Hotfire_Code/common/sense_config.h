#pragma once

#include "hotfire_config.h"

//-----------------------------------------------------------------------------
// Sense hotfire (PT / TC / LC / RTD) — timing and data collection
// Same structure as actuator_config.h; board-specific values can override in main.h.
//-----------------------------------------------------------------------------

// Server heartbeat timeout (optional; sensor core may use for connection loss)
#define HEARTBEAT_TIMEOUT_MS              5000   // Server heartbeat watchdog

// Sensor data chunking: max chunks buffered before send
#define SENSOR_MAX_CHUNKS_BEFORE_SEND     9
