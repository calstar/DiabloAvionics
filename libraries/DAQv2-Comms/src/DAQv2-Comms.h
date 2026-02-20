#pragma once

#include <Arduino.h>

// Version information
#define DIABLO_COMMS_VERSION 0 // Protocol version (uint8_t)

// Maximum values
#define MAX_SENSORS_PER_BOARD 10
#define MAX_ACTUATORS_PER_BOARD 10
#define MAX_CHUNKS_PER_PACKET 10
#define MAX_PACKET_SIZE 512

// Hardcoded parameters
#define NUM_ABORT_ACTUATOR_LOCATIONS 4
#define NUM_ABORT_PT_LOCATIONS 4

// Include all other headers
#include "DiabloEnums.h"
#include "DiabloPackets.h"
#include "DiabloPacketUtils.h"


