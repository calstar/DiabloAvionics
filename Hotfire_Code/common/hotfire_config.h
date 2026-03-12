#pragma once

//-----------------------------------------------------------------------------
// Shared hotfire config — used by actuator_config.h and sense_config.h
// All hotfire boards (actuator, PT, TC, LC, RTD) share these.
//-----------------------------------------------------------------------------

// Heartbeat and loop
#define BOARD_HEARTBEAT_INTERVAL_MS  1000   // Send board heartbeat once per second
#define LOOP_DELAY_MS                 10    // Delay at end of each loop()

// Ethernet init delays (milliseconds)
#define ETHERNET_SPI_DELAY_MS        1000   // Delay after SPI.begin() for Ethernet
#define ETHERNET_INIT_DELAY_MS       1000   // Delay after Ethernet.init()
#define ETHERNET_BEGIN_DELAY_MS     1000   // Delay after Ethernet.begin()

// LED status blink (optional; actuator uses, sense boards may use)
#define LED_CYCLE_MS                 5000   // Cycle period for state-blink
#define LED_BLINK_ON_MS              100
#define LED_BLINK_OFF_MS             100

// Board identity (SPIFFS)
#define SPIFFS_BOARD_VALUE_PATH     "/value.bin"
#define BOARD_ID_DEFAULT            1

// Temporary: set to non-zero (1-254) to hardcode board ID and skip SPIFFS.
// Set to 0 to use SPIFFS (normal). Easy to undo: set back to 0.
#define TEMP_HARDCODE_BOARD_ID      31

// Server (all hotfire boards send heartbeats/data here; hardcoded, not updated from packets)
#define HOTFIRE_SERVER_IP_OCTET_4   20   // 192.168.2.20
#define HOTFIRE_SERVER_PORT         5006

// Sensor data: chunks per packet (all sense boards: PT, TC, LC, RTD)
#define HOTFIRE_CHUNKS_PER_PACKET   9

// OTA: TCP port all hotfire boards listen on for firmware updates
#define HOTFIRE_OTA_PORT            3232
