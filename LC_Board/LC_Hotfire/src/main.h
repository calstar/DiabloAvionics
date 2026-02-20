#pragma once

//-----------------------------------------------------------------------------
// Timing and timeouts (all in milliseconds)
//-----------------------------------------------------------------------------
#define BOARD_HEARTBEAT_INTERVAL_MS  1000   // Send board heartbeat once per second
#define LOOP_DELAY_MS                 10    // Delay at end of each loop()
#define ETHERNET_SPI_DELAY_MS        1000   // Delay after SPI.begin() for Ethernet
#define ETHERNET_INIT_DELAY_MS       1000   // Delay after Ethernet.init()
#define ETHERNET_BEGIN_DELAY_MS      1000   // Delay after Ethernet.begin()

//-----------------------------------------------------------------------------
// ADC scan (user setting: reads per connector per chunk)
//-----------------------------------------------------------------------------
#define READINGS_PER_CONNECTOR      1   // Number of ADC reads per connector per chunk (use last valid)

//-----------------------------------------------------------------------------
// Board identity (SPIFFS)
//-----------------------------------------------------------------------------
#define SPIFFS_BOARD_VALUE_PATH     "/value.bin"
#define BOARD_ID_DEFAULT            1
