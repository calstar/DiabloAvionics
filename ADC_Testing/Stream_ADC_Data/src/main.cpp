#include <Arduino.h>
#include <SPI.h>
#include "STAR_ADS126X.h"
#include "main.h"
#include "adc_mappings.h"


// Change the following line to automatically use the correct pins for the board being tested (PT_Board, LC_Board, RTD_Board, or TC_Board)
#define PINS_ACTIVE_LAYOUT sense_board_pins::PT_Board

// These lines MUST be after the #define PINS_ACTIVE_LAYOUT or they will overwrite it with the default value!
#include "sense_board_pins.h"
#include "connector_adc_map.h"

using namespace sense_board_pins;

static ADS126X ads126x;

float convert_code_to_voltage(int32_t code) {
  // Assumes the 2.5V internal reference is being used! 
  return ((float)code * 2.5f) / 2147483648.0f;
}

void setup() {
  Serial.begin(115200);

  // Setup SPI
  SPI.begin(Pins.ADC_SCLK, Pins.ADC_MISO, Pins.ADC_MOSI, Pins.ADC_CS_1);

  // You can set the freq if you want, or just use the default
  // SPI.setFrequency(1'000'000);

  // Due to the ADC output having valid data on FALLING CLK edges
  SPI.setDataMode(SPI_MODE1);
  pinMode(Pins.ADC_DRDY_1, INPUT);

  // Setup ADC
  ads126x.begin(Pins.ADC_CS_1);

  // Stop it while we config it, as suggested by datasheet
  ads126x.stopADC1();

  // Set the input to ADC1 to be the whatever pin you want
  ads126x.setInputMux(ADS126X_AIN0, ADS126X_AINCOM);

  // Bypas the PGA, so it does not affect measurements 
  ads126x.bypassPGA();

  // Set the filter. You can change this to try different filters
  ads126x.setFilter(FILTER);

  // Set the datarate. You can change this, but the options depends on the filter
  // I do not know what happens if you program an invalid data rate for a given filter
  ads126x.setRate(DATA_RATE);

  // Start ADC now that configuration is done
  ads126x.startADC1();
}

void loop() {
  read_data(READINGS_PER_MUX);

  // Store some variable for the next channel to read
  // Increment to the next channel here 

  ads126x.setInputMux(nextChannel, ADS126X_AINCOM);

  flush_cycles(settlePulses(FILTER));

  // Implement ethernet buffer saving and sending
}

void read_data(int count) {
  for (int i = 0; i < count; i++) {
    // Wait for data 
    while(digitalRead(Pins.ADC_DRDY_1) != LOW) {
      delayMicroseconds(10);
    }

    // Get reading
    const auto reading = ads126x.readADC1();

    // Skip if bad checksum
    if (!reading.checksumValid) {
      continue;
    }

    // Do something here (like add it to the buffer idk man)
    reading.value
  }
}

void flush_cycles(int cycles) {
  for (int i = 0; i < cycles; i++) {
    // Wait for data 
    while(digitalRead(Pins.ADC_DRDY_1) != LOW) {
      delayMicroseconds(10);
    }
  }
}