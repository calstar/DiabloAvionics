#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <DAQv2-Comms.h>
#include "STAR_ADS126X.h"
#include "main.h"
#include "adc_mappings.h"
#include "esp_mac.h"



// Change the following line to automatically use the correct pins for the board being tested (PT_Board, LC_Board, RTD_Board, or TC_Board)
#define PINS_ACTIVE_LAYOUT sense_board_pins::PT_Board

// These lines MUST be after the #define PINS_ACTIVE_LAYOUT or they will overwrite it with the default value!
#include "sense_board_pins.h"
#include "connector_adc_map.h"

using namespace sense_board_pins;

// Ethernet configuration
byte mac[6];  // Will be populated with unique MAC from ESP32 eFuse
IPAddress staticIP(192, 168, 2, 100);
IPAddress gateway(0, 0, 0, 0);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 2, 1);
IPAddress receiverIP(192, 168, 2, 20);
const int receiverPort = 5006;
EthernetUDP udp;

static ADS126X ads126x;
SPIClass ADC_SPI(HSPI);

// Forward declarations
void read_data(int count);
void flush_cycles(int cycles);
void sendSensorDataPacket();

// TODO: Implement channel cycling logic
static uint8_t nextChannel = ADS126X_AIN0;

// Sensor data collection
#define NUM_SENSORS 1  // We're reading from one ADC channel
#define MAX_CHUNKS 10  // Store up to 10 chunks before sending
std::vector<Diablo::SensorDataChunkCollection> dataChunks;
uint8_t sensorId = 0;  // Current sensor ID (channel)

float convert_code_to_voltage(int32_t code) {
  // Assumes the 2.5V internal reference is being used! 
  return ((float)code * 2.5f) / 2147483648.0f;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);  // Wait for native USB serial to connect
  }

  Serial.println("Starting ADC with Ethernet...");

  // Setup ADC SPI
  ADC_SPI.begin(Pins.ADC_SCLK, Pins.ADC_MISO, Pins.ADC_MOSI, Pins.ADC_CS_1);

  // You can set the freq if you want, or just use the default
  // SPI.setFrequency(1'000'000);

  // Due to the ADC output having valid data on FALLING CLK edges
  SPI.setDataMode(SPI_MODE1);
  pinMode(Pins.ADC_DRDY_1, INPUT);

  // Setup ADC
  ads126x.begin(Pins.ADC_CS_1, &ADC_SPI);

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

  // Generate unique MAC address from ESP32 eFuse (derived for Ethernet)
  ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_ETH));   // Derived from base eFuse MAC

  Serial.print("Generated unique MAC address: ");
  for (int i = 0; i < 6; i++) {
    if (i > 0) Serial.print(":");
    if (mac[i] < 0x10) Serial.print("0");
    Serial.print(mac[i], HEX);
  }
  Serial.println();

  // Setup Ethernet SPI
  Serial.println("Initializing Ethernet...");
  SPI.begin(Pins.ETH_SCLK, Pins.ETH_MISO, Pins.ETH_MOSI, Pins.ETH_CS);
  delay(1000);

  // Initialize Ethernet with CS pin
  Ethernet.init(Pins.ETH_CS);
  delay(1000);

  // Start Ethernet with static IP
  Ethernet.begin(mac, staticIP, dns, gateway, subnet);
  delay(1000);

  // Start UDP
  udp.begin(5005);

  // Print Ethernet status
  Serial.print("Ethernet initialized. IP: ");
  Serial.println(Ethernet.localIP());
  Serial.print("Link Status: ");
  if (Ethernet.linkStatus() == LinkON) {
    Serial.println("Connected");
  } else if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Disconnected");
  } else {
    Serial.println("Unknown");
  }

  Serial.println("Setup complete!");
}

void loop() {
  read_data(READINGS_PER_MUX);

  // Send packet if we have accumulated enough chunks
  if (dataChunks.size() >= MAX_CHUNKS) {
    sendSensorDataPacket();
    dataChunks.clear();
  }

  // Store some variable for the next channel to read
  // Increment to the next channel here 

  ads126x.setInputMux(nextChannel, ADS126X_AINCOM);

  flush_cycles(settlePulses(FILTER));
}

void read_data(int count) {
  // Create a new chunk for this set of readings
  Diablo::SensorDataChunkCollection chunk(millis(), NUM_SENSORS);
  
  for (int i = 0; i < count; i++) {
    // Wait for data 
    while(digitalRead(Pins.ADC_DRDY_1) != LOW) {
      delayMicroseconds(10);
    }

    // Get reading
    const auto reading = ads126x.readADC1();

    // Skip if bad checksum
    if (!reading.checksumValid) {
      Serial.println("Warning: Bad checksum");
      continue;
    }

    // Store the reading in the chunk
    // Using the raw ADC value (int32_t) directly
    chunk.add_datapoint(sensorId, static_cast<uint32_t>(reading.value));
    
    // Serial.print("ADC Reading: ");
    // Serial.println(reading.value);
  }

  // Add the chunk to our collection
  if (!chunk.empty()) {
    dataChunks.push_back(chunk);
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

void sendSensorDataPacket() {
  if (dataChunks.empty()) {
    return;
  }

  // Create packet buffer
  uint8_t packetBuffer[MAX_PACKET_SIZE];
  
  // Create the sensor data packet using DAQv2-Comms library
  size_t packetSize = Diablo::create_sensor_data_packet(
    dataChunks,
    NUM_SENSORS,
    packetBuffer, 
    sizeof(packetBuffer)
  );

  if (packetSize == 0) {
    Serial.println("Error: Failed to create sensor data packet");
    return;
  }

  // Send the packet via UDP Ethernet
  udp.beginPacket(receiverIP, receiverPort);
  udp.write(packetBuffer, packetSize);
  udp.endPacket();

  Serial.print("Sent sensor data packet: ");
  Serial.print(packetSize);
  Serial.print(" bytes, ");
  Serial.print(dataChunks.size());
  Serial.println(" chunks");
}