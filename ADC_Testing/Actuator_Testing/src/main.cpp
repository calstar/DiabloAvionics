#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <DAQv2-Comms.h>
#include <cstring>
#include <esp_mac.h>
#include "actuator_board_pins.h"

using namespace actuator_board_pins;

// Configuration
const unsigned long ADC_READ_INTERVAL_MS = 1000;  // Read ADC every 100ms (adjust as needed)

// Ethernet configuration
byte mac[6];  // Will be populated with unique MAC from ESP32 eFuse
IPAddress staticIP(192, 168, 2, 100);
IPAddress gateway(0, 0, 0, 0);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 2, 1);
const int udpPort = 5005;  // Port to listen for actuator commands
IPAddress receiverIP(192, 168, 2, 20);  // IP address to send sensor data to
const int receiverPort = 5006;  // Port to send sensor data to
EthernetUDP udp;

// Actuator state tracking
// Store the current state of each actuator (0 = OFF, 1 = ON)
uint8_t actuator_states[NUM_ACTUATORS];

// Sensor data collection
const uint8_t NUM_SENSORS = NUM_ACTUATORS;  // 10 current sense pins
unsigned long lastAdcReadTime = 0;

// Forward declarations
void initializeActuators();
void processActuatorCommand(const std::vector<Diablo::ActuatorCommand> &commands);
void readCurrentSensePins();
void sendSensorDataPacket(const Diablo::SensorDataChunkCollection &chunk);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);  // Wait for native USB serial to connect
  }

  Serial.println("Starting Actuator Testing with Ethernet...");

  // Initialize all actuators to OFF state
  initializeActuators();

  // Initialize current sense pins as analog inputs
  Serial.println("Initializing current sense pins...");
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    int pin = getCurrentSensePin(i);
    if (pin >= 0) {
      pinMode(pin, INPUT);
      Serial.print("Current sense sensor ");
      Serial.print(i);
      Serial.print(" (GPIO ");
      Serial.print(pin);
      Serial.println(") initialized");
    }
  }

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
  SPI.begin(Actuator_Board.ETH_SCLK, Actuator_Board.ETH_MISO, Actuator_Board.ETH_MOSI, Actuator_Board.ETH_CS);
  delay(1000);

  // Initialize Ethernet with CS pin
  Ethernet.init(Actuator_Board.ETH_CS);
  delay(1000);

  // Start Ethernet with static IP
  Ethernet.begin(mac, staticIP, dns, gateway, subnet);
  delay(1000);

  // Start UDP
  udp.begin(udpPort);

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

  Serial.print("Listening for actuator commands on UDP port: ");
  Serial.println(udpPort);
  Serial.println("Setup complete!");
}

void loop() {
  // Check for incoming UDP packets
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    // Read the packet into a buffer
    uint8_t packetBuffer[MAX_PACKET_SIZE];
    int bytesRead = udp.read(packetBuffer, MAX_PACKET_SIZE);
    
    if (bytesRead > 0) {
      // Parse the packet as an actuator command
      Diablo::PacketHeader header;
      std::vector<Diablo::ActuatorCommand> commands;
      
      if (Diablo::parse_actuator_command_packet(packetBuffer, bytesRead, header, commands)) {
        Serial.print("Received actuator command packet with ");
        Serial.print(commands.size());
        Serial.println(" commands");
        
        // Process the actuator commands
        processActuatorCommand(commands);
      } else {
        Serial.println("Error: Failed to parse actuator command packet");
      }
    }
  }

  // Read current sense pins and send sensor data at regular intervals
  unsigned long currentTime = millis();
  if (currentTime - lastAdcReadTime >= ADC_READ_INTERVAL_MS) {
    lastAdcReadTime = currentTime;
    readCurrentSensePins();
  }
}

void initializeActuators() {
  Serial.println("Initializing actuators...");
  
  // Set all actuator pins as outputs and write LOW (OFF state)
  // Actuators are 1-indexed (1-10)
  for (int i = 1; i <= NUM_ACTUATORS; i++) {
    int pin = getActuatorPin(i);
    if (pin < 0) {
      Serial.print("Warning: Invalid actuator ID ");
      Serial.println(i);
      continue;
    }
    
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    actuator_states[i - 1] = 0;  // Store OFF state (0-indexed array)
    Serial.print("Actuator ");
    Serial.print(i);
    Serial.print(" (GPIO ");
    Serial.print(pin);
    Serial.println(") set to OFF");
  }
  
  Serial.println("All actuators initialized to OFF state");
}

void processActuatorCommand(const std::vector<Diablo::ActuatorCommand> &commands) {
  for (const auto &cmd : commands) {
    // Actuator IDs are 1-indexed (1-10) from the server
    // Validate actuator ID
    if (cmd.actuator_id < 1 || cmd.actuator_id > NUM_ACTUATORS) {
      Serial.print("Warning: Invalid actuator ID: ");
      Serial.println(cmd.actuator_id);
      continue;
    }
    
    // Get the GPIO pin for this actuator (1-indexed)
    int pin = getActuatorPin(cmd.actuator_id);
    if (pin < 0) {
      Serial.print("Warning: Failed to get pin for actuator ID: ");
      Serial.println(cmd.actuator_id);
      continue;
    }
    
    // Determine the state (0 = OFF, non-zero = ON)
    uint8_t new_state = (cmd.actuator_state != 0) ? 1 : 0;
    
    // Convert 1-indexed actuator ID to 0-indexed array index
    uint8_t array_index = cmd.actuator_id - 1;
    
    // Update the actuator if the state has changed
    if (actuator_states[array_index] != new_state) {
      digitalWrite(pin, new_state ? HIGH : LOW);
      actuator_states[array_index] = new_state;
      
      Serial.print("Actuator ");
      Serial.print(cmd.actuator_id);
      Serial.print(" (GPIO ");
      Serial.print(pin);
      Serial.print(") set to ");
      Serial.println(new_state ? "ON" : "OFF");
    } else {
      Serial.print("Actuator ");
      Serial.print(cmd.actuator_id);
      Serial.println(" already in requested state");
    }
  }
}

void readCurrentSensePins() {
  // Create a sensor data chunk with timestamp
  Diablo::SensorDataChunkCollection chunk(millis(), NUM_SENSORS);
  
  // Read all current sense pins
  for (uint8_t sensor_id = 0; sensor_id < NUM_SENSORS; sensor_id++) {
    int pin = getCurrentSensePin(sensor_id);
    if (pin < 0) {
      Serial.print("Warning: Invalid current sense sensor ID: ");
      Serial.println(sensor_id);
      continue;
    }
    
    // Read ADC value (ESP32 returns 12-bit value: 0-4095)
    int adcValue = analogRead(pin);
    
    // Convert ADC value to voltage (ESP32 ADC reference is 3.3V)
    // Voltage = (ADC_value / 4095.0) * 3.3V
    float voltage = (static_cast<float>(adcValue) / 4095.0f) * 3.3f;
    
    // Store voltage as uint32_t (interpreted as float in packet)
    // We need to send the float as a uint32_t by bit-casting
    uint32_t voltage_bits;
    memcpy(&voltage_bits, &voltage, sizeof(float));
    chunk.add_datapoint(sensor_id, voltage_bits);
  }
  
  // Send the sensor data packet if we have data
  if (!chunk.empty()) {
    sendSensorDataPacket(chunk);
  }
}

void sendSensorDataPacket(const Diablo::SensorDataChunkCollection &chunk) {
  // Create packet buffer
  uint8_t packetBuffer[MAX_PACKET_SIZE];
  
  // Create a vector with a single chunk for the packet
  std::vector<Diablo::SensorDataChunkCollection> chunks;
  chunks.push_back(chunk);
  
  // Create the sensor data packet using DAQv2-Comms library
  size_t packetSize = Diablo::create_sensor_data_packet(
    chunks,
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
  Serial.print(NUM_SENSORS);
  Serial.println(" sensors");
}
