/**
 * TC (Thermocouple) Hotfire state machine — ESP32 PCB
 *
 * Same state machine as TC Hotfire: Waiting for Server → Active → Standalone Abort (and back to Active on clear).
 * Sends heartbeats, streams thermocouple/sensor data to server or to actuator controller in abort.
 * Uses TC_Board pin layout and BoardType::THERMOCOUPLE.
 */

#include <Arduino.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <DAQv2-Comms.h>
#include <cstring>
#include <vector>
#include <esp_mac.h>

#include "main.h"
#define PINS_ACTIVE_LAYOUT sense_board_pins::LC_Board
#include "STAR_ADS126X.h"
#include "sense_board_pins.h"
#include "connector_adc_map.h"
#include "adc_mappings.h"

using namespace sense_board_pins;

//-----------------------------------------------------------------------------
// ADC config (Stream_ADC_Data / multi-channel style)
//-----------------------------------------------------------------------------
#define FILTER       ADS126X_SINC4
#define DATA_RATE    ADS126X_RATE_7200
#define TEST_PIN     1
#define NUM_CHANNELS  10
#define MAX_CHUNKS   9   // 9 chunks fit in 512-byte packet with 10 sensors

static ADS126X ads126x;
SPIClass ADC_SPI(HSPI);

//-----------------------------------------------------------------------------
// Ethernet and network (IP 192.168.2.XXX and board_id = XXX from SPIFFS)
//-----------------------------------------------------------------------------
static uint8_t board_id = BOARD_ID_DEFAULT;
byte mac[6];
IPAddress staticIP(192, 168, 2, BOARD_ID_DEFAULT);
IPAddress gateway(0, 0, 0, 0);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 2, 1);
const int udpListenPort = 5005;
const int serverPortDefault = 5006;
EthernetUDP udp;

// Server address: updated when we receive a server heartbeat (store server address in server heartbeat)
IPAddress serverIP(192, 168, 2, 20);
int serverPort = serverPortDefault;

//-----------------------------------------------------------------------------
// State machine
//-----------------------------------------------------------------------------
enum class TCHotfireState {
  WaitingForServer,
  Active,
  StandaloneAbort
};

static TCHotfireState state = TCHotfireState::WaitingForServer;

// Stored config from SENSOR_CONFIG packet (necessary_for_abort, actuator controller IP)
struct StoredSensorConfig {
  bool valid;
  bool necessary_for_abort;
  uint32_t actuator_controller_ip;  // IPv4 as uint32_t (e.g. network order)
};
static StoredSensorConfig stored_config = { false, false, 0 };

// Sensor data collection: each chunk = one full scan (NUM_CHANNELS datapoints)
std::vector<Diablo::SensorDataChunkCollection> dataChunks;

// Heartbeat at fixed interval (main.h: BOARD_HEARTBEAT_INTERVAL_MS)
static unsigned long lastHeartbeatMillis = 0;

// State LED: blink N times per cycle, N = state number (1=WaitingForServer, 2=Active, 3=StandaloneAbort)
#define STATE_LED_CYCLE_MS  2500
#define BLINK_ON_MS          120
#define BLINK_OFF_MS         120
static unsigned long last_led_cycle_millis = 0;
static unsigned long last_led_edge_millis = 0;
static int led_blink_index = 0;
static bool led_on = false;

//-----------------------------------------------------------------------------
// Incoming packet kinds (for transitions)
//-----------------------------------------------------------------------------
enum class IncomingPacketKind {
  None,
  ServerHeartbeat,
  SensorConfig,
  ClearAbort,
  NoConnAbort
};

//-----------------------------------------------------------------------------
// Helpers: ADC and voltage
//-----------------------------------------------------------------------------
static float convert_code_to_voltage(int32_t code) {
  return (static_cast<float>(code) * 2.5f) / 2147483648.0f;
}

static void flush_cycles(int cycles) {
  for (int i = 0; i < cycles; i++) {
    while (digitalRead(Pins.ADC_DRDY_1) != LOW)
      delayMicroseconds(10);
    ads126x.readADC1();
  }
}

//-----------------------------------------------------------------------------
// Read packet header (for type dispatch)
//-----------------------------------------------------------------------------
static bool readPacketHeader(const uint8_t *buffer, size_t len, Diablo::PacketHeader &hdr_out) {
  if (len < sizeof(Diablo::PacketHeader)) return false;
  memcpy(&hdr_out, buffer, sizeof(Diablo::PacketHeader));
  return true;
}

static const char* packetTypeName(uint8_t type) {
  switch (static_cast<Diablo::PacketType>(type)) {
    case Diablo::PacketType::BOARD_HEARTBEAT:     return "BOARD_HEARTBEAT";
    case Diablo::PacketType::SERVER_HEARTBEAT:    return "SERVER_HEARTBEAT";
    case Diablo::PacketType::SENSOR_DATA:         return "SENSOR_DATA";
    case Diablo::PacketType::ACTUATOR_COMMAND:    return "ACTUATOR_COMMAND";
    case Diablo::PacketType::SENSOR_CONFIG:       return "SENSOR_CONFIG";
    case Diablo::PacketType::ACTUATOR_CONFIG:     return "ACTUATOR_CONFIG";
    case Diablo::PacketType::ABORT:               return "ABORT";
    case Diablo::PacketType::ABORT_DONE:          return "ABORT_DONE";
    case Diablo::PacketType::CLEAR_ABORT:         return "CLEAR_ABORT";
    case Diablo::PacketType::PWM_ACTUATOR_COMMAND: return "PWM_ACTUATOR_COMMAND";
    case Diablo::PacketType::NO_CONNECTION_ABORT: return "NO_CONNECTION_ABORT";
    default: return "UNKNOWN";
  }
}

//-----------------------------------------------------------------------------
// Process one incoming packet; return kind and optionally store server address
// remote_ip/remote_port: set when we receive SERVER_HEARTBEAT (store server address)
//-----------------------------------------------------------------------------
static IncomingPacketKind processIncomingPacket(const uint8_t *buffer, size_t len,
                                                IPAddress remote_ip, int remote_port) {
  Diablo::PacketHeader hdr;
  if (!readPacketHeader(buffer, len, hdr)) return IncomingPacketKind::None;

  switch (hdr.packet_type) {
    case Diablo::PacketType::SERVER_HEARTBEAT: {
      Diablo::PacketHeader dummy;
      Diablo::ServerHeartbeatPacket data;
      if (Diablo::parse_server_heartbeat_packet(buffer, len, dummy, data)) {
        serverIP = remote_ip;
        serverPort = remote_port;
        return IncomingPacketKind::ServerHeartbeat;
      }
      return IncomingPacketKind::None;
    }
    case Diablo::PacketType::SENSOR_CONFIG:
      serverIP = remote_ip;
      serverPort = remote_port;
      // Store config: layout per generate_packets (necessary_for_abort byte, then optional controller_ip)
      stored_config.valid = true;
      if (len >= 6 + 1) {
        stored_config.necessary_for_abort = (buffer[6] != 0);
        if (stored_config.necessary_for_abort && len >= 6 + 5) {
          stored_config.actuator_controller_ip = (static_cast<uint32_t>(buffer[7]) << 24) |
            (static_cast<uint32_t>(buffer[8]) << 16) |
            (static_cast<uint32_t>(buffer[9]) << 8) |
            static_cast<uint32_t>(buffer[10]);
        } else {
          stored_config.actuator_controller_ip = 0;
        }
      }
      return IncomingPacketKind::SensorConfig;
    case Diablo::PacketType::CLEAR_ABORT:
      return IncomingPacketKind::ClearAbort;
    case Diablo::PacketType::NO_CONNECTION_ABORT: {
      Diablo::PacketHeader dummy;
      if (Diablo::parse_no_connection_abort_packet(buffer, len, dummy))
        return IncomingPacketKind::NoConnAbort;
      return IncomingPacketKind::None;
    }
    default:
      return IncomingPacketKind::None;
  }
}

//-----------------------------------------------------------------------------
// Send board heartbeat (board ID + boardState), like actuator_c / Stream_ADC_Data
//-----------------------------------------------------------------------------
static void sendBoardHeartbeat(Diablo::BoardState board_state, IPAddress dest_ip, int dest_port) {
  Diablo::BoardHeartbeatPacket hb;
  hb.board_type = Diablo::BoardType::THERMOCOUPLE;
  hb.board_id = board_id;
  hb.engine_state = Diablo::EngineState::SAFE;
  hb.board_state = board_state;

  uint8_t packetBuffer[MAX_PACKET_SIZE];
  size_t n = Diablo::create_board_heartbeat_packet(hb, packetBuffer, sizeof(packetBuffer));
  if (n == 0) return;
  udp.beginPacket(dest_ip, dest_port);
  udp.write(packetBuffer, n);
  udp.endPacket();
}

//-----------------------------------------------------------------------------
// Stream sensor data to a destination (Stream_ADC_Data style).
// Only up to MAX_CHUNKS fit in one UDP packet; send one packet per call and drain that many.
//-----------------------------------------------------------------------------
static void sendSensorDataPacketTo(IPAddress dest_ip, int dest_port) {
  if (dataChunks.empty()) return;
  size_t n = (dataChunks.size() > MAX_CHUNKS) ? MAX_CHUNKS : dataChunks.size();
  std::vector<Diablo::SensorDataChunkCollection> batch(dataChunks.begin(), dataChunks.begin() + n);
  uint8_t packetBuffer[MAX_PACKET_SIZE];
  size_t packetSize = Diablo::create_sensor_data_packet(
    batch, static_cast<uint8_t>(NUM_CHANNELS), packetBuffer, sizeof(packetBuffer));
  if (packetSize == 0) {
    Serial.print("Send FAIL: create_sensor_data_packet returned 0 (n=");
    Serial.print(n);
    Serial.print(", buf=");
    Serial.print(MAX_PACKET_SIZE);
    Serial.println(")");
    return;
  }
  udp.beginPacket(dest_ip, dest_port);
  udp.write(packetBuffer, packetSize);
  udp.endPacket();
  Serial.print("Sent sensor data: ");
  Serial.print(packetSize);
  Serial.print(" bytes, ");
  Serial.print(n);
  Serial.print(" chunks -> ");
  Serial.print(dest_ip);
  Serial.print(":");
  Serial.println(dest_port);
  dataChunks.erase(dataChunks.begin(), dataChunks.begin() + n);
}

//-----------------------------------------------------------------------------
// One chunk = one full scan of all NUM_CHANNELS connectors (exactly 10 datapoints).
// Per connector: READINGS_PER_CONNECTOR reads (main.h), use last valid for that connector.
// Packet format and GUI expect num_sensors datapoints per chunk.
//-----------------------------------------------------------------------------
static void collect_chunk() {
  Diablo::SensorDataChunkCollection chunk(millis(), NUM_CHANNELS);
  for (uint8_t connector_id = 1; connector_id <= NUM_CHANNELS; connector_id++) {
    ads126x.setInputMux(getAdcChannel(connector_id, TEST_PIN), ADS126X_AINCOM);
    flush_cycles(settlePulses(FILTER, DATA_RATE));
    uint32_t value = 0u;
    for (int r = 0; r < READINGS_PER_CONNECTOR; r++) {
      while (digitalRead(Pins.ADC_DRDY_1) != LOW)
        delayMicroseconds(10);
      const auto reading = ads126x.readADC1();
      if (reading.checksumValid)
        value = static_cast<uint32_t>(reading.value);
    }
    chunk.add_datapoint(connector_id, value);
  }
  if (chunk.size() == NUM_CHANNELS) {
    dataChunks.push_back(chunk);
    Serial.print("Chunk pushed, total chunks=");
    Serial.println(dataChunks.size());
  } else {
    static unsigned long lastIncompleteLog = 0;
    if (millis() - lastIncompleteLog >= 2000) {
      lastIncompleteLog = millis();
      Serial.print("Chunk incomplete: size=");
      Serial.println(chunk.size());
    }
  }
}

//-----------------------------------------------------------------------------
// State handlers
//-----------------------------------------------------------------------------

// 1: Waiting for Server — heartbeat sent at fixed interval in loop; on config -> Active
static void run_WaitingForServer() {
  // No per-state work; heartbeat sent in loop
}

// 2: Active — each loop: if we have any chunks, send one packet (up to MAX_CHUNKS) and drain them
static void run_Active() {
  if (dataChunks.size() >= 1) {
    Serial.print("Active: sending ");
    Serial.print(dataChunks.size());
    Serial.println(" chunk(s)");
    sendSensorDataPacketTo(serverIP, serverPort);
  }
}

// 3: Standalone Abort — stream sensor data to actuator controller
static void run_StandaloneAbort() {
  if (!stored_config.valid || stored_config.actuator_controller_ip == 0)
    return;
  IPAddress actuatorIP(
    (stored_config.actuator_controller_ip >> 24) & 0xFF,
    (stored_config.actuator_controller_ip >> 16) & 0xFF,
    (stored_config.actuator_controller_ip >> 8) & 0xFF,
    stored_config.actuator_controller_ip & 0xFF);
  if (!dataChunks.empty())
    sendSensorDataPacketTo(actuatorIP, serverPortDefault);
}

//-----------------------------------------------------------------------------
// State LED: periodically blink N times where N = current state number (non-blocking)
//-----------------------------------------------------------------------------
static void update_state_led(int state_num) {
  if (state_num < 1) state_num = 1;
  if (state_num > 3) state_num = 3;
  unsigned long now = millis();

  if (now - last_led_cycle_millis >= STATE_LED_CYCLE_MS) {
    last_led_cycle_millis = now;
    last_led_edge_millis = now;
    led_blink_index = 0;
    led_on = true;
    digitalWrite(Pins.LED, HIGH);
    return;
  }

  if (led_blink_index >= state_num)
    return;

  if (led_on) {
    if (now - last_led_edge_millis >= (unsigned long)BLINK_ON_MS) {
      digitalWrite(Pins.LED, LOW);
      led_on = false;
      led_blink_index++;
      last_led_edge_millis = now;
    }
  } else {
    if (now - last_led_edge_millis >= (unsigned long)BLINK_OFF_MS) {
      digitalWrite(Pins.LED, HIGH);
      led_on = true;
      last_led_edge_millis = now;
    }
  }
}

//-----------------------------------------------------------------------------
// Apply packet-driven transitions
//-----------------------------------------------------------------------------
static void applyPacketTransition(IncomingPacketKind kind) {
  switch (state) {
    case TCHotfireState::WaitingForServer:
      if (kind == IncomingPacketKind::SensorConfig) {
        state = TCHotfireState::Active;
        Serial.println("State -> Active");
      }
      break;
    case TCHotfireState::Active:
      if (kind == IncomingPacketKind::NoConnAbort && stored_config.necessary_for_abort)
        state = TCHotfireState::StandaloneAbort;
      break;
    case TCHotfireState::StandaloneAbort:
      if (kind == IncomingPacketKind::ClearAbort) {
        state = TCHotfireState::Active;
      }
      break;
  }
}

//-----------------------------------------------------------------------------
// Setup
//-----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("TC Hotfire state machine starting...");

  bool spiffs_ok = false;
  // Mount read-only: do not format on fail, so we never overwrite burned value
  if (SPIFFS.begin(false)) {
    File f = SPIFFS.open(SPIFFS_BOARD_VALUE_PATH, "r");  // path from main.h
    if (f && f.available() >= 1) {
      uint8_t b;
      if (f.read(&b, 1) == 1) {
        board_id = b;
        staticIP = IPAddress(192, 168, 2, b);
        spiffs_ok = true;
        Serial.print("Board ID and IP from SPIFFS: ");
        Serial.print(static_cast<unsigned>(board_id));
        Serial.print(" / 192.168.2.");
        Serial.println(static_cast<unsigned>(b));
      }
    }
    if (f) f.close();
    SPIFFS.end();
  }
  if (!spiffs_ok)
    Serial.println("SPIFFS read skipped or failed, using default board ID 1 / 192.168.2.1");

  pinMode(Pins.LED, OUTPUT);
  digitalWrite(Pins.LED, LOW);

  ADC_SPI.begin(Pins.ADC_SCLK, Pins.ADC_MISO, Pins.ADC_MOSI, Pins.ADC_CS_1);
  ADC_SPI.setDataMode(SPI_MODE1);
  pinMode(Pins.ADC_DRDY_1, INPUT);

  ads126x.begin(Pins.ADC_CS_1, &ADC_SPI);
  ads126x.stopADC1();
  ads126x.setInputMux(static_cast<uint8_t>(getAdcChannel(1, TEST_PIN)), ADS126X_AINCOM);
  ads126x.bypassPGA();
  ads126x.setFilter(FILTER);
  ads126x.setRate(DATA_RATE);
  ads126x.setReference(ADS126X_REF_NEG_VSS, ADS126X_REF_POS_VDD);
  ads126x.startADC1();

  ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_ETH));
  SPI.begin(Pins.ETH_SCLK, Pins.ETH_MISO, Pins.ETH_MOSI);
  delay(ETHERNET_SPI_DELAY_MS);
  Ethernet.init(Pins.ETH_CS);
  delay(ETHERNET_INIT_DELAY_MS);
  Ethernet.begin(mac, staticIP, dns, gateway, subnet);
  delay(ETHERNET_BEGIN_DELAY_MS);
  udp.begin(udpListenPort);

  state = TCHotfireState::WaitingForServer;
  Serial.print("Ethernet IP: ");
  Serial.println(Ethernet.localIP());
  Serial.println("Setup complete. State: WaitingForServer (TC Board)");
}

//-----------------------------------------------------------------------------
// Loop
//-----------------------------------------------------------------------------
void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    IPAddress remoteIP = udp.remoteIP();
    int remotePort = udp.remotePort();
    uint8_t packetBuffer[MAX_PACKET_SIZE];
    int bytesRead = udp.read(packetBuffer, sizeof(packetBuffer));
    if (bytesRead > 0) {
      uint8_t ptype = packetBuffer[0];
      Serial.print("Received packet from ");
      Serial.print(remoteIP);
      Serial.print(":");
      Serial.print(remotePort);
      Serial.print("  type ");
      Serial.print(static_cast<unsigned>(ptype));
      Serial.print(" (");
      Serial.print(packetTypeName(ptype));
      Serial.println(")");
      IncomingPacketKind kind = processIncomingPacket(
        packetBuffer, bytesRead, remoteIP, remotePort);
      applyPacketTransition(kind);
    }
  }

  // Only collect sensor data when we're in a state that streams it (Active or StandaloneAbort)
  if (state == TCHotfireState::Active || state == TCHotfireState::StandaloneAbort)
    collect_chunk();

  switch (state) {
    case TCHotfireState::WaitingForServer:
      run_WaitingForServer();
      break;
    case TCHotfireState::Active:
      run_Active();
      break;
    case TCHotfireState::StandaloneAbort:
      run_StandaloneAbort();
      break;
  }

  int state_num = (state == TCHotfireState::WaitingForServer) ? 1 : (state == TCHotfireState::Active) ? 2 : 3;
  update_state_led(state_num);

  // Send board heartbeat at fixed interval (e.g. once per second)
  unsigned long now = millis();
  if (now - lastHeartbeatMillis >= BOARD_HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMillis = now;
    switch (state) {
      case TCHotfireState::WaitingForServer:
        Serial.print("Heartbeat SETUP | chunks=");
        Serial.println(dataChunks.size());
        sendBoardHeartbeat(Diablo::BoardState::SETUP, serverIP, serverPort);
        break;
      case TCHotfireState::Active:
        Serial.print("Heartbeat ACTIVE | chunks=");
        Serial.println(dataChunks.size());
        sendBoardHeartbeat(Diablo::BoardState::ACTIVE, serverIP, serverPort);
        break;
      case TCHotfireState::StandaloneAbort:
        if (!stored_config.valid || stored_config.actuator_controller_ip == 0)
          sendBoardHeartbeat(Diablo::BoardState::ABORT, serverIP, serverPort);
        else {
          IPAddress actuatorIP(
            (stored_config.actuator_controller_ip >> 24) & 0xFF,
            (stored_config.actuator_controller_ip >> 16) & 0xFF,
            (stored_config.actuator_controller_ip >> 8) & 0xFF,
            stored_config.actuator_controller_ip & 0xFF);
          sendBoardHeartbeat(Diablo::BoardState::ABORT, actuatorIP, serverPortDefault);
        }
        break;
    }
  }

  delay(LOOP_DELAY_MS);
}
