/**
 * LC (Load Cell) Hotfire state machine — ESP32 PCB
 *
 * Same state machine: Waiting for Server → Active → Standalone Abort (and back to Active on clear).
 * Sends heartbeats, streams load-cell (differential) sensor data to server or to actuator controller in abort.
 * Uses LC_Board pin layout and BoardType::LOAD_CELL. Only connectors on ADC 1 (1, 2, 3, 6, 7).
 * Each load cell: differential read between pin 1 and pin 2 on the connector (connector mapping).
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
// ADC config — LC Board ADC 1 only; differential (pin 1 vs pin 2) per connector
//-----------------------------------------------------------------------------
#define FILTER       ADS126X_SINC4
#define DATA_RATE    ADS126X_RATE_7200
#define GAIN         ADS126X_GAIN_32
#define NUM_CHANNELS  5   // ADC 1 connectors only: 1, 2, 3, 6, 7
#define MAX_CHUNKS    9

// Connectors on LC Board that are on ADC 1 (connector map adc_index == 1)
static const uint8_t LC_ADC1_CONNECTORS[] = { 1, 2, 3, 6, 7 };

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
enum class LCHotfireState {
  WaitingForServer,
  Active,
  StandaloneAbort
};

static LCHotfireState state = LCHotfireState::WaitingForServer;

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
  hb.board_type = Diablo::BoardType::LOAD_CELL;
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
// One chunk = one full scan of ADC-1 load-cell connectors (differential pin 1 vs pin 2).
// Per connector: setInputMux(getAdcChannel(c,1), getAdcChannel(c,2)), then READINGS_PER_CONNECTOR reads.
//-----------------------------------------------------------------------------
static void collect_chunk() {
  Diablo::SensorDataChunkCollection chunk(millis(), NUM_CHANNELS);
  for (size_t i = 0; i < NUM_CHANNELS; i++) {
    const uint8_t connector_id = LC_ADC1_CONNECTORS[i];
    const int ch1 = getAdcChannel(connector_id, 1);
    const int ch2 = getAdcChannel(connector_id, 2);
    if (ch1 < 0 || ch2 < 0) continue;
    ads126x.setInputMux(static_cast<uint8_t>(ch1), static_cast<uint8_t>(ch2));
    flush_cycles(settlePulses(FILTER));
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
// Apply packet-driven transitions
//-----------------------------------------------------------------------------
static void applyPacketTransition(IncomingPacketKind kind) {
  switch (state) {
    case LCHotfireState::WaitingForServer:
      if (kind == IncomingPacketKind::SensorConfig) {
        state = LCHotfireState::Active;
        Serial.println("State -> Active");
      }
      break;
    case LCHotfireState::Active:
      if (kind == IncomingPacketKind::NoConnAbort && stored_config.necessary_for_abort)
        state = LCHotfireState::StandaloneAbort;
      break;
    case LCHotfireState::StandaloneAbort:
      if (kind == IncomingPacketKind::ClearAbort) {
        state = LCHotfireState::Active;
      }
      break;
  }
}

//-----------------------------------------------------------------------------
// Setup
//-----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("LC Hotfire state machine starting...");

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

  ADC_SPI.begin(Pins.ADC_SCLK, Pins.ADC_MISO, Pins.ADC_MOSI, Pins.ADC_CS_1);
  ADC_SPI.setDataMode(SPI_MODE1);
  pinMode(Pins.ADC_DRDY_1, INPUT);

  ads126x.begin(Pins.ADC_CS_1, &ADC_SPI);
  ads126x.stopADC1();
  ads126x.setInputMux(static_cast<uint8_t>(getAdcChannel(1, 1)), static_cast<uint8_t>(getAdcChannel(1, 2)));

  ads126x.enablePGA();
  ads126x.setGain(GAIN);

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

  state = LCHotfireState::WaitingForServer;
  Serial.print("Ethernet IP: ");
  Serial.println(Ethernet.localIP());
  Serial.println("Setup complete. State: WaitingForServer (LC Board, ADC 1 connectors 1,2,3,6,7)");
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
  if (state == LCHotfireState::Active || state == LCHotfireState::StandaloneAbort)
    collect_chunk();

  switch (state) {
    case LCHotfireState::WaitingForServer:
      run_WaitingForServer();
      break;
    case LCHotfireState::Active:
      run_Active();
      break;
    case LCHotfireState::StandaloneAbort:
      run_StandaloneAbort();
      break;
  }

  // Send board heartbeat at fixed interval (e.g. once per second)
  unsigned long now = millis();
  if (now - lastHeartbeatMillis >= BOARD_HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMillis = now;
    switch (state) {
      case LCHotfireState::WaitingForServer:
        Serial.print("Heartbeat SETUP | chunks=");
        Serial.println(dataChunks.size());
        sendBoardHeartbeat(Diablo::BoardState::SETUP, serverIP, serverPort);
        break;
      case LCHotfireState::Active:
        Serial.print("Heartbeat ACTIVE | chunks=");
        Serial.println(dataChunks.size());
        sendBoardHeartbeat(Diablo::BoardState::ACTIVE, serverIP, serverPort);
        break;
      case LCHotfireState::StandaloneAbort:
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
