/**
 * Actuator Hotfire — single firmware for all actuator boards (controller and designated survivor).
 *
 * State machine: WaitingForServer -> Active <-> StandardAbort/NoConnectionAbort -> PTAbort/NoPTAbort/StandaloneAbort -> AbortFinished.
 * Behavior gated by is_abort_controller from ACTUATOR_CONFIG. Uses DAQv2-Comms over Ethernet/UDP.
 * Implements plan: Single Actuator Hotfire Script (Hotfire_Code/Actuator_Hotfire).
 */

#include <Arduino.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <DAQv2-Comms.h>
#include <cstring>
#include <esp_mac.h>
#include <vector>
#include <set>
#include "main.h"
#include "actuator_board_pins.h"
#include "actuator_config.h"

using namespace actuator_board_pins;

//-----------------------------------------------------------------------------
// Board identity and network
//-----------------------------------------------------------------------------
static uint8_t board_id = BOARD_ID_DEFAULT;
byte mac[6];
IPAddress staticIP(192, 168, 2, BOARD_ID_DEFAULT);
IPAddress gateway(0, 0, 0, 0);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 2, 1);
IPAddress serverIP(192, 168, 2, HOTFIRE_SERVER_IP_OCTET_4);
const int udpListenPort = 5005;
const int serverPort = HOTFIRE_SERVER_PORT;
const int ptBoardPort = 5005;
EthernetUDP udp;

//-----------------------------------------------------------------------------
// Serial gating from ACTUATOR_CONFIG (enable_serial_printing); default on until config says otherwise
//-----------------------------------------------------------------------------
static bool g_actuator_serial = true;
#define ACTUATOR_PRINT(x)    do { if (g_actuator_serial) Serial.print(x); } while(0)
#define ACTUATOR_PRINTLN(x)  do { if (g_actuator_serial) Serial.println(x); } while(0)

//-----------------------------------------------------------------------------
// State machine
//-----------------------------------------------------------------------------
enum class ActuatorControllerState {
  WaitingForServer,
  Active,
  NoConnectionAbort,
  PTAbort,
  NoPTAbort,
  AbortFinished,
  ConnectionLossDetected,
  NoConnAbortFollower
};

static ActuatorControllerState state = ActuatorControllerState::WaitingForServer;
static unsigned long last_server_heartbeat_ms = 0;
static unsigned long state_enter_ms = 0;

// Config from server
static bool is_abort_controller = false;
static std::vector<Diablo::AbortActuatorLocation> abort_actuator_locations;
static std::vector<Diablo::AbortPTLocation> abort_pt_locations;
static bool config_valid = false;

// NoConnectionAbort: vent sent once, then per-PT tracking (all received -> PTAbort else NoPTAbort)
static bool no_conn_abort_vent_sent = false;
static bool no_conn_abort_pt_sent = false;
static std::vector<bool> pt_received_during_no_conn;  // index = abort_pt_locations index

// ConnectionLossDetected: IPs we've received a packet from (any packet = "reachable")
static std::set<uint32_t> connection_loss_received_ips;

// PT Abort: per-PT "below threshold" and vent/abort phase done
static std::vector<bool> pt_below_threshold;
static bool pt_abort_vent_done = false;
static bool pt_abort_abort_done = false;

// No PT Abort: vent done, then after 10s abort done
static bool no_pt_abort_vent_done = false;
static bool no_pt_abort_abort_done = false;



// Sensor data
const uint8_t NUM_SENSORS = NUM_ACTUATORS;
static unsigned long last_adc_read_ms = 0;
static uint8_t actuator_states[NUM_ACTUATORS];
static unsigned long lastHeartbeatMillis = 0;

// PWM state (for PWM_ACTUATOR_COMMAND)
struct PWMState {
  bool active;
  unsigned long start_time;
  uint32_t duration;
  uint8_t channel;
};
static PWMState pwm_states[NUM_ACTUATORS];

//-----------------------------------------------------------------------------
// Status LED: state numbers 1-11 (plan: 7=PTAbort, 8=NoPTAbort, 9=StandaloneAbort)
//-----------------------------------------------------------------------------
enum class LedPhase { Idle, On, Off };
static LedPhase led_phase = LedPhase::Idle;
static unsigned long led_cycle_start_ms = 0;
static uint8_t led_blink_index = 0;
static unsigned long led_phase_start_ms = 0;
static ActuatorControllerState last_led_state = ActuatorControllerState::WaitingForServer;

static uint8_t getStateNumber(ActuatorControllerState s) {
  switch (s) {
    case ActuatorControllerState::WaitingForServer:   return 1;
    case ActuatorControllerState::Active:             return 2;
    case ActuatorControllerState::ConnectionLossDetected: return 3;
    case ActuatorControllerState::NoConnectionAbort: return 4;
    case ActuatorControllerState::NoConnAbortFollower: return 5;
    case ActuatorControllerState::PTAbort:           return 6;
    case ActuatorControllerState::NoPTAbort:         return 7;
    case ActuatorControllerState::AbortFinished:     return 8;
    default: return 1;
  }
}

static const char* stateName(ActuatorControllerState s) {
  switch (s) {
    case ActuatorControllerState::WaitingForServer:   return "WaitingForServer";
    case ActuatorControllerState::Active:             return "Active";
    case ActuatorControllerState::NoConnectionAbort: return "NoConnectionAbort";
    case ActuatorControllerState::PTAbort:           return "PTAbort";
    case ActuatorControllerState::NoPTAbort:         return "NoPTAbort";
    case ActuatorControllerState::AbortFinished:     return "AbortFinished";
    case ActuatorControllerState::ConnectionLossDetected: return "ConnectionLossDetected";
    case ActuatorControllerState::NoConnAbortFollower: return "NoConnAbortFollower";
    default: return "?";
  }
}

static void setState(ActuatorControllerState new_state) {
  if (state != new_state) {
    Serial.print("State -> ");
    Serial.println(stateName(new_state));
    Serial.flush();
    state = new_state;
    state_enter_ms = millis();
  }
}

static void updateLedNonBlocking() {
  const unsigned long now = millis();
  const uint8_t nblinks = getStateNumber(state);
  const int pin = Actuator_Board.LED;

  if (state != last_led_state) {
    last_led_state = state;
    led_phase = LedPhase::Idle;
    led_cycle_start_ms = now;
  }

  switch (led_phase) {
    case LedPhase::Idle:
      digitalWrite(pin, LOW);
      if ((now - led_cycle_start_ms) >= LED_CYCLE_MS) {
        led_cycle_start_ms = now;
        led_blink_index = 0;
        digitalWrite(pin, HIGH);
        led_phase = LedPhase::On;
        led_phase_start_ms = now;
      }
      break;
    case LedPhase::On:
      if ((now - led_phase_start_ms) >= LED_BLINK_ON_MS) {
        digitalWrite(pin, LOW);
        led_phase = LedPhase::Off;
        led_phase_start_ms = now;
      }
      break;
    case LedPhase::Off:
      if ((now - led_phase_start_ms) >= LED_BLINK_OFF_MS) {
        led_blink_index++;
        if (led_blink_index < nblinks) {
          digitalWrite(pin, HIGH);
          led_phase = LedPhase::On;
        } else {
          led_phase = LedPhase::Idle;
          led_cycle_start_ms = now;
        }
        led_phase_start_ms = now;
      }
      break;
  }
}

//-----------------------------------------------------------------------------
// Heartbeat
//-----------------------------------------------------------------------------
static Diablo::BoardState getBoardStateForHeartbeat() {
  switch (state) {
    case ActuatorControllerState::WaitingForServer:
      return Diablo::BoardState::SETUP;
    case ActuatorControllerState::Active:
      return Diablo::BoardState::ACTIVE;
    case ActuatorControllerState::ConnectionLossDetected:
      return Diablo::BoardState::CONNECTION_LOSS_DETECTED;
    case ActuatorControllerState::NoConnectionAbort:
      return Diablo::BoardState::NO_CONNECTION_ABORT;
    case ActuatorControllerState::NoConnAbortFollower:
      return Diablo::BoardState::NO_CONN_ABORT_FOLLOWER;
    case ActuatorControllerState::PTAbort:
      return Diablo::BoardState::PT_ABORT;
    case ActuatorControllerState::NoPTAbort:
      return Diablo::BoardState::NO_PT_ABORT;
    case ActuatorControllerState::AbortFinished:
      return Diablo::BoardState::ABORT_FINISHED;
    default:
      return Diablo::BoardState::SETUP;
  }
}

static void sendBoardHeartbeat() {
  Diablo::BoardHeartbeatPacket hb;
  hb.board_type = Diablo::BoardType::ACTUATOR;
  hb.board_id = board_id;
  hb.engine_state = Diablo::EngineState::SAFE;
  hb.board_state = getBoardStateForHeartbeat();

  uint8_t packetBuffer[MAX_PACKET_SIZE];
  size_t len = Diablo::create_board_heartbeat_packet(hb, packetBuffer, sizeof(packetBuffer));
  if (len == 0) return;

  Serial.print("Sent: heartbeat to ");
  Serial.print(serverIP);
  Serial.print(":");
  Serial.println(serverPort);
  Serial.flush();
  udp.beginPacket(serverIP, serverPort);
  udp.write(packetBuffer, len);
  udp.endPacket();
}

//-----------------------------------------------------------------------------
// Helpers: self IP as uint32_t, IP in PT list
//-----------------------------------------------------------------------------
static uint32_t getSelfIP() {
  return static_cast<uint32_t>(staticIP[0]) << 24 |
         static_cast<uint32_t>(staticIP[1]) << 16 |
         static_cast<uint32_t>(staticIP[2]) << 8 |
         static_cast<uint32_t>(staticIP[3]);
}

static IPAddress uint32ToIPAddress(uint32_t ip) {
  return IPAddress(
    (ip >> 24) & 0xFF,
    (ip >> 16) & 0xFF,
    (ip >> 8) & 0xFF,
    ip & 0xFF
  );
}

static bool isIPInPTLocations(uint32_t ip) {
  for (const auto &loc : abort_pt_locations) {
    if (loc.ip_address == ip) return true;
  }
  return false;
}

// NO_CONNECTION_ABORT: header-only packet (ABORT type) sent to PTs so they enter standalone/abort path
static size_t create_no_connection_abort_packet(uint8_t *buffer, size_t buffer_size) {
  if (buffer_size < sizeof(Diablo::PacketHeader)) return 0;
  Diablo::PacketHeader header;
  header.packet_type = Diablo::PacketType::NO_CONNECTION_ABORT;
  header.version = 1;
  header.timestamp = millis();
  memcpy(buffer, &header, sizeof(header));
  return sizeof(header);
}

// ABORT_DONE: header-only packet sent to PTs and generic actuators to signify abort completion
static size_t create_abort_done_packet(uint8_t *buffer, size_t buffer_size) {
  if (buffer_size < sizeof(Diablo::PacketHeader)) return 0;
  Diablo::PacketHeader header;
  header.packet_type = Diablo::PacketType::ABORT_DONE;
  header.version = 1;
  header.timestamp = millis();
  memcpy(buffer, &header, sizeof(header));
  return sizeof(header);
}

//-----------------------------------------------------------------------------
// Apply vent or abort to all actuators (self = local, others = UDP to that IP)
//-----------------------------------------------------------------------------
static void applyVentToAllActuators() {
  const uint32_t self_ip = getSelfIP();
  uint8_t packetBuffer[MAX_PACKET_SIZE];
  for (const auto &loc : abort_actuator_locations) {
    if (loc.ip_address == self_ip) {
      int pin = getActuatorPin(loc.actuator_id);
      if (pin >= 0) {
        digitalWrite(pin, loc.vent_state ? HIGH : LOW);
        actuator_states[loc.actuator_id - 1] = loc.vent_state ? 1 : 0;
      }
    } else {
      std::vector<Diablo::ActuatorCommand> cmds;
      cmds.push_back({ loc.actuator_id, loc.vent_state ? static_cast<uint8_t>(1) : static_cast<uint8_t>(0) });
      size_t len = Diablo::create_actuator_command_packet(cmds, packetBuffer, sizeof(packetBuffer));
      if (len > 0) {
        udp.beginPacket(uint32ToIPAddress(loc.ip_address), udpListenPort);
        udp.write(packetBuffer, len);
        udp.endPacket();
        ACTUATOR_PRINT("Sent: actuator_command (vent) to ");
        ACTUATOR_PRINT(uint32ToIPAddress(loc.ip_address));
        ACTUATOR_PRINT(":");
        ACTUATOR_PRINTLN(udpListenPort);
      }
    }
  }
}

static void applyAbortToAllActuators() {
  const uint32_t self_ip = getSelfIP();
  uint8_t packetBuffer[MAX_PACKET_SIZE];
  for (const auto &loc : abort_actuator_locations) {
    if (loc.ip_address == self_ip) {
      int pin = getActuatorPin(loc.actuator_id);
      if (pin >= 0) {
        digitalWrite(pin, loc.abort_state ? HIGH : LOW);
        actuator_states[loc.actuator_id - 1] = loc.abort_state ? 1 : 0;
      }
    } else {
      std::vector<Diablo::ActuatorCommand> cmds;
      cmds.push_back({ loc.actuator_id, loc.abort_state ? static_cast<uint8_t>(1) : static_cast<uint8_t>(0) });
      size_t len = Diablo::create_actuator_command_packet(cmds, packetBuffer, sizeof(packetBuffer));
      if (len > 0) {
        udp.beginPacket(uint32ToIPAddress(loc.ip_address), udpListenPort);
        udp.write(packetBuffer, len);
        udp.endPacket();
        ACTUATOR_PRINT("Sent: actuator_command (abort) to ");
        ACTUATOR_PRINT(uint32ToIPAddress(loc.ip_address));
        ACTUATOR_PRINT(":");
        ACTUATOR_PRINTLN(udpListenPort);
      }
    }
  }
}

static void applyVentLocalOnly() {
  const uint32_t self_ip = getSelfIP();
  for (const auto &loc : abort_actuator_locations) {
    if (loc.ip_address != self_ip) continue;
    int pin = getActuatorPin(loc.actuator_id);
    if (pin >= 0) {
      digitalWrite(pin, loc.vent_state ? HIGH : LOW);
      actuator_states[loc.actuator_id - 1] = loc.vent_state ? 1 : 0;
    }
  }
}

static void applyAbortLocalOnly() {
  const uint32_t self_ip = getSelfIP();
  for (const auto &loc : abort_actuator_locations) {
    if (loc.ip_address != self_ip) continue;
    int pin = getActuatorPin(loc.actuator_id);
    if (pin >= 0) {
      digitalWrite(pin, loc.abort_state ? HIGH : LOW);
      actuator_states[loc.actuator_id - 1] = loc.abort_state ? 1 : 0;
    }
  }
}

//-----------------------------------------------------------------------------
// Parse incoming UDP
//-----------------------------------------------------------------------------
static bool readPacketHeader(const uint8_t *buffer, size_t buffer_size, Diablo::PacketHeader &header_out) {
  if (buffer_size < sizeof(Diablo::PacketHeader)) return false;
  memcpy(&header_out, buffer, sizeof(Diablo::PacketHeader));
  return true;
}

enum class IncomingPacketKind {
  None,
  ServerHeartbeat,
  Config,
  Abort,
  AbortDone,
  ClearAbort,
  SensorData,
  NoConnAbort
};

static void processActuatorCommands(const std::vector<Diablo::ActuatorCommand> &commands);
static void processPWMActuatorCommand(const std::vector<Diablo::PWMActuatorCommand> &commands);

static bool stateAcceptsActuatorCommand(uint32_t remote_ip32) {
  uint32_t sip = (static_cast<uint32_t>(serverIP[0]) << 24) |
                 (static_cast<uint32_t>(serverIP[1]) << 16) |
                 (static_cast<uint32_t>(serverIP[2]) << 8) |
                 static_cast<uint32_t>(serverIP[3]);

  // All boards unconditionally ignore server commands when in AbortFinished or NoConnAbortFollower
  if ((state == ActuatorControllerState::AbortFinished || 
       state == ActuatorControllerState::NoConnAbortFollower) && 
      remote_ip32 == sip) {
    return false;
  }

  // Accept commands unconditionally if Active
  if (state == ActuatorControllerState::Active) return true;

  // The designated survivor accepts commands from the server during its abort sequences
  if (is_abort_controller) {
    if (state == ActuatorControllerState::NoConnectionAbort ||
        state == ActuatorControllerState::PTAbort ||
        state == ActuatorControllerState::NoPTAbort ||
        state == ActuatorControllerState::AbortFinished) {
      return true;
    }
  } else {
    // Generic boards accept commands from non-server IPs (i.e. the designated survivor) during aborts
    if (state == ActuatorControllerState::NoConnAbortFollower ||
        state == ActuatorControllerState::NoConnectionAbort ||
        state == ActuatorControllerState::PTAbort ||
        state == ActuatorControllerState::NoPTAbort ||
        state == ActuatorControllerState::AbortFinished) {
      return true;
    }
  }

  return false;
}

static IncomingPacketKind processIncomingPacket(const uint8_t *buffer, size_t len, IPAddress remoteIP) {
  Diablo::PacketHeader hdr;
  if (!readPacketHeader(buffer, len, hdr)) return IncomingPacketKind::None;

  uint32_t remote_ip32 = static_cast<uint32_t>(remoteIP[0]) << 24 |
                         static_cast<uint32_t>(remoteIP[1]) << 16 |
                         static_cast<uint32_t>(remoteIP[2]) << 8 |
                         static_cast<uint32_t>(remoteIP[3]);

  switch (hdr.packet_type) {
    case Diablo::PacketType::SERVER_HEARTBEAT: {
      Diablo::PacketHeader dummy;
      Diablo::ServerHeartbeatPacket data;
      if (Diablo::parse_server_heartbeat_packet(buffer, len, dummy, data)) {
        last_server_heartbeat_ms = millis();
        uint32_t sip = (static_cast<uint32_t>(serverIP[0]) << 24) | (static_cast<uint32_t>(serverIP[1]) << 16) |
                       (static_cast<uint32_t>(serverIP[2]) << 8) | static_cast<uint32_t>(serverIP[3]);
        connection_loss_received_ips.insert(sip);
        return IncomingPacketKind::ServerHeartbeat;
      }
      return IncomingPacketKind::None;
    }
    case Diablo::PacketType::ACTUATOR_CONFIG: {
      Diablo::PacketHeader dummy;
      uint8_t is_controller = 0;
      uint8_t enable_serial = 0;
      std::vector<Diablo::AbortActuatorLocation> act_locs;
      std::vector<Diablo::AbortPTLocation> pt_locs;
      if (Diablo::parse_actuator_config_packet(buffer, len, dummy, is_controller, act_locs, pt_locs, enable_serial)) {
        is_abort_controller = (is_controller != 0);
        abort_actuator_locations = act_locs;
        abort_pt_locations = pt_locs;
        config_valid = true;
        g_actuator_serial = (enable_serial != 0);
        return IncomingPacketKind::Config;
      }
      return IncomingPacketKind::None;
    }
    case Diablo::PacketType::ABORT:
      return IncomingPacketKind::Abort;
    case Diablo::PacketType::ABORT_DONE:
      return IncomingPacketKind::AbortDone;
    case Diablo::PacketType::CLEAR_ABORT:
      return IncomingPacketKind::ClearAbort;
    case Diablo::PacketType::SENSOR_DATA: {
      connection_loss_received_ips.insert(remote_ip32);
      if (state == ActuatorControllerState::NoConnectionAbort && config_valid && !abort_pt_locations.empty()) {
        std::vector<Diablo::SensorDataChunkCollection> chunks;
        Diablo::PacketHeader dummy;
        if (Diablo::parse_sensor_data_packet(buffer, len, dummy, chunks)) {
          for (const auto &col : chunks)
            for (const auto &dp : col.datapoints)
              for (size_t i = 0; i < abort_pt_locations.size(); i++)
                if (abort_pt_locations[i].ip_address == remote_ip32 && abort_pt_locations[i].sensor_id == dp.sensor_id)
                  pt_received_during_no_conn[i] = true;
        }
      }
      if (state == ActuatorControllerState::PTAbort && config_valid && !abort_pt_locations.empty()) {
        std::vector<Diablo::SensorDataChunkCollection> chunks;
        Diablo::PacketHeader dummy;
        if (Diablo::parse_sensor_data_packet(buffer, len, dummy, chunks)) {
          for (const auto &col : chunks) {
            for (const auto &dp : col.datapoints) {
              for (size_t i = 0; i < abort_pt_locations.size(); i++) {
                if (abort_pt_locations[i].ip_address == remote_ip32 &&
                    abort_pt_locations[i].sensor_id == dp.sensor_id &&
                    dp.data < abort_pt_locations[i].pressure_threshold_adc)
                  pt_below_threshold[i] = true;
              }
            }
          }
        }
      }
      return IncomingPacketKind::SensorData;
    }
    case Diablo::PacketType::NO_CONNECTION_ABORT:
      return IncomingPacketKind::NoConnAbort;
    case Diablo::PacketType::ACTUATOR_COMMAND: {
      Diablo::PacketHeader cmd_header;
      std::vector<Diablo::ActuatorCommand> commands;
      if (Diablo::parse_actuator_command_packet(buffer, len, cmd_header, commands) && stateAcceptsActuatorCommand(remote_ip32))
        processActuatorCommands(commands);
      return IncomingPacketKind::None;
    }
    case Diablo::PacketType::PWM_ACTUATOR_COMMAND: {
      Diablo::PacketHeader cmd_header;
      std::vector<Diablo::PWMActuatorCommand> commands;
      if (Diablo::parse_pwm_actuator_packet(buffer, len, cmd_header, commands) && stateAcceptsActuatorCommand(remote_ip32))
        processPWMActuatorCommand(commands);
      return IncomingPacketKind::None;
    }
    default:
      return IncomingPacketKind::None;
  }
}

static void processActuatorCommands(const std::vector<Diablo::ActuatorCommand> &commands) {
  for (const auto &cmd : commands) {
    if (cmd.actuator_id < 1 || cmd.actuator_id > NUM_ACTUATORS) continue;
    uint8_t idx = cmd.actuator_id - 1;
    if (pwm_states[idx].active) {
      int pin = getActuatorPin(cmd.actuator_id);
      if (pin >= 0) {
        ledcWrite(pwm_states[idx].channel, 0);
        ledcDetachPin(pin);
      }
      pwm_states[idx].active = false;
    }
    int pin = getActuatorPin(cmd.actuator_id);
    if (pin < 0) continue;
    uint8_t new_state = (cmd.actuator_state != 0) ? 1 : 0;
    if (actuator_states[idx] != new_state) {
      digitalWrite(pin, new_state ? HIGH : LOW);
      actuator_states[idx] = new_state;
    }
  }
}

static void processPWMActuatorCommand(const std::vector<Diablo::PWMActuatorCommand> &commands) {
  for (const auto &cmd : commands) {
    if (cmd.actuator_id < 1 || cmd.actuator_id > NUM_ACTUATORS) continue;
    int pin = getActuatorPin(cmd.actuator_id);
    if (pin < 0) continue;
    uint8_t array_index = cmd.actuator_id - 1;
    uint8_t channel = pwm_states[array_index].channel;
    if (channel > 7) continue;
    ledcSetup(channel, cmd.frequency, 14);
    ledcAttachPin(pin, channel);
    uint32_t max_duty = 16383;
    uint32_t duty = (uint32_t)(cmd.duty_cycle * (float)max_duty);
    if (duty > max_duty) duty = max_duty;
    ledcWrite(channel, duty);
    pwm_states[array_index].active = true;
    pwm_states[array_index].start_time = millis();
    pwm_states[array_index].duration = cmd.duration;
    actuator_states[array_index] = (duty > 0) ? 1 : 0;
  }
}

static void updatePWM() {
  unsigned long now = millis();
  for (int i = 0; i < NUM_ACTUATORS; i++) {
    if (!pwm_states[i].active) continue;
    if (now - pwm_states[i].start_time >= pwm_states[i].duration) {
      int pin = getActuatorPin(i + 1);
      if (pin >= 0) {
        ledcWrite(pwm_states[i].channel, 0);
        ledcDetachPin(pin);
        digitalWrite(pin, LOW);
      }
      pwm_states[i].active = false;
      actuator_states[i] = 0;
    }
  }
}

//-----------------------------------------------------------------------------
// Current-sense: 1-based (match Actuator_Testing), send to server only
//-----------------------------------------------------------------------------
static void readCurrentSensePinsAndSend() {
  Diablo::SensorDataChunkCollection chunk(millis(), NUM_SENSORS);
  for (uint8_t actuator_id = 1; actuator_id <= NUM_SENSORS; actuator_id++) {
    int pin = getCurrentSensePin(actuator_id);
    if (pin < 0) continue;
    int adcValue = analogRead(pin);
    float voltage = (static_cast<float>(adcValue) / 4095.0f) * 3.3f;
    uint32_t voltage_bits;
    memcpy(&voltage_bits, &voltage, sizeof(float));
    chunk.add_datapoint(actuator_id, voltage_bits);
  }
  if (chunk.empty()) return;

  uint8_t packetBuffer[MAX_PACKET_SIZE];
  std::vector<Diablo::SensorDataChunkCollection> chunks;
  chunks.push_back(chunk);
  size_t packetSize = Diablo::create_sensor_data_packet(chunks, NUM_SENSORS, packetBuffer, sizeof(packetBuffer));
  if (packetSize == 0) return;

  udp.beginPacket(serverIP, serverPort);
  udp.write(packetBuffer, packetSize);
  udp.endPacket();
  ACTUATOR_PRINT("Sent: sensor_data to ");
  ACTUATOR_PRINT(serverIP);
  ACTUATOR_PRINT(":");
  ACTUATOR_PRINTLN(serverPort);
}

static bool heartbeatTimedOut() {
  if (last_server_heartbeat_ms == 0) return false;
  return (millis() - last_server_heartbeat_ms) > HEARTBEAT_TIMEOUT_MS;
}

static void streamSensorDataIfDue() {
  unsigned long now = millis();
  if (now - last_adc_read_ms >= ADC_READ_INTERVAL_MS) {
    last_adc_read_ms = now;
    readCurrentSensePinsAndSend();
  }
}

//-----------------------------------------------------------------------------
// Send NO_CONNECTION_ABORT to all PTs
//-----------------------------------------------------------------------------
static void sendNoConnectionAbortToAllPTs() {
  uint8_t packetBuffer[MAX_PACKET_SIZE];
  size_t len = create_no_connection_abort_packet(packetBuffer, sizeof(packetBuffer));
  if (len == 0) return;
  for (const auto &loc : abort_pt_locations) {
    udp.beginPacket(uint32ToIPAddress(loc.ip_address), ptBoardPort);
    udp.write(packetBuffer, len);
    udp.endPacket();
    ACTUATOR_PRINT("Sent: no_connection_abort to ");
    ACTUATOR_PRINT(uint32ToIPAddress(loc.ip_address));
    ACTUATOR_PRINT(":");
    ACTUATOR_PRINTLN(ptBoardPort);
  }
}

//-----------------------------------------------------------------------------
// Broadcast ABORT_DONE to all boards (PTs and Actuators)
//-----------------------------------------------------------------------------
static void sendAbortDoneToAllBoards() {
  uint8_t packetBuffer[MAX_PACKET_SIZE];
  size_t len = create_abort_done_packet(packetBuffer, sizeof(packetBuffer));
  if (len == 0) return;

  ACTUATOR_PRINTLN("Broadcasting ABORT_DONE to network...");

  // Broadcast to all PTs
  for (const auto &loc : abort_pt_locations) {
    udp.beginPacket(uint32ToIPAddress(loc.ip_address), ptBoardPort);
    udp.write(packetBuffer, len);
    udp.endPacket();
  }

  // Broadcast to all actuator boards (excluding self)
  const uint32_t self_ip = getSelfIP();
  for (const auto &loc : abort_actuator_locations) {
    if (loc.ip_address == self_ip) continue;
    udp.beginPacket(uint32ToIPAddress(loc.ip_address), udpListenPort);
    udp.write(packetBuffer, len);
    udp.endPacket();
  }
}

//-----------------------------------------------------------------------------
// State handlers
//-----------------------------------------------------------------------------
static void run_WaitingForServer() {
  (void)0;
}

static void run_Active() {
  streamSensorDataIfDue();
  // Stay in Active: no transition on connection loss (code kept but unreachable).
  if (false && heartbeatTimedOut()) {
    setState(ActuatorControllerState::ConnectionLossDetected);
    connection_loss_received_ips.clear();
  }
}

static void run_NoConnectionAbort() {
  if (is_abort_controller && config_valid) {
    if (!no_conn_abort_pt_sent) {
      // Broadcast NO_CONNECTION_ABORT to PTs 3 times
      sendNoConnectionAbortToAllPTs();
      delay(10);
      sendNoConnectionAbortToAllPTs();
      delay(10);
      sendNoConnectionAbortToAllPTs();
      
      no_conn_abort_pt_sent = true;
    }
    if (!no_conn_abort_vent_sent && !abort_actuator_locations.empty()) {
      applyVentToAllActuators();
      no_conn_abort_vent_sent = true;
    }
  }

  unsigned long elapsed = millis() - state_enter_ms;
  if (elapsed >= NO_CONN_ABORT_PT_WAIT_MS) {
    no_conn_abort_pt_sent = false;
    no_conn_abort_vent_sent = false;
    bool all_pt_received = true;
    if (!abort_pt_locations.empty()) {
      for (size_t i = 0; i < pt_received_during_no_conn.size(); i++)
        if (!pt_received_during_no_conn[i]) { all_pt_received = false; break; }
    }
    if (all_pt_received && !abort_pt_locations.empty()) {
      setState(ActuatorControllerState::PTAbort);
      pt_below_threshold.resize(abort_pt_locations.size(), false);
      pt_abort_vent_done = false;
      pt_abort_abort_done = false;
    } else {
      setState(ActuatorControllerState::NoPTAbort);
      no_pt_abort_vent_done = false;
      no_pt_abort_abort_done = false;
    }
  }
}

static void run_PTAbort() {
  if (!pt_abort_vent_done && config_valid) {
    applyVentToAllActuators();
    pt_abort_vent_done = true;
  }
  if (!pt_abort_abort_done) {
    bool all_below = true;
    if (!pt_below_threshold.empty()) {
      for (bool b : pt_below_threshold) if (!b) { all_below = false; break; }
    }
    unsigned long elapsed = millis() - state_enter_ms;
    if (all_below || elapsed >= PT_ABORT_THRESHOLD_TIMEOUT_MS) {
      applyAbortToAllActuators();
      sendAbortDoneToAllBoards();
      pt_abort_abort_done = true;
      setState(ActuatorControllerState::AbortFinished);
    }
  }
}

static void run_NoPTAbort() {
  if (!no_pt_abort_vent_done && config_valid) {
    applyVentToAllActuators();
    no_pt_abort_vent_done = true;
  }
  if (no_pt_abort_vent_done && !no_pt_abort_abort_done) {
    if ((millis() - state_enter_ms) >= NO_PT_ABORT_VENT_TO_ABORT_MS) {
      applyAbortToAllActuators();
      sendAbortDoneToAllBoards();
      no_pt_abort_abort_done = true;
      setState(ActuatorControllerState::AbortFinished);
    }
  }
}

static void run_AbortFinished() {
  (void)0;
}

static void run_ConnectionLossDetected() {
  if (connection_loss_received_ips.find(static_cast<uint32_t>(serverIP)) != connection_loss_received_ips.end()) {
    setState(ActuatorControllerState::Active);
    return;
  }
  
  if ((millis() - state_enter_ms) >= CONNECTION_LOSS_GRACE_MS) {
    if (is_abort_controller) {
      setState(ActuatorControllerState::NoConnectionAbort);
      no_conn_abort_vent_sent = false;
      no_conn_abort_pt_sent = false;
      pt_received_during_no_conn.clear();
      pt_received_during_no_conn.resize(abort_pt_locations.size(), false);
    } else {
      setState(ActuatorControllerState::NoConnAbortFollower);
    }
  }
}

static void run_NoConnAbortFollower() {
  (void)0; // Passive state; awaits UDP commands from designated survivor
}

//-----------------------------------------------------------------------------
// Packet-driven transitions
//-----------------------------------------------------------------------------
static void applyPacketTransition(IncomingPacketKind kind) {
  // Global Clear Abort recovery
  if (kind == IncomingPacketKind::ClearAbort) {
     if (state == ActuatorControllerState::AbortFinished) {
       setState(ActuatorControllerState::Active);
       return;
     }
  }

  switch (state) {
    case ActuatorControllerState::WaitingForServer:
      if (kind == IncomingPacketKind::Config) {
        setState(ActuatorControllerState::Active);
      }
      break;
    case ActuatorControllerState::Active:
      // Stay in Active: no transitions on Abort, AbortDone, or NoConnAbort (code kept but unreachable).
      if (false) {
        if (kind == IncomingPacketKind::Abort || kind == IncomingPacketKind::AbortDone) {
          setState(ActuatorControllerState::AbortFinished);
        } else if (kind == IncomingPacketKind::NoConnAbort && !is_abort_controller) {
          setState(ActuatorControllerState::NoConnAbortFollower);
        }
      }
      break;
    case ActuatorControllerState::ConnectionLossDetected:
      if (kind == IncomingPacketKind::ServerHeartbeat) {
        setState(ActuatorControllerState::Active);
      }
      break;
    default:
      break;
  }
}

//-----------------------------------------------------------------------------
// Pin setup
//-----------------------------------------------------------------------------
static void initializeActuators() {
  ACTUATOR_PRINTLN("Initializing actuators...");
  for (int i = 1; i <= NUM_ACTUATORS; i++) {
    int pin = getActuatorPin(i);
    if (pin < 0) continue;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    actuator_states[i - 1] = 0;
  }
  ACTUATOR_PRINTLN("All actuators initialized to OFF state");
}

static void initializeCurrentSensePins() {
  for (uint8_t actuator_id = 1; actuator_id <= NUM_SENSORS; actuator_id++) {
    int pin = getCurrentSensePin(actuator_id);
    if (pin >= 0) pinMode(pin, INPUT);
  }
}

static void initializePWMStates() {
  for (int i = 0; i < NUM_ACTUATORS; i++) {
    pwm_states[i].active = false;
    pwm_states[i].start_time = 0;
    pwm_states[i].duration = 0;
    pwm_states[i].channel = (uint8_t)i;
  }
}

//-----------------------------------------------------------------------------
// Setup and loop
//-----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Actuator Hotfire starting...");

#if TEMP_HARDCODE_BOARD_ID
  board_id = (uint8_t)TEMP_HARDCODE_BOARD_ID;
  staticIP = IPAddress(192, 168, 2, (uint8_t)TEMP_HARDCODE_BOARD_ID);
  Serial.print("Board ID and IP (temp hardcoded): ");
  Serial.print(static_cast<unsigned>(board_id));
  Serial.print(" / 192.168.2.");
  Serial.println(static_cast<unsigned>(board_id));
#else
  if (SPIFFS.begin(false)) {
    File f = SPIFFS.open(SPIFFS_BOARD_VALUE_PATH, "r");
    if (f && f.available() >= 1) {
      uint8_t b;
      if (f.read(&b, 1) == 1) {
        board_id = b;
        staticIP = IPAddress(192, 168, 2, b);
        Serial.print("Board ID and IP from SPIFFS: ");
        Serial.print(static_cast<unsigned>(board_id));
        Serial.print(" / 192.168.2.");
        Serial.println(static_cast<unsigned>(b));
      }
    }
    if (f) f.close();
    SPIFFS.end();
  }
#endif

  initializeActuators();
  initializeCurrentSensePins();
  initializePWMStates();

  pinMode(Actuator_Board.LED, OUTPUT);
  digitalWrite(Actuator_Board.LED, LOW);
  last_led_state = state;
  led_cycle_start_ms = millis();

  ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_ETH));
  SPI.begin(Actuator_Board.ETH_SCLK, Actuator_Board.ETH_MISO, Actuator_Board.ETH_MOSI, Actuator_Board.ETH_CS);
  delay(ETHERNET_SPI_DELAY_MS);
  Ethernet.init(Actuator_Board.ETH_CS);
  delay(ETHERNET_INIT_DELAY_MS);
  Ethernet.begin(mac, staticIP, dns, gateway, subnet);
  delay(ETHERNET_BEGIN_DELAY_MS);
  udp.begin(udpListenPort);

  Serial.print("Ethernet initialized. IP: ");
  Serial.println(Ethernet.localIP());

  state = ActuatorControllerState::WaitingForServer;
  Serial.println("State -> WaitingForServer");
  Serial.flush();
  state_enter_ms = millis();
  last_server_heartbeat_ms = 0;

  Serial.println("Setup complete. State: WaitingForServer");
}

void loop() {
  updateLedNonBlocking();
  updatePWM();

  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    IPAddress remoteIP = udp.remoteIP();
    uint8_t packetBuffer[MAX_PACKET_SIZE];
    int bytesRead = udp.read(packetBuffer, sizeof(packetBuffer));
    if (bytesRead > 0) {
      Serial.print("Received packet from ");
      Serial.print(remoteIP);
      Serial.print(":");
      Serial.print(udp.remotePort());
      Serial.print(" type ");
      Serial.println(bytesRead >= (int)sizeof(Diablo::PacketHeader) ? (unsigned)packetBuffer[0] : 0);
      Serial.flush();
      connection_loss_received_ips.insert(
        static_cast<uint32_t>(remoteIP[0]) << 24 |
        static_cast<uint32_t>(remoteIP[1]) << 16 |
        static_cast<uint32_t>(remoteIP[2]) << 8 |
        static_cast<uint32_t>(remoteIP[3]));
      IncomingPacketKind kind = processIncomingPacket(packetBuffer, bytesRead, remoteIP);
      applyPacketTransition(kind);
    }
  }

  switch (state) {
    case ActuatorControllerState::WaitingForServer:
      run_WaitingForServer();
      break;
    case ActuatorControllerState::Active:
      run_Active();
      break;
    case ActuatorControllerState::NoConnectionAbort:
      run_NoConnectionAbort();
      break;
    case ActuatorControllerState::PTAbort:
      run_PTAbort();
      break;
    case ActuatorControllerState::NoPTAbort:
      run_NoPTAbort();
      break;
    case ActuatorControllerState::AbortFinished:
      run_AbortFinished();
      break;
    case ActuatorControllerState::ConnectionLossDetected:
      run_ConnectionLossDetected();
      break;
    case ActuatorControllerState::NoConnAbortFollower:
      run_NoConnAbortFollower();
      break;
  }

  unsigned long now = millis();
  if (now - lastHeartbeatMillis >= BOARD_HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMillis = now;
    if (state == ActuatorControllerState::WaitingForServer) {
      Serial.println("Setup state: sending heartbeat");
      Serial.flush();
    }
    sendBoardHeartbeat();
  }

  delay(LOOP_DELAY_MS);
}
