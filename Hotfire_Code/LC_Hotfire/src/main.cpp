/**
 * LC (Load Cell) Hotfire state machine — ESP32 PCB
 *
 * Same state machine: Waiting for Server → Active → Standalone Abort (and back to Active on clear).
 * Sends heartbeats, streams load-cell (differential) sensor data to server or to actuator controller in abort.
 * Uses LC_Board pin layout and BoardType::LOAD_CELL. Only connectors on ADC 1 (1, 2, 3, 6, 7).
 * Differential read between pin 1 and pin 2 per connector.
 * Uses shared SensorHotfireCore; board-specific: ADC init, collect_chunk, send_chunks_to.
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
#include "SensorHotfireCore.h"

using namespace sense_board_pins;

//-----------------------------------------------------------------------------
// ADC config — LC Board ADC 1 only; differential (pin 1 vs pin 2) per connector
//-----------------------------------------------------------------------------
#define FILTER       ADS126X_SINC4
#define DATA_RATE    ADS126X_RATE_7200
#define GAIN         ADS126X_GAIN_32
#define MAX_CHUNKS   9

static ADS126X ads126x;
SPIClass ADC_SPI(HSPI);
std::vector<Diablo::SensorDataChunkCollection> dataChunks;

static SensorHotfire::CoreState coreState;
static SensorHotfire::Config coreConfig;

// Set to true to enable Serial output from core and board; false to disable.
bool g_sensor_hotfire_serial = true;

static void flush_cycles(int cycles) {
  for (int i = 0; i < cycles; i++) {
    while (digitalRead(Pins.ADC_DRDY_1) != LOW)
      delayMicroseconds(10);
    ads126x.readADC1();
  }
}

static void collect_chunk_impl() {
  const SensorHotfire::StoredSensorConfig& cfg = coreState.stored_config;
  const uint8_t n = cfg.valid ? cfg.num_sensors : 0;
  if (n == 0) return;
  Diablo::SensorDataChunkCollection chunk(millis(), n);
  for (uint8_t i = 0; i < n; i++) {
    const uint8_t connector_id = cfg.sensor_ids[i];
    const int ch1 = getAdcChannel(connector_id, 1);
    const int ch2 = getAdcChannel(connector_id, 2);
    if (ch1 < 0 || ch2 < 0) continue;
    ads126x.setInputMux(static_cast<uint8_t>(ch1), static_cast<uint8_t>(ch2));
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
  if (chunk.size() == n) {
    dataChunks.push_back(chunk);
    if (coreConfig.debug_packets) {
      SENSOR_HOTFIRE_PRINT("Chunk pushed, total chunks=");
      SENSOR_HOTFIRE_PRINTLN(dataChunks.size());
    }
  } else if (coreConfig.debug_packets) {
    static unsigned long lastIncompleteLog = 0;
    if (millis() - lastIncompleteLog >= 2000) {
      lastIncompleteLog = millis();
      SENSOR_HOTFIRE_PRINT("Chunk incomplete: size=");
      SENSOR_HOTFIRE_PRINTLN(chunk.size());
    }
  }
}

static void send_chunks_to_impl(IPAddress dest_ip, int dest_port,
                                bool also_to_abort_controller,
                                IPAddress abort_controller_ip, int abort_controller_port) {
  if (dataChunks.empty()) return;
  const uint8_t num_sensors = coreState.stored_config.num_sensors;
  if (num_sensors == 0) return;
  size_t n = (dataChunks.size() > MAX_CHUNKS) ? MAX_CHUNKS : dataChunks.size();
  std::vector<Diablo::SensorDataChunkCollection> batch(dataChunks.begin(), dataChunks.begin() + n);
  uint8_t packetBuffer[SENSOR_HOTFIRE_MAX_PACKET_SIZE];
  size_t packetSize = Diablo::create_sensor_data_packet(
      batch, num_sensors, packetBuffer, sizeof(packetBuffer));
  if (packetSize == 0) {
    SENSOR_HOTFIRE_PRINT("Send FAIL: create_sensor_data_packet returned 0 (n=");
    SENSOR_HOTFIRE_PRINT(n);
    SENSOR_HOTFIRE_PRINT(", buf=");
    SENSOR_HOTFIRE_PRINT(SENSOR_HOTFIRE_MAX_PACKET_SIZE);
    SENSOR_HOTFIRE_PRINTLN(")");
    return;
  }
  coreState.udp.beginPacket(dest_ip, dest_port);
  coreState.udp.write(packetBuffer, packetSize);
  coreState.udp.endPacket();
  SENSOR_HOTFIRE_PRINT("Sent: sensor_data to ");
  SENSOR_HOTFIRE_PRINT(dest_ip);
  SENSOR_HOTFIRE_PRINT(":");
  SENSOR_HOTFIRE_PRINTLN(dest_port);
  if (also_to_abort_controller) {
    coreState.udp.beginPacket(abort_controller_ip, abort_controller_port);
    coreState.udp.write(packetBuffer, packetSize);
    coreState.udp.endPacket();
    SENSOR_HOTFIRE_PRINT("Sent: sensor_data to ");
    SENSOR_HOTFIRE_PRINT(abort_controller_ip);
    SENSOR_HOTFIRE_PRINT(":");
    SENSOR_HOTFIRE_PRINTLN(abort_controller_port);
  }
  if (coreConfig.debug_packets) {
    SENSOR_HOTFIRE_PRINT("Sent sensor data: ");
    SENSOR_HOTFIRE_PRINT(packetSize);
    SENSOR_HOTFIRE_PRINT(" bytes, ");
    SENSOR_HOTFIRE_PRINT(n);
    SENSOR_HOTFIRE_PRINT(" chunks -> ");
    SENSOR_HOTFIRE_PRINT(dest_ip);
    SENSOR_HOTFIRE_PRINT(":");
    SENSOR_HOTFIRE_PRINT(dest_port);
    if (also_to_abort_controller) {
      SENSOR_HOTFIRE_PRINT(" + abort ");
      SENSOR_HOTFIRE_PRINT(abort_controller_ip);
      SENSOR_HOTFIRE_PRINT(":");
      SENSOR_HOTFIRE_PRINT(abort_controller_port);
    }
    SENSOR_HOTFIRE_PRINTLN_();
  }
  dataChunks.erase(dataChunks.begin(), dataChunks.begin() + n);
}

static void init_adc_cb(void*) {
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
  ads126x.setReference(ADS126X_REF_NEG_VSS, ADS126X_REF_POS_INT);
  ads126x.startADC1();
}

static void on_reference_voltage_cb(void*, uint8_t reference_voltage) {
  uint8_t ref_pos = (reference_voltage == 1) ? ADS126X_REF_POS_VDD : ADS126X_REF_POS_INT;
  ads126x.setReference(ADS126X_REF_NEG_VSS, ref_pos);
}

static void collect_chunk_cb(void*) { collect_chunk_impl(); }

static void send_chunks_to_cb(void*, IPAddress dest_ip, int dest_port,
                             bool also_to_abort_controller,
                             IPAddress abort_controller_ip, int abort_controller_port) {
  if (!dataChunks.empty())
    send_chunks_to_impl(dest_ip, dest_port, also_to_abort_controller,
                        abort_controller_ip, abort_controller_port);
}

void setup() {
  memset(&coreState, 0, sizeof(coreState));
  coreState.gateway = IPAddress(0, 0, 0, 0);
  coreState.subnet = IPAddress(255, 255, 255, 0);
  coreState.dns = IPAddress(192, 168, 2, 1);

  coreConfig.board_type = Diablo::BoardType::LOAD_CELL;
  coreConfig.board_name = "LC";
  coreConfig.update_server_on_sensor_config = true;
  coreConfig.debug_packets = true;
  coreConfig.pins = &Pins;
  coreConfig.init_adc = init_adc_cb;
  coreConfig.collect_chunk = collect_chunk_cb;
  coreConfig.send_chunks_to = send_chunks_to_cb;
  coreConfig.on_reference_voltage_config = on_reference_voltage_cb;
  coreConfig.user_data = &coreState;
  coreConfig.default_sensor_ids = LC_DEFAULT_SENSOR_IDS;
  coreConfig.default_num_sensors = LC_DEFAULT_NUM_SENSORS;

  SensorHotfire::setup(coreState, coreConfig);
}

void loop() {
  SensorHotfire::loop(coreState, coreConfig);
}
