/**
 * PT (Pressure Transducer) Hotfire state machine — ESP32 PCB
 *
 * Three states: Waiting for Server → Active → Standalone Abort (and back to Active on clear).
 * Sends heartbeats, streams sensor data to server or to actuator controller in abort.
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
#include "STAR_ADS126X.h"
#include "sense_board_pins.h"
#include "connector_adc_map.h"
#include "adc_mappings.h"
#include "SensorHotfireCore.h"

#define PINS_ACTIVE_LAYOUT sense_board_pins::PT_Board
using namespace sense_board_pins;

//-----------------------------------------------------------------------------
// ADC config (Stream_ADC_Data / PT_BOARD_Multi style)
//-----------------------------------------------------------------------------
#define FILTER       ADS126X_SINC4
#define DATA_RATE    ADS126X_RATE_7200
#define TEST_PIN     1

static ADS126X ads126x;
SPIClass ADC_SPI(HSPI);
std::vector<Diablo::SensorDataChunkCollection> dataChunks;

static SensorHotfire::CoreState coreState;
static SensorHotfire::Config coreConfig;

static unsigned long g_last_sensor_packet_log_ms = 0;

// Set to true to enable Serial output from core and board; false to disable.
bool g_sensor_hotfire_serial = true;

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

static void read_single_connector(uint8_t connector_id, int num_readings,
                                  Diablo::SensorDataChunkCollection& chunk) {
  uint32_t value = 0;
  for (int i = 0; i < num_readings; i++) {
    while (digitalRead(Pins.ADC_DRDY_1) != LOW)
      delayMicroseconds(10);
    const auto reading = ads126x.readADC1();
    if (!reading.checksumValid) continue;
    value = static_cast<uint32_t>(reading.value);
  }
  chunk.add_datapoint(connector_id, value);
}

static void collect_chunk_impl() {
  Diablo::SensorDataChunkCollection chunk(millis(), NUM_PTS);
  for (uint8_t connector_id = 1; connector_id <= NUM_PTS; connector_id++) {
    ads126x.setInputMux(getAdcChannel(connector_id, TEST_PIN), ADS126X_AINCOM);
    flush_cycles(settlePulses(FILTER, DATA_RATE));
    read_single_connector(connector_id, READINGS_PER_MUX, chunk);
  }
  if (chunk.size() == NUM_PTS && dataChunks.size() < SENSOR_MAX_CHUNKS_BEFORE_SEND)
    dataChunks.push_back(chunk);
}

static void send_chunks_to_impl(IPAddress dest_ip, int dest_port,
                                bool also_to_abort_controller,
                                IPAddress abort_controller_ip, int abort_controller_port) {
  if (dataChunks.empty()) return;
  if (dataChunks.size() < SENSOR_MAX_CHUNKS_BEFORE_SEND) return;
  uint8_t packetBuffer[SENSOR_HOTFIRE_MAX_PACKET_SIZE];
  size_t packetSize = Diablo::create_sensor_data_packet(
      dataChunks, static_cast<uint8_t>(NUM_PTS), packetBuffer, sizeof(packetBuffer));
  if (packetSize == 0) return;
  coreState.udp.beginPacket(dest_ip, dest_port);
  coreState.udp.write(packetBuffer, packetSize);
  coreState.udp.endPacket();
  SENSOR_HOTFIRE_PRINT("Sent: sensor_data to ");
  SENSOR_HOTFIRE_PRINT(dest_ip);
  SENSOR_HOTFIRE_PRINT(":");
  SENSOR_HOTFIRE_PRINTLN(dest_port);

  unsigned long now = millis();
  if (now - g_last_sensor_packet_log_ms >= 1000) {
    g_last_sensor_packet_log_ms = now;
    SENSOR_HOTFIRE_PRINTLN("SENSOR_DATA contents:");
    for (size_t i = 0; i < dataChunks.size(); ++i) {
      const auto& chunk = dataChunks[i];
      SENSOR_HOTFIRE_PRINT("  chunk ");
      SENSOR_HOTFIRE_PRINT(i);
      SENSOR_HOTFIRE_PRINT(" ts=");
      SENSOR_HOTFIRE_PRINT(chunk.timestamp);
      SENSOR_HOTFIRE_PRINT(" :");
      for (const auto& dp : chunk.datapoints) {
        SENSOR_HOTFIRE_PRINT(" (id=");
        SENSOR_HOTFIRE_PRINT(static_cast<unsigned>(dp.sensor_id));
        SENSOR_HOTFIRE_PRINT(", data=");
        SENSOR_HOTFIRE_PRINT(dp.data);
        SENSOR_HOTFIRE_PRINT(")");
      }
      SENSOR_HOTFIRE_PRINTLN_();
    }
  }

  if (also_to_abort_controller) {
    coreState.udp.beginPacket(abort_controller_ip, abort_controller_port);
    coreState.udp.write(packetBuffer, packetSize);
    coreState.udp.endPacket();
    SENSOR_HOTFIRE_PRINT("Sent: sensor_data to ");
    SENSOR_HOTFIRE_PRINT(abort_controller_ip);
    SENSOR_HOTFIRE_PRINT(":");
    SENSOR_HOTFIRE_PRINTLN(abort_controller_port);
  }
  dataChunks.clear();
}

static void init_adc_cb(void*) {
  ADC_SPI.begin(Pins.ADC_SCLK, Pins.ADC_MISO, Pins.ADC_MOSI);
  ADC_SPI.setDataMode(SPI_MODE1);
  pinMode(Pins.ADC_DRDY_1, INPUT);
  ads126x.begin(Pins.ADC_CS_1, &ADC_SPI);
  ads126x.stopADC1();
  ads126x.setInputMux(static_cast<uint8_t>(getAdcChannel(1, TEST_PIN)), ADS126X_AINCOM);
  ads126x.bypassPGA();
  ads126x.setFilter(FILTER);
  ads126x.setRate(DATA_RATE);
  // Reference is not configured here; it must be set from server SENSOR_CONFIG
  ads126x.startADC1();
}

static void on_reference_voltage_cb(void*, uint8_t reference_voltage) {
  // 0 = internal 2.5V, 1 = VDD, 2 = 5V (ignored; use internal)
  uint8_t ref_pos = (reference_voltage == 1) ? ADS126X_REF_POS_VDD : ADS126X_REF_POS_INT;
  ads126x.setReference(ADS126X_REF_NEG_VSS, ref_pos);
}

static void collect_chunk_cb(void*) { collect_chunk_impl(); }

static void send_chunks_to_cb(void*, IPAddress dest_ip, int dest_port,
                             bool also_to_abort_controller,
                             IPAddress abort_controller_ip, int abort_controller_port) {
  send_chunks_to_impl(dest_ip, dest_port, also_to_abort_controller,
                      abort_controller_ip, abort_controller_port);
}

void setup() {
  memset(&coreState, 0, sizeof(coreState));
  coreState.gateway = IPAddress(0, 0, 0, 0);
  coreState.subnet = IPAddress(255, 255, 255, 0);
  coreState.dns = IPAddress(192, 168, 2, 1);

  coreConfig.board_type = Diablo::BoardType::PRESSURE_TRANSDUCER;
  coreConfig.board_name = "PT";
  coreConfig.update_server_on_sensor_config = false;
  coreConfig.debug_packets = false;
  coreConfig.pins = &Pins;
  coreConfig.init_adc = init_adc_cb;
  coreConfig.collect_chunk = collect_chunk_cb;
  coreConfig.send_chunks_to = send_chunks_to_cb;
  coreConfig.on_reference_voltage_config = on_reference_voltage_cb;
  coreConfig.user_data = nullptr;

  SensorHotfire::setup(coreState, coreConfig);
}

void loop() {
  SensorHotfire::loop(coreState, coreConfig);
}
