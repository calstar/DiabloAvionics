#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "daqv2/fsm.hpp"
#include "daqv2/platform_network.hpp"

enum : uint8_t { PKT_HB = 0, PKT_SENSOR = 1, PKT_ABORT = 2, PKT_SERVER_HB = 3 };
constexpr uint16_t PKT_MAGIC = 0xDA26;

#pragma pack(push, 1)
struct PacketHeader {
    uint16_t magic;
    uint8_t  version;
    uint8_t  type;
    uint8_t  board_id;
    uint8_t  reserved;
    uint32_t seq;
    uint64_t timestamp_us;
    uint16_t payload_len;
    uint16_t crc16;
};
struct HeartbeatPayload {
    uint8_t status;
};
struct SensorPayload {
    float PT_HP, PT_LP, PT_F, PT_O;
    float RTD_O1, RTD_O2, RTD_O3;
    float PT_I, TC_I, PT_C1, TC_C1, TC_C3, TC_C2, PT_C2, TC_C4;
    uint8_t sol_bitmap;
};
#pragma pack(pop)

static_assert(sizeof(PacketHeader) == 22, "PacketHeader size unexpected");
static_assert(sizeof(HeartbeatPayload) == 1, "HeartbeatPayload size unexpected");

void fill_sensor_payload(const SystemParameters& p, SensorPayload& sp);

uint16_t telemetry_crc16(const uint8_t* data, size_t len);

template<typename PayloadT>
std::vector<uint8_t> make_packet(uint8_t type, uint8_t board_id,
                                 uint32_t seq, uint64_t ts_us,
                                 const PayloadT& pld);

bool parse_and_verify(const uint8_t* buf, size_t len,
                      PacketHeader& out_hdr, const uint8_t*& out_payload);

#include "telemetry.inl"
