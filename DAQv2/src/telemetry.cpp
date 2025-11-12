#include "daqv2/telemetry.hpp"

#include <cstring>
#include <vector>

void fill_sensor_payload(const SystemParameters& p, SensorPayload& sp) {
    sp.PT_HP=p.PT_HP; sp.PT_LP=p.PT_LP; sp.PT_F=p.PT_F; sp.PT_O=p.PT_O;
    sp.RTD_O1=p.RTD_O1; sp.RTD_O2=p.RTD_O2; sp.RTD_O3=p.RTD_O3;
    sp.PT_I=p.PT_I; sp.TC_I=p.TC_I; sp.PT_C1=p.PT_C1; sp.TC_C1=p.TC_C1;
    sp.TC_C3=p.TC_C3; sp.TC_C2=p.TC_C2; sp.PT_C2=p.PT_C2; sp.TC_C4=p.TC_C4;
    sp.sol_bitmap =
        (p.SOL_PV?1<<0:0)|(p.SOL_FUP?1<<1:0)|(p.SOL_FV?1<<2:0)|
        (p.SOL_OUP?1<<3:0)|(p.SOL_OV?1<<4:0)|(p.SOL_FDP?1<<5:0)|
        (p.MOT_ODP?1<<6:0);
}

uint16_t telemetry_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else              crc = (crc << 1);
        }
    }
    return crc;
}

bool parse_and_verify(const uint8_t* buf, size_t len,
                      PacketHeader& out_hdr, const uint8_t*& out_payload)
{
    if (len < sizeof(PacketHeader)) return false;
    std::memcpy(&out_hdr, buf, sizeof(PacketHeader));
    if (ntohs(out_hdr.magic) != PKT_MAGIC) return false;

    uint16_t reported = ntohs(out_hdr.crc16);
    std::vector<uint8_t> tmp(buf, buf + len);
    reinterpret_cast<PacketHeader*>(tmp.data())->crc16 = 0;
    if (telemetry_crc16(tmp.data(), tmp.size()) != reported) return false;

    uint16_t paylen = ntohs(out_hdr.payload_len);
    if (sizeof(PacketHeader) + paylen != len) return false;

    out_hdr.seq = ntohl(out_hdr.seq);
    uint64_t ts_be = 0;
    std::memcpy(&ts_be, &out_hdr.timestamp_us, sizeof(ts_be));
    ts_be = be64toh(ts_be);
    out_hdr.timestamp_us = ts_be;

    out_payload = buf + sizeof(PacketHeader);
    return true;
}
