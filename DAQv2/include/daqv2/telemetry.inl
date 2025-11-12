#pragma once

#include <cstring>

template<typename PayloadT>
std::vector<uint8_t> make_packet(uint8_t type, uint8_t board_id,
                                 uint32_t seq, uint64_t ts_us,
                                 const PayloadT& pld)
{
    PacketHeader hdr{};
    hdr.magic   = htons(PKT_MAGIC);
    hdr.version = 1;
    hdr.type    = type;
    hdr.board_id= board_id;
    hdr.reserved= 0;
    hdr.seq     = htonl(seq);

    uint64_t ts_be = htobe64(ts_us);
    std::memcpy(&hdr.timestamp_us, &ts_be, sizeof(ts_be));

    hdr.payload_len = htons((uint16_t)sizeof(PayloadT));
    hdr.crc16 = 0;

    std::vector<uint8_t> buf(sizeof(PacketHeader) + sizeof(PayloadT));
    std::memcpy(buf.data(), &hdr, sizeof(PacketHeader));
    std::memcpy(buf.data()+sizeof(PacketHeader), &pld, sizeof(PayloadT));

    uint16_t crc = telemetry_crc16(buf.data(), buf.size());
    reinterpret_cast<PacketHeader*>(buf.data())->crc16 = htons(crc);
    return buf;
}

