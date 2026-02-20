// ethernet_packets.hpp
#pragma once
#include <cstdint>
#include <vector>

// -----------------------------------------------------------------------------
// Packet Type Definitions
// -----------------------------------------------------------------------------
enum PacketType : uint8_t
{
    BOARD_HEARTBEAT = 1,
    SERVER_HEARTBEAT = 2,
    SENSOR_DATA = 3,
    ACTUATOR_COMMAND = 4,
    SENSOR_CONFIG = 5,
    ACTUATOR_CONFIG = 6,
    ABORT = 7,
    CLEAR_ABORT = 8,
    PWM_ACTUATOR_COMMAND = 10,
    NO_CONNECTION_ABORT = 11
};

// -----------------------------------------------------------------------------
// Helper: Append bytes to buffer
// -----------------------------------------------------------------------------
inline void append_bytes(std::vector<uint8_t> &buf, const void *data, size_t size)
{
    const uint8_t *ptr = static_cast<const uint8_t *>(data);
    buf.insert(buf.end(), ptr, ptr + size);
}

template <typename T>
inline void append(std::vector<uint8_t> &buf, const T &value)
{
    append_bytes(buf, &value, sizeof(T));
}

// -----------------------------------------------------------------------------
// Common Header
// -----------------------------------------------------------------------------
inline void append_header(std::vector<uint8_t> &buf, PacketType type, uint8_t version, uint32_t timestamp)
{
    append<uint8_t>(buf, static_cast<uint8_t>(type));
    append<uint8_t>(buf, version);
    append<uint32_t>(buf, timestamp);
}

// -----------------------------------------------------------------------------
// 1. Board Heartbeat Packet
// -----------------------------------------------------------------------------
inline std::vector<uint8_t> make_board_heartbeat(uint8_t version, uint32_t timestamp,
                                                 uint8_t board_type, uint8_t board_id,
                                                 bool is_actuator, uint8_t state = 0, bool is_in_abort = false)
{
    std::vector<uint8_t> pkt;
    append_header(pkt, BOARD_HEARTBEAT, version, timestamp);

    uint8_t type_and_id = ((board_type & 0x0F) << 4) | (board_id & 0x0F);
    append<uint8_t>(pkt, type_and_id);

    if (is_actuator)
    {
        append<uint8_t>(pkt, state);
        append<uint8_t>(pkt, static_cast<uint8_t>(is_in_abort));
    }
    return pkt;
}

// -----------------------------------------------------------------------------
// 2. Server Heartbeat Packet
// -----------------------------------------------------------------------------
inline std::vector<uint8_t> make_server_heartbeat(uint8_t version, uint32_t timestamp)
{
    std::vector<uint8_t> pkt;
    append_header(pkt, SERVER_HEARTBEAT, version, timestamp);
    return pkt;
}

// -----------------------------------------------------------------------------
// 3. Sensor Data Packet
// -----------------------------------------------------------------------------
struct SensorDataPoint
{
    uint8_t id;
    float value;
};

inline std::vector<uint8_t> make_sensor_data(uint8_t version, uint32_t timestamp,
                                             const std::vector<std::vector<SensorDataPoint>> &chunks)
{
    std::vector<uint8_t> pkt;
    append_header(pkt, SENSOR_DATA, version, timestamp);

    uint8_t num_chunks = static_cast<uint8_t>(chunks.size());
    append<uint8_t>(pkt, num_chunks);

    for (const auto &chunk : chunks)
    {
        uint8_t num_sensors = static_cast<uint8_t>(chunk.size());
        append<uint8_t>(pkt, num_sensors);

        uint32_t chunk_timestamp = timestamp; // could differ per chunk
        append<uint32_t>(pkt, chunk_timestamp);

        for (const auto &s : chunk)
        {
            append<uint8_t>(pkt, s.id);
            append<float>(pkt, s.value);
        }
    }
    return pkt;
}

// -----------------------------------------------------------------------------
// 4. Actuator Command Packet
// -----------------------------------------------------------------------------
struct ActuatorCommand
{
    uint8_t id;
    uint8_t state;
};

inline std::vector<uint8_t> make_actuator_command(uint8_t version, uint32_t timestamp,
                                                  const std::vector<ActuatorCommand> &cmds)
{
    std::vector<uint8_t> pkt;
    append_header(pkt, ACTUATOR_COMMAND, version, timestamp);

    uint8_t num_cmds = static_cast<uint8_t>(cmds.size());
    append<uint8_t>(pkt, num_cmds);

    for (const auto &c : cmds)
    {
        append<uint8_t>(pkt, c.id);
        append<uint8_t>(pkt, c.state);
    }
    return pkt;
}

// -----------------------------------------------------------------------------
// 5. Sensor Config Packet
// -----------------------------------------------------------------------------
inline std::vector<uint8_t> make_sensor_config(uint8_t version, uint32_t timestamp,
                                               const std::vector<uint8_t> &sensor_ids,
                                               bool necessary_for_abort, uint32_t controller_ip = 0)
{
    std::vector<uint8_t> pkt;
    append_header(pkt, SENSOR_CONFIG, version, timestamp);

    uint8_t num_sensors = static_cast<uint8_t>(sensor_ids.size());
    append<uint8_t>(pkt, num_sensors);

    for (auto id : sensor_ids)
        append<uint8_t>(pkt, id);

    append<uint8_t>(pkt, static_cast<uint8_t>(necessary_for_abort));
    if (necessary_for_abort)
        append<uint32_t>(pkt, controller_ip);

    return pkt;
}

// -----------------------------------------------------------------------------
// 6. Abort Packets
// -----------------------------------------------------------------------------
inline std::vector<uint8_t> make_abort(uint8_t version, uint32_t timestamp)
{
    std::vector<uint8_t> pkt;
    append_header(pkt, ABORT, version, timestamp);
    return pkt;
}

inline std::vector<uint8_t> make_clear_abort(uint8_t version, uint32_t timestamp)
{
    std::vector<uint8_t> pkt;
    append_header(pkt, CLEAR_ABORT, version, timestamp);
    return pkt;
}

inline std::vector<uint8_t> make_no_connection_abort(uint8_t version, uint32_t timestamp)
{
    std::vector<uint8_t> pkt;
    append_header(pkt, NO_CONNECTION_ABORT, version, timestamp);
    return pkt;
}
