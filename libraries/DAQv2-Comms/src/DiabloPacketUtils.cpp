#include "DiabloPacketUtils.h"
#include "DAQv2-Comms.h"
#include <cstring> // For memcpy
#include <cstddef> // For size_t

namespace Diablo {

//==============================================================================
// PACKET SERIALIZATION IMPLEMENTATIONS
//==============================================================================


size_t create_board_heartbeat_packet(const BoardHeartbeatPacket &data,
                                     uint8_t *buffer, size_t buffer_size) {
  // Calculate the total packet size
  const size_t header_size = sizeof(PacketHeader);
  const size_t body_size = sizeof(BoardHeartbeatPacket);
  const size_t total_size = header_size + body_size;
  
  // Check if buffer is large enough
  if (buffer_size < total_size) {
    return 0; // Error: buffer too small
  }
  
  // Create the packet header
  PacketHeader header;
  header.packet_type = PacketType::BOARD_HEARTBEAT;
  header.version = DIABLO_COMMS_VERSION; // Current protocol version
  header.timestamp = millis(); // Use Arduino millis() for timestamp (ESP32 compatible) (ESP32 compatible)
  
  // Copy header to buffer
  memcpy(buffer, &header, header_size);
  
  // Copy body data to buffer
  memcpy(buffer + header_size, &data, body_size);
  
  return total_size;
}

size_t create_sensor_data_packet(const std::vector<SensorDataChunkCollection> &chunks, const uint8_t num_sensors,
                                uint8_t *buffer, size_t buffer_size) {
  const size_t header_size = sizeof(PacketHeader);
  const size_t body_header_size = sizeof(SensorDataPacket);

  const size_t num_chunks = chunks.size();

  // Compute total size: header + body + per-chunk (timestamp + datapoints)
  const size_t per_chunk_size = sizeof(SensorDataChunk) + (static_cast<size_t>(num_sensors) * sizeof(SensorDatapoint));
  const size_t total_size = header_size + body_header_size + (num_chunks * per_chunk_size);

  if (buffer_size < total_size) {
    return 0; // Buffer too small
  }

  // Prepare header
  PacketHeader header;
  header.packet_type = PacketType::SENSOR_DATA;
  header.version = DIABLO_COMMS_VERSION;
  header.timestamp = millis();

  // Write header
  uint8_t *ptr = buffer;
  memcpy(ptr, &header, header_size);
  ptr += header_size;

  // Write body header
  SensorDataPacket body;
  body.num_chunks = num_chunks;
  body.num_sensors = num_sensors;
  memcpy(ptr, &body, body_header_size);
  ptr += body_header_size;

  // Write chunks and datapoints
  for (uint8_t i = 0; i < num_chunks; ++i) {
    // Chunk header (timestamp)
    SensorDataChunk chunk_hdr;
    chunk_hdr.timestamp = chunks[i].timestamp;
    memcpy(ptr, &chunk_hdr, sizeof(SensorDataChunk));
    ptr += sizeof(SensorDataChunk);

    // Datapoints (assume exactly num_sensors datapoints are present)
    const SensorDatapoint *dp = chunks[i].datapoints.data();
    size_t datapoints_total_size = static_cast<size_t>(num_sensors) * sizeof(SensorDatapoint);
    memcpy(ptr, dp, datapoints_total_size);
    ptr += datapoints_total_size;
  }

  return total_size;
}

size_t create_abort_done_packet(uint8_t *buffer, size_t buffer_size) {
  // Calculate the total packet size (header only, no body)
  const size_t header_size = sizeof(PacketHeader);
  
  // Check if buffer is large enough
  if (buffer_size < header_size) {
    return 0; // Error: buffer too small
  }
  
  // Create the packet header
  PacketHeader header;
  header.packet_type = PacketType::ABORT_DONE;
  header.version = 1; // Current protocol version
  header.timestamp = millis(); // Use Arduino millis() for timestamp (ESP32 compatible)
  
  // Copy header to buffer
  memcpy(buffer, &header, header_size);
  
  return header_size;
}

size_t create_no_connection_abort_packet(uint8_t *buffer, size_t buffer_size) {
  const size_t header_size = sizeof(PacketHeader);
  if (buffer_size < header_size) return 0;

  PacketHeader header;
  header.packet_type = PacketType::NO_CONNECTION_ABORT;
  header.version = DIABLO_COMMS_VERSION;
  header.timestamp = millis();
  memcpy(buffer, &header, header_size);
  return header_size;
}

size_t create_actuator_command_packet(const std::vector<ActuatorCommand> &commands,
                                      uint8_t *buffer, size_t buffer_size) {
  const size_t header_size = sizeof(PacketHeader);
  const size_t body_size = sizeof(ActuatorCommandPacket);
  const size_t num_commands = commands.size();

  if (num_commands > 255 || num_commands == 0) {
    return 0; // num_commands must be between 1 and 255
  }

  const size_t commands_bytes = num_commands * sizeof(ActuatorCommand);
  const size_t total_size = header_size + body_size + commands_bytes;

  if (buffer_size < total_size) {
    return 0; // Buffer too small
  }

  // Header
  PacketHeader header;
  header.packet_type = PacketType::ACTUATOR_COMMAND;
  header.version = DIABLO_COMMS_VERSION;
  header.timestamp = millis();

  uint8_t *ptr = buffer;
  memcpy(ptr, &header, header_size);
  ptr += header_size;

  // Body
  ActuatorCommandPacket body;
  body.num_commands = static_cast<uint8_t>(num_commands);
  memcpy(ptr, &body, body_size);
  ptr += body_size;

  // Commands array
  if (num_commands) {
    memcpy(ptr, commands.data(), commands_bytes);
  }

  return total_size;
}

bool parse_board_heartbeat_packet(const uint8_t *buffer, size_t buffer_size,
                                  PacketHeader &header_out,
                                  BoardHeartbeatPacket &data_out) {
  const size_t header_size = sizeof(PacketHeader);
  const size_t body_size = sizeof(BoardHeartbeatPacket);
  const size_t total_size = header_size + body_size;
  if (!buffer || buffer_size < total_size) return false;

  // Read header
  PacketHeader hdr;
  memcpy(&hdr, buffer, header_size);
  if (hdr.packet_type != PacketType::BOARD_HEARTBEAT) return false;

  // Read body
  memcpy(&data_out, buffer + header_size, body_size);
  header_out = hdr;
  return true;
}

bool parse_server_heartbeat_packet(const uint8_t *buffer, size_t buffer_size,
                                    PacketHeader &header_out,
                                    ServerHeartbeatPacket &data_out) {
  const size_t header_size = sizeof(PacketHeader);
  const size_t body_size = sizeof(ServerHeartbeatPacket);
  const size_t total_size = header_size + body_size;
  if (!buffer || buffer_size < total_size) return false;

  // Read header
  PacketHeader hdr;
  memcpy(&hdr, buffer, header_size);
  if (hdr.packet_type != PacketType::SERVER_HEARTBEAT) return false;

  // Read body
  memcpy(&data_out, buffer + header_size, body_size);
  header_out = hdr;
  return true;
}

bool parse_sensor_data_packet(const uint8_t *buffer, size_t buffer_size,
                              PacketHeader &header_out,
                              std::vector<SensorDataChunkCollection> &chunks_out) {
  const size_t header_size = sizeof(PacketHeader);
  const size_t body_hdr_size = sizeof(SensorDataPacket);

  if (!buffer || buffer_size < header_size + body_hdr_size) return false;

  // Header
  PacketHeader hdr;
  memcpy(&hdr, buffer, header_size);
  if (hdr.packet_type != PacketType::SENSOR_DATA) return false;

  const uint8_t *ptr = buffer + header_size;
  // Body header
  SensorDataPacket body;
  memcpy(&body, ptr, body_hdr_size);
  ptr += body_hdr_size;

  const size_t per_chunk_size = sizeof(SensorDataChunk) + (static_cast<size_t>(body.num_sensors) * sizeof(SensorDatapoint));
  const size_t expected_size = header_size + body_hdr_size + (static_cast<size_t>(body.num_chunks) * per_chunk_size);
  if (buffer_size < expected_size) return false;

  chunks_out.clear();
  chunks_out.reserve(body.num_chunks);

  for (uint8_t c = 0; c < body.num_chunks; ++c) {
    // Chunk header
    SensorDataChunk chunk_hdr;
    memcpy(&chunk_hdr, ptr, sizeof(SensorDataChunk));
    ptr += sizeof(SensorDataChunk);

    // Datapoints
    SensorDataChunkCollection col(chunk_hdr.timestamp, body.num_sensors);
    if (body.num_sensors) {
      col.datapoints.resize(body.num_sensors);
      memcpy(col.datapoints.data(), ptr, static_cast<size_t>(body.num_sensors) * sizeof(SensorDatapoint));
      ptr += static_cast<size_t>(body.num_sensors) * sizeof(SensorDatapoint);
    }
    chunks_out.push_back(std::move(col));
  }

  header_out = hdr;
  return true;
}

bool parse_abort_done_packet(const uint8_t *buffer, size_t buffer_size,
                             PacketHeader &header_out) {
  const size_t header_size = sizeof(PacketHeader);
  if (!buffer || buffer_size < header_size) return false;

  PacketHeader hdr;
  memcpy(&hdr, buffer, header_size);
  if (hdr.packet_type != PacketType::ABORT_DONE) return false;
  header_out = hdr;
  return true;
}

bool parse_no_connection_abort_packet(const uint8_t *buffer, size_t buffer_size,
                                      PacketHeader &header_out) {
  const size_t header_size = sizeof(PacketHeader);
  if (!buffer || buffer_size < header_size) return false;

  PacketHeader hdr;
  memcpy(&hdr, buffer, header_size);
  if (hdr.packet_type != PacketType::NO_CONNECTION_ABORT) return false;
  header_out = hdr;
  return true;
}

bool parse_actuator_command_packet(const uint8_t *buffer, size_t buffer_size,
                                   PacketHeader &header_out,
                                   std::vector<ActuatorCommand> &commands_out) {
  const size_t header_size = sizeof(PacketHeader);
  const size_t body_size = sizeof(ActuatorCommandPacket);
  if (!buffer || buffer_size < header_size + body_size) return false;

  PacketHeader hdr;
  memcpy(&hdr, buffer, header_size);
  if (hdr.packet_type != PacketType::ACTUATOR_COMMAND) return false;

  const uint8_t *ptr = buffer + header_size;
  ActuatorCommandPacket body;
  memcpy(&body, ptr, body_size);
  ptr += body_size;

  const size_t commands_bytes = static_cast<size_t>(body.num_commands) * sizeof(ActuatorCommand);
  const size_t expected_size = header_size + body_size + commands_bytes;
  if (buffer_size < expected_size) return false;

  commands_out.clear();
  if (body.num_commands) {
    commands_out.resize(body.num_commands);
    memcpy(commands_out.data(), ptr, commands_bytes);
  }

  header_out = hdr;
  return true;
}

size_t create_pwm_actuator_packet(const std::vector<PWMActuatorCommand> &commands,
                                  uint8_t *buffer, size_t buffer_size) {
  const size_t header_size = sizeof(PacketHeader);
  const size_t body_size = sizeof(PWMActuatorCommandPacket);
  const size_t num_commands = commands.size();

  if (num_commands > 255 || num_commands == 0) {
    return 0; // num_commands must be between 1 and 255
  }

  const size_t commands_bytes = num_commands * sizeof(PWMActuatorCommand);
  const size_t total_size = header_size + body_size + commands_bytes;

  if (buffer_size < total_size) {
    return 0; // Buffer too small
  }

  // Header
  PacketHeader header;
  header.packet_type = PacketType::PWM_ACTUATOR_COMMAND;
  header.version = DIABLO_COMMS_VERSION;
  header.timestamp = millis();

  uint8_t *ptr = buffer;
  memcpy(ptr, &header, header_size);
  ptr += header_size;

  // Body
  PWMActuatorCommandPacket body;
  body.num_commands = static_cast<uint8_t>(num_commands);
  memcpy(ptr, &body, body_size);
  ptr += body_size;

  // Commands array
  if (num_commands) {
    memcpy(ptr, commands.data(), commands_bytes);
  }

  return total_size;
}

bool parse_pwm_actuator_packet(const uint8_t *buffer, size_t buffer_size,
                               PacketHeader &header_out,
                               std::vector<PWMActuatorCommand> &commands_out) {
  const size_t header_size = sizeof(PacketHeader);
  const size_t body_size = sizeof(PWMActuatorCommandPacket);
  if (!buffer || buffer_size < header_size + body_size) return false;

  PacketHeader hdr;
  memcpy(&hdr, buffer, header_size);
  if (hdr.packet_type != PacketType::PWM_ACTUATOR_COMMAND) return false;

  const uint8_t *ptr = buffer + header_size;
  PWMActuatorCommandPacket body;
  memcpy(&body, ptr, body_size);
  ptr += body_size;

  const size_t commands_bytes = static_cast<size_t>(body.num_commands) * sizeof(PWMActuatorCommand);
  const size_t expected_size = header_size + body_size + commands_bytes;
  if (buffer_size < expected_size) return false;

  commands_out.clear();
  if (body.num_commands) {
    commands_out.resize(body.num_commands);
    memcpy(commands_out.data(), ptr, commands_bytes);
  }

  header_out = hdr;
  return true;
}

bool parse_actuator_config_packet(const uint8_t *buffer, size_t buffer_size,
                                  PacketHeader &header_out,
                                  uint8_t &is_abort_controller_out,
                                  std::vector<AbortActuatorLocation> &actuator_locations_out,
                                  std::vector<AbortPTLocation> &pt_locations_out) {
  const size_t header_size = sizeof(PacketHeader);
  if (!buffer || buffer_size < header_size + 2) return false;  // at least header + is_abort_controller + N

  PacketHeader hdr;
  memcpy(&hdr, buffer, header_size);
  if (hdr.packet_type != PacketType::ACTUATOR_CONFIG) return false;

  const uint8_t *ptr = buffer + header_size;
  is_abort_controller_out = ptr[0];
  const uint8_t N = ptr[1];
  ptr += 2;

  const size_t actuator_block = static_cast<size_t>(N) * sizeof(AbortActuatorLocation);
  if (buffer_size < header_size + 2 + actuator_block + 1) return false;  // need at least num_abort_pts

  actuator_locations_out.clear();
  actuator_locations_out.reserve(N);
  for (uint8_t i = 0; i < N && actuator_locations_out.size() < NUM_ABORT_ACTUATOR_LOCATIONS; i++) {
    AbortActuatorLocation loc;
    memcpy(&loc, ptr, sizeof(AbortActuatorLocation));
    actuator_locations_out.push_back(loc);
    ptr += sizeof(AbortActuatorLocation);
  }
  ptr = buffer + header_size + 2 + actuator_block;  // in case N was larger than MAX we skipped some
  const uint8_t X = *ptr;
  ptr += 1;

  const size_t pt_block = static_cast<size_t>(X) * sizeof(AbortPTLocation);
  if (buffer_size < header_size + 2 + actuator_block + 1 + pt_block) return false;

  pt_locations_out.clear();
  pt_locations_out.reserve(X);
  for (uint8_t i = 0; i < X && pt_locations_out.size() < NUM_ABORT_PT_LOCATIONS; i++) {
    AbortPTLocation loc;
    memcpy(&loc, ptr, sizeof(AbortPTLocation));
    pt_locations_out.push_back(loc);
    ptr += sizeof(AbortPTLocation);
  }

  header_out = hdr;
  return true;
}

} // namespace Diablo
