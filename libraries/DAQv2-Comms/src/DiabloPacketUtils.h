#pragma once

#include "DiabloEnums.h"   // For enums like PacketType
#include "DiabloPackets.h" // For all packet data structures
#include <Arduino.h>       // ESP32 Arduino core for standard library support
#include <stdint.h>        // For standard integer types
#include <vector>          // For std::vector

namespace Diablo {

//==============================================================================
// PACKET SERIALIZATION (Struct -> uint8_t* Buffer)
//
// These functions take high-level data structs, serialize them into a
// byte buffer, and return the final number of bytes to be sent.
// A return value of 0 indicates an error.
//==============================================================================

/**
 * @brief Creates a complete Board Heartbeat packet in the provided buffer.
 *
 * This is a fixed-size packet sent periodically by a board to the server to
 * indicate it is online and operational.
 *
 * @param data The heartbeat data to encode (board type, state, etc.).
 * @param buffer The output buffer to write the final packet into.
 * @param buffer_size The total size of the output buffer, used for safety
 * checks.
 * @return The number of bytes written to the buffer (always
 * sizeof(PacketHeader) + sizeof(BoardHeartbeatPacket)), or 0 on error.
 */
size_t create_board_heartbeat_packet(const BoardHeartbeatPacket &data,
                                     uint8_t *buffer, size_t buffer_size);

/**
 * @brief Creates a complete Sensor Data packet in the provided buffer.
 *
 * This is a variable-size packet containing readings from one or more sensor
 * data chunks. It consists of a header, a fixed-size body specifying the number
 * of chunks and sensors, followed by the actual sensor data chunks and their
 * datapoints.
 *
 * @param chunks A vector of SensorDataChunkCollection structs containing the
 * sensor data.
 * @param num_sensors The number of sensors that are included in the packet
 * @param buffer The output buffer to write the final packet into.
 * @param buffer_size The total size of the output buffer.
 * @return The total number of bytes written to the buffer, or 0 on error.
 */
size_t
create_sensor_data_packet(const std::vector<SensorDataChunkCollection> &chunks, const uint8_t num_sensors,
                          uint8_t *buffer, size_t buffer_size);

/**
 * @brief Creates a simple Abort Done packet.
 *
 * This packet is sent from a board to the server to acknowledge that it has
 * successfully completed its abort sequence. It has no data payload.
 *
 * @param buffer The output buffer to write the final packet into.
 * @param buffer_size The total size of the output buffer.
 * @return The number of bytes written (always sizeof(PacketHeader)), or 0 on
 * error.
 */
size_t create_abort_done_packet(uint8_t *buffer, size_t buffer_size);

/**
 * @brief Creates a No Connection Abort packet (header only).
 *
 * Sent by the server when it has lost connection; tells sensor boards to
 * transition to standalone abort and stream to the actuator controller.
 *
 * @param buffer The output buffer to write the final packet into.
 * @param buffer_size The total size of the output buffer.
 * @return The number of bytes written (always sizeof(PacketHeader)), or 0 on error.
 */
size_t create_no_connection_abort_packet(uint8_t *buffer, size_t buffer_size);

// Add declarations for other packets the boards will create...

/**
 * @brief Creates a complete Actuator Command packet in the provided buffer.
 *
 * Packet layout: PacketHeader + ActuatorCommandPacket + N ActuatorCommand.
 *
 * @param commands The list of actuator commands to serialize.
 * @param buffer The output buffer to write the packet into.
 * @param buffer_size The size of the provided buffer.
 * @return The total size of the created packet, or 0 on error.
 */
size_t create_actuator_command_packet(const std::vector<ActuatorCommand> &commands,
                                      uint8_t *buffer, size_t buffer_size);

//==============================================================================
// PACKET DESERIALIZATION (uint8_t* Buffer -> Struct)
//==============================================================================

/**
 * @brief Parses a Board Heartbeat packet from buffer.
 * @return true on success, false on error (size/type mismatch).
 */
bool parse_board_heartbeat_packet(const uint8_t *buffer, size_t buffer_size,
                                  PacketHeader &header_out,
                                  BoardHeartbeatPacket &data_out);

/**
 * @brief Parses a Server Heartbeat packet from buffer.
 * @return true on success, false on error (size/type mismatch).
 */
bool parse_server_heartbeat_packet(const uint8_t *buffer, size_t buffer_size,
                                    PacketHeader &header_out,
                                    ServerHeartbeatPacket &data_out);

/**
 * @brief Parses a Sensor Data packet from buffer into chunk collections.
 * @return true on success, false on error.
 */
bool parse_sensor_data_packet(const uint8_t *buffer, size_t buffer_size,
                              PacketHeader &header_out,
                              std::vector<SensorDataChunkCollection> &chunks_out);

/**
 * @brief Parses an Abort Done packet from buffer.
 * @return true on success, false on error.
 */
bool parse_abort_done_packet(const uint8_t *buffer, size_t buffer_size,
                             PacketHeader &header_out);

/**
 * @brief Parses a No Connection Abort packet from buffer (header only).
 * @return true on success, false on error.
 */
bool parse_no_connection_abort_packet(const uint8_t *buffer, size_t buffer_size,
                                      PacketHeader &header_out);

/**
 * @brief Parses an Actuator Command packet from buffer.
 * @return true on success, false on error.
 */
bool parse_actuator_command_packet(const uint8_t *buffer, size_t buffer_size,
                                   PacketHeader &header_out,
                                   std::vector<ActuatorCommand> &commands_out);

/**
 * @brief Creates a complete PWM Actuator Command packet in the provided buffer.
 *
 * Packet layout: PacketHeader + PWMActuatorCommandPacket + N PWMActuatorCommand.
 *
 * @param commands The list of PWM actuator commands to serialize.
 * @param buffer The output buffer to write the packet into.
 * @param buffer_size The size of the provided buffer.
 * @return The total size of the created packet, or 0 on error.
 */
size_t create_pwm_actuator_packet(const std::vector<PWMActuatorCommand> &commands,
                                  uint8_t *buffer, size_t buffer_size);

/**
 * @brief Parses a PWM Actuator Command packet from buffer.
 * @return true on success, false on error.
 */
bool parse_pwm_actuator_packet(const uint8_t *buffer, size_t buffer_size,
                               PacketHeader &header_out,
                               std::vector<PWMActuatorCommand> &commands_out);

/**
 * @brief Parses an Actuator Config packet from buffer.
 * Body layout: is_abort_controller(1), num_abort_actuators(1), N x AbortActuatorLocation(7 each), num_abort_pts(1), X x AbortPTLocation(6 each).
 * @param buffer Full packet (header + body).
 * @param buffer_size Total buffer size.
 * @param header_out Filled with packet header.
 * @param is_abort_controller_out Filled with is_abort_controller byte.
 * @param actuator_locations_out Filled with up to NUM_ABORT_ACTUATOR_LOCATIONS entries.
 * @param pt_locations_out Filled with up to NUM_ABORT_PT_LOCATIONS entries.
 * @return true on success, false on size/type mismatch.
 */
bool parse_actuator_config_packet(const uint8_t *buffer, size_t buffer_size,
                                PacketHeader &header_out,
                                uint8_t &is_abort_controller_out,
                                std::vector<AbortActuatorLocation> &actuator_locations_out,
                                std::vector<AbortPTLocation> &pt_locations_out);

} // namespace Diablo
