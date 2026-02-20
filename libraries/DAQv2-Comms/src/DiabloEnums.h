#pragma once

#include <stdint.h>

namespace Diablo {

/**
 * @brief Defines the type of data contained in a packet.
 */
enum class PacketType : uint8_t {
  BOARD_HEARTBEAT = 1,
  SERVER_HEARTBEAT = 2,
  SENSOR_DATA = 3,
  ACTUATOR_COMMAND = 4,
  SENSOR_CONFIG = 5,
  ACTUATOR_CONFIG = 6,
  ABORT = 7,
  ABORT_DONE = 8,
  CLEAR_ABORT = 9,
  PWM_ACTUATOR_COMMAND = 10,
  NO_CONNECTION_ABORT = 11
};

/**
 * @brief Defines the operational state of a board.
 */
enum class BoardState : uint8_t {
  SETUP = 1,
  ACTIVE = 2,
  ABORT = 3,
  ABORT_DONE = 4
};

/**
 * @brief Defines the physical type of a board.
 * Used in the Board Heartbeat packet.
 */
enum class BoardType : uint8_t {
  UNKNOWN = 0,
  PRESSURE_TRANSDUCER = 1,
  LOAD_CELL = 2,
  RTD = 3,
  THERMOCOUPLE = 4,
  ACTUATOR = 5
};

/**
 * @brief Defines the overall state of the engine system.
 * This is communicated from the server to the boards.
 */
enum class EngineState : uint8_t {
  // These are examples; the full list would be defined by the system leads.
  SAFE = 0,
  PRESSURIZING = 1,
  LOX_FILL = 2,
  FIRING = 3,
  POST_FIRE = 4
};

// TODO: Configure these enums based on actual system design
enum class ActuatorPurpose : uint8_t {
  MAIN_VALVE = 1,
  ABORT_VALVE = 2,
  DUMP_VALVE = 3,
  OTHER = 255
};

enum class PTPurpose : uint8_t {
  LOX_PRESSURE = 1,
  FUEL_PRESSURE = 2,
  CHAMBER_PRESSURE = 3,
  OTHER = 255
};

} // namespace Diablo
