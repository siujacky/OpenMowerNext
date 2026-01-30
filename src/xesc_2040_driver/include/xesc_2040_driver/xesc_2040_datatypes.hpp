/**
 * @file xesc_2040_datatypes.hpp
 * @brief Data types for XESC 2040 communication protocol
 * 
 * Ported from open_mower_ros (ROS1) to ROS2
 * Original author: Clemens Elflein
 */

#ifndef XESC_2040_DRIVER__XESC_2040_DATATYPES_HPP_
#define XESC_2040_DRIVER__XESC_2040_DATATYPES_HPP_

#include <cstdint>

namespace xesc_2040_driver
{

// Message types
constexpr uint8_t XESC2040_MSG_TYPE_STATUS = 1;
constexpr uint8_t XESC2040_MSG_TYPE_CONTROL = 2;
constexpr uint8_t XESC2040_MSG_TYPE_SETTINGS = 3;

// Fault codes (bitmask)
constexpr int32_t FAULT_UNINITIALIZED = 0b1;
constexpr int32_t FAULT_WATCHDOG = 0b10;
constexpr int32_t FAULT_UNDERVOLTAGE = 0b100;
constexpr int32_t FAULT_OVERVOLTAGE = 0b1000;
constexpr int32_t FAULT_OVERCURRENT = 0b10000;
constexpr int32_t FAULT_OVERTEMP_MOTOR = 0b100000;
constexpr int32_t FAULT_OVERTEMP_PCB = 0b1000000;
constexpr int32_t FAULT_INVALID_HALL = 0b10000000;

#pragma pack(push, 1)

/**
 * @brief Status packet received from XESC 2040
 */
struct Xesc2040StatusPacket
{
  uint8_t message_type;
  uint32_t seq;
  uint8_t fw_version_major;
  uint8_t fw_version_minor;
  double voltage_input;       ///< Input voltage (volt)
  double temperature_pcb;     ///< PCB temperature (degrees Celsius)
  double temperature_motor;   ///< Motor temperature (degrees Celsius)
  double current_input;       ///< Input current (ampere)
  double duty_cycle;          ///< Duty cycle (0 to 1)
  uint32_t tacho;             ///< Tachometer count
  uint32_t tacho_absolute;    ///< Wheel ticks absolute
  bool direction;             ///< Direction CW/CCW
  int32_t fault_code;         ///< Fault code bitmask
  uint16_t crc;               ///< CRC checksum
};

/**
 * @brief Control packet sent to XESC 2040
 */
struct Xesc2040ControlPacket
{
  uint8_t message_type;
  double duty_cycle;          ///< Duty cycle (0 to 1)
  uint16_t crc;               ///< CRC checksum
};

/**
 * @brief Settings packet sent to XESC 2040
 */
struct Xesc2040SettingsPacket
{
  uint8_t message_type;
  uint8_t hall_table[8];      ///< Hall sensor lookup table
  float motor_current_limit;  ///< Motor current limit
  float acceleration;         ///< Acceleration setting
  bool has_motor_temp;        ///< Whether motor temperature sensor is present
  float min_motor_temp;       ///< Minimum motor temperature
  float max_motor_temp;       ///< Maximum motor temperature
  float min_pcb_temp;         ///< Minimum PCB temperature
  float max_pcb_temp;         ///< Maximum PCB temperature
  uint16_t crc;               ///< CRC checksum
};

#pragma pack(pop)

}  // namespace xesc_2040_driver

#endif  // XESC_2040_DRIVER__XESC_2040_DATATYPES_HPP_
