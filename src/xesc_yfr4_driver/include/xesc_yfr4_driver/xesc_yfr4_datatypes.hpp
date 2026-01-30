/**
 * @file xesc_yfr4_datatypes.hpp
 * @brief Data types for XESC YardForce R4 communication protocol
 * 
 * Ported from open_mower_ros (ROS1) to ROS2
 * Original author: Clemens Elflein
 */

#ifndef XESC_YFR4_DRIVER__XESC_YFR4_DATATYPES_HPP_
#define XESC_YFR4_DRIVER__XESC_YFR4_DATATYPES_HPP_

#include <cstdint>

namespace xesc_yfr4_driver
{

// Message types
constexpr uint8_t XESCYFR4_MSG_TYPE_STATUS = 1;
constexpr uint8_t XESCYFR4_MSG_TYPE_CONTROL = 2;
constexpr uint8_t XESCYFR4_MSG_TYPE_SETTINGS = 3;

// Fault codes (bitmask)
constexpr int32_t FAULT_UNINITIALIZED = 0b1;
constexpr int32_t FAULT_WATCHDOG = 0b10;
constexpr int32_t FAULT_UNDERVOLTAGE = 0b100;
constexpr int32_t FAULT_OVERVOLTAGE = 0b1000;
constexpr int32_t FAULT_OVERCURRENT = 0b10000;
constexpr int32_t FAULT_OVERTEMP_MOTOR = 0b100000;
constexpr int32_t FAULT_OVERTEMP_PCB = 0b1000000;
constexpr int32_t FAULT_INVALID_HALL = 0b10000000;
constexpr int32_t FAULT_INTERNAL_ERROR = 0b100000000;
constexpr int32_t FAULT_OPEN_LOAD = 0b1000000000;

#pragma pack(push, 1)

/**
 * @brief Status packet received from XESC YFR4
 */
struct XescYFR4StatusPacket
{
  uint8_t message_type;
  uint32_t seq;
  uint8_t fw_version_major;
  uint8_t fw_version_minor;
  double temperature_pcb;     ///< PCB temperature (degrees Celsius)
  double current_input;       ///< Input current (ampere)
  double duty_cycle;          ///< Duty cycle (0 to 1)
  bool direction;             ///< Direction CW/CCW
  uint32_t tacho;             ///< Wheel ticks
  uint32_t tacho_absolute;    ///< Wheel ticks absolute
  uint16_t rpm;               ///< Revolutions per minute (of the axis/shaft)
  int32_t fault_code;         ///< Fault code bitmask
  uint16_t crc;               ///< CRC checksum
};

/**
 * @brief Control packet sent to XESC YFR4
 */
struct XescYFR4ControlPacket
{
  uint8_t message_type;
  double duty_cycle;          ///< Duty cycle (0 to 1)
  uint16_t crc;               ///< CRC checksum
};

/**
 * @brief Settings packet sent to XESC YFR4
 */
struct XescYFR4SettingsPacket
{
  uint8_t message_type;
  float motor_current_limit;  ///< Motor current limit
  float min_pcb_temp;         ///< Minimum PCB temperature
  float max_pcb_temp;         ///< Maximum PCB temperature
  uint16_t crc;               ///< CRC checksum
};

#pragma pack(pop)

}  // namespace xesc_yfr4_driver

#endif  // XESC_YFR4_DRIVER__XESC_YFR4_DATATYPES_HPP_
