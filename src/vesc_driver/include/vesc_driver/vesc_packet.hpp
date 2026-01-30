/**
 * @file vesc_packet.hpp
 * @brief VESC Packet definitions for serial communication
 * 
 * Ported from open_mower_ros (ROS1) to ROS2
 * Original copyright: SoftBank Corp. (BSD 3-clause)
 * Original author: Michael T. Boulet (MIT)
 */

#ifndef VESC_DRIVER__VESC_PACKET_HPP_
#define VESC_DRIVER__VESC_PACKET_HPP_

#include <cassert>
#include <cstdint>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <boost/crc.hpp>
#include <boost/range/begin.hpp>
#include <boost/range/distance.hpp>
#include <boost/range/end.hpp>

#include "vesc_driver/data_map.hpp"

namespace vesc_driver
{

using Buffer = std::vector<uint8_t>;
using BufferRange = std::pair<Buffer::iterator, Buffer::iterator>;
using BufferRangeConst = std::pair<Buffer::const_iterator, Buffer::const_iterator>;

/**
 * @brief The raw frame for communicating with the VESC
 */
class VescFrame
{
public:
  virtual ~VescFrame() = default;

  /**
   * @brief Gets a reference of the frame
   * @return Reference of the frame
   */
  virtual const Buffer & getFrame() const
  {
    return frame_;
  }

  // Packet properties
  static constexpr int16_t VESC_MAX_PAYLOAD_SIZE = 1024;
  static constexpr int16_t VESC_MIN_FRAME_SIZE = 5;
  static constexpr int16_t VESC_MAX_FRAME_SIZE = 6 + VESC_MAX_PAYLOAD_SIZE;
  static constexpr int16_t VESC_SOF_VAL_SMALL_FRAME = 2;
  static constexpr int16_t VESC_SOF_VAL_LARGE_FRAME = 3;
  static constexpr int16_t VESC_EOF_VAL = 3;

  /**
   * @brief CRC parameters for the VESC
   */
  using CRC = boost::crc_optimal<16, 0x1021, 0, 0, false, false>;

protected:
  explicit VescFrame(const int16_t payload_size);

  Buffer frame_;
  BufferRange payload_end_;

private:
  /**
   * @brief Private constructor for VescPacketFactory
   * @param frame View of the raw frame bytes
   * @param payload View of the payload section
   */
  VescFrame(const BufferRangeConst & frame, const BufferRangeConst & payload);

  // VescPacketFactory needs access to the private constructor
  friend class VescPacketFactory;
};

/**
 * @brief VescFrame with a non-zero length payload
 */
class VescPacket : public VescFrame
{
public:
  virtual ~VescPacket() = default;

  virtual const std::string & getName() const
  {
    return name_;
  }

protected:
  VescPacket(const std::string & name, const int16_t payload_size, const int16_t payload_id);
  VescPacket(const std::string & name, std::shared_ptr<VescFrame> raw);

private:
  std::string name_;
};

using VescPacketPtr = std::shared_ptr<VescPacket>;
using VescPacketConstPtr = std::shared_ptr<VescPacket const>;

/**
 * @brief Firmware version packet
 */
class VescPacketFWVersion : public VescPacket
{
public:
  explicit VescPacketFWVersion(std::shared_ptr<VescFrame> raw);

  int16_t fwMajor() const;
  int16_t fwMinor() const;
};

/**
 * @brief Requests firmware version
 */
class VescPacketRequestFWVersion : public VescPacket
{
public:
  VescPacketRequestFWVersion();
};

/**
 * @brief Gets values in return packets
 */
class VescPacketValues : public VescPacket
{
public:
  explicit VescPacketValues(std::shared_ptr<VescFrame> raw);

  double getMosTemp() const;
  double getMotorTemp() const;
  double getMotorCurrent() const;
  double getInputCurrent() const;
  double getVelocityERPM() const;
  double getInputVoltage() const;
  double getDuty() const;
  double getConsumedCharge() const;
  double getInputCharge() const;
  double getConsumedPower() const;
  double getInputPower() const;
  uint32_t getPosition() const;
  uint32_t getDisplacement() const;
  int getFaultCode() const;

private:
  double readBuffer(const uint8_t, const uint8_t) const;
};

/**
 * @brief Packet for requesting return packets
 */
class VescPacketRequestValues : public VescPacket
{
public:
  VescPacketRequestValues();
};

/**
 * @brief Packet for setting duty
 */
class VescPacketSetDuty : public VescPacket
{
public:
  explicit VescPacketSetDuty(double duty);
};

/**
 * @brief Packet for setting reference current
 */
class VescPacketSetCurrent : public VescPacket
{
public:
  explicit VescPacketSetCurrent(double current);
};

/**
 * @brief Packet for setting current brake
 */
class VescPacketSetCurrentBrake : public VescPacket
{
public:
  explicit VescPacketSetCurrentBrake(double current_brake);
};

/**
 * @brief Packet for setting reference angular velocity
 */
class VescPacketSetVelocityERPM : public VescPacket
{
public:
  explicit VescPacketSetVelocityERPM(double vel_erpm);
};

/**
 * @brief Packet for setting a reference position
 */
class VescPacketSetPos : public VescPacket
{
public:
  explicit VescPacketSetPos(double pos);
};

/**
 * @brief Packet for setting a servo position
 */
class VescPacketSetServoPos : public VescPacket
{
public:
  explicit VescPacketSetServoPos(double servo_pos);
};

}  // namespace vesc_driver

#endif  // VESC_DRIVER__VESC_PACKET_HPP_
