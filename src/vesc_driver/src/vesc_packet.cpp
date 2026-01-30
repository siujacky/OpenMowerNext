/**
 * @file vesc_packet.cpp
 * @brief VESC Packet implementation
 * 
 * Ported from open_mower_ros (ROS1) to ROS2
 * Original copyright: SoftBank Corp. (BSD 3-clause)
 * Original author: Michael T. Boulet (MIT)
 */

#include "vesc_driver/vesc_packet.hpp"

namespace vesc_driver
{

VescFrame::VescFrame(const int16_t payload_size)
{
  assert(payload_size >= 0 && payload_size <= 1024);

  if (payload_size < 256) {
    // single byte payload size
    frame_.resize(VESC_MIN_FRAME_SIZE + payload_size);
    *(frame_.begin()) = 2;
    *(frame_.begin() + 1) = payload_size;
    payload_end_.first = frame_.begin() + 2;
  } else {
    // two byte payload size
    frame_.resize(VESC_MIN_FRAME_SIZE + 1 + payload_size);
    *(frame_.begin()) = 3;
    *(frame_.begin() + 1) = payload_size >> 8;
    *(frame_.begin() + 2) = payload_size & 0xFF;
    payload_end_.first = frame_.begin() + 3;
  }

  payload_end_.second = payload_end_.first + payload_size;
  *(frame_.end() - 1) = 3;
}

VescFrame::VescFrame(const BufferRangeConst & frame, const BufferRangeConst & payload)
{
  assert(boost::distance(frame) >= VESC_MIN_FRAME_SIZE);
  assert(boost::distance(frame) <= VESC_MAX_FRAME_SIZE);
  assert(boost::distance(payload) <= VESC_MAX_PAYLOAD_SIZE);
  assert(
    std::distance(frame.first, payload.first) > 0 &&
    std::distance(payload.second, frame.second) > 0);

  frame_.resize(std::distance(boost::begin(frame), boost::end(frame)));
  frame_.assign(boost::begin(frame), boost::end(frame));
  payload_end_.first = frame_.begin() + std::distance(frame.first, payload.first);
  payload_end_.second = frame_.begin() + std::distance(frame.first, payload.second);
}

/*------------------------------------------------------------------*/

VescPacket::VescPacket(
  const std::string & name,
  const int16_t payload_size,
  const int16_t payload_id)
: VescFrame(payload_size), name_(name)
{
  assert(payload_id >= 0 && payload_id < 256);
  assert(boost::distance(payload_end_) > 0);
  *payload_end_.first = payload_id;
}

VescPacket::VescPacket(const std::string & name, std::shared_ptr<VescFrame> raw)
: VescFrame(*raw), name_(name)
{
  uint16_t original_payload_size = std::distance(payload_end_.first, payload_end_.second);
  payload_end_.first = frame_.begin() + 2;
  payload_end_.second = std::min(payload_end_.first + original_payload_size, frame_.end());
}

/*------------------------------------------------------------------*/

VescPacketFWVersion::VescPacketFWVersion(std::shared_ptr<VescFrame> raw)
: VescPacket("FWVersion", raw)
{
}

int16_t VescPacketFWVersion::fwMajor() const
{
  return *(payload_end_.first + 1);
}

int16_t VescPacketFWVersion::fwMinor() const
{
  return *(payload_end_.first + 2);
}

/*------------------------------------------------------------------*/

VescPacketRequestFWVersion::VescPacketRequestFWVersion()
: VescPacket("RequestFWVersion", 1, COMM_FW_VERSION)
{
  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

VescPacketValues::VescPacketValues(std::shared_ptr<VescFrame> raw)
: VescPacket("Values", raw)
{
}

double VescPacketValues::getMosTemp() const
{
  return readBuffer(TEMP_MOS, 2) / 10.0;
}

double VescPacketValues::getMotorTemp() const
{
  return readBuffer(TEMP_MOTOR, 2) / 10.0;
}

double VescPacketValues::getMotorCurrent() const
{
  return readBuffer(CURRENT_MOTOR, 4) / 100.0;
}

double VescPacketValues::getInputCurrent() const
{
  return readBuffer(CURRENT_IN, 4) / 100.0;
}

double VescPacketValues::getDuty() const
{
  int16_t duty_raw = static_cast<int32_t>(readBuffer(DUTY_NOW, 2));

  if (duty_raw > 1000) {
    duty_raw = !duty_raw;
  }

  return static_cast<double>(duty_raw) / 1000.0;
}

double VescPacketValues::getVelocityERPM() const
{
  return readBuffer(ERPM, 4);
}

double VescPacketValues::getInputVoltage() const
{
  return readBuffer(VOLTAGE_IN, 2) / 10.0;
}

double VescPacketValues::getConsumedCharge() const
{
  return readBuffer(AMP_HOURS, 4) / 10000.0;
}

double VescPacketValues::getInputCharge() const
{
  return readBuffer(AMP_HOURS_CHARGED, 4) / 10000.0;
}

double VescPacketValues::getConsumedPower() const
{
  return readBuffer(WATT_HOURS, 4) / 10000.0;
}

double VescPacketValues::getInputPower() const
{
  return readBuffer(WATT_HOURS, 4) / 10000.0;
}

uint32_t VescPacketValues::getPosition() const
{
  int32_t value = 0;
  value += static_cast<int32_t>(*(payload_end_.first + TACHOMETER) << 24);
  value += static_cast<int32_t>(*(payload_end_.first + TACHOMETER + 1) << 16);
  value += static_cast<int32_t>(*(payload_end_.first + TACHOMETER + 2) << 8);
  value += static_cast<int32_t>(*(payload_end_.first + TACHOMETER + 3));
  return static_cast<uint32_t>(value);
}

uint32_t VescPacketValues::getDisplacement() const
{
  int32_t value = 0;
  value += static_cast<int32_t>(*(payload_end_.first + TACHOMETER_ABS) << 24);
  value += static_cast<int32_t>(*(payload_end_.first + TACHOMETER_ABS + 1) << 16);
  value += static_cast<int32_t>(*(payload_end_.first + TACHOMETER_ABS + 2) << 8);
  value += static_cast<int32_t>(*(payload_end_.first + TACHOMETER_ABS + 3));
  return static_cast<uint32_t>(value);
}

int VescPacketValues::getFaultCode() const
{
  return static_cast<int32_t>(*(payload_end_.first + FAULT_CODE));
}

double VescPacketValues::readBuffer(const uint8_t map_id, const uint8_t size) const
{
  int32_t value = 0;
  switch (size) {
    case 2:
      value = static_cast<int16_t>(
        (*(payload_end_.first + map_id) << 8) |
        (*(payload_end_.first + map_id + 1)));
      break;
    case 4:
      value += static_cast<int32_t>(*(payload_end_.first + map_id) << 24);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 1) << 16);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 2) << 8);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 3));
      break;
  }
  return static_cast<double>(value);
}

/*------------------------------------------------------------------*/

VescPacketRequestValues::VescPacketRequestValues()
: VescPacket("RequestFWVersion", 1, COMM_GET_VALUES)
{
  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

VescPacketSetDuty::VescPacketSetDuty(double duty)
: VescPacket("SetDuty", 5, COMM_SET_DUTY)
{
  if (duty > 1.0) {
    duty = 1.0;
  } else if (duty < -1.0) {
    duty = -1.0;
  }

  const int32_t v = static_cast<int32_t>(duty * 100000.0);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 24) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>((v >> 16) & 0xFF);
  *(payload_end_.first + 3) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 4) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

VescPacketSetCurrent::VescPacketSetCurrent(double current)
: VescPacket("SetCurrent", 5, COMM_SET_CURRENT)
{
  const int32_t v = static_cast<int32_t>(current * 1000.0);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 24) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>((v >> 16) & 0xFF);
  *(payload_end_.first + 3) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 4) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

VescPacketSetCurrentBrake::VescPacketSetCurrentBrake(double current_brake)
: VescPacket("SetCurrentBrake", 5, COMM_SET_CURRENT_BRAKE)
{
  const int32_t v = static_cast<int32_t>(current_brake * 1000.0);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 24) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>((v >> 16) & 0xFF);
  *(payload_end_.first + 3) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 4) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

VescPacketSetVelocityERPM::VescPacketSetVelocityERPM(double vel_erpm)
: VescPacket("SetERPM", 5, COMM_SET_ERPM)
{
  const int32_t v = static_cast<int32_t>(vel_erpm);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 24) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>((v >> 16) & 0xFF);
  *(payload_end_.first + 3) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 4) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

VescPacketSetPos::VescPacketSetPos(double pos)
: VescPacket("SetPos", 5, COMM_SET_POS)
{
  const int32_t v = static_cast<int32_t>(pos * 1000000.0);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 24) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>((v >> 16) & 0xFF);
  *(payload_end_.first + 3) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 4) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

VescPacketSetServoPos::VescPacketSetServoPos(double servo_pos)
: VescPacket("SetServoPos", 3, COMM_SET_SERVO_POS)
{
  uint16_t v = static_cast<uint16_t>(servo_pos * 1000.0);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

}  // namespace vesc_driver
