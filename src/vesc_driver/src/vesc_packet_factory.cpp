/**
 * @file vesc_packet_factory.cpp
 * @brief VESC Packet Factory implementation
 * 
 * Ported from open_mower_ros (ROS1) to ROS2
 * 
 * Copyright (c) 2019, SoftBank Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Softbank Corp. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Originally developed by Michael T. Boulet at MIT under BSD 3-clause License.
 */

#include "vesc_driver/vesc_packet_factory.hpp"

namespace vesc_driver
{

namespace
{
/**
 * @brief Helper function to create failure result
 * @param num_bytes_needed[out] Number of bytes needed (if applicable)
 * @param what[out] Error message
 * @param message The error message string
 * @param bytes_needed Number of bytes needed (default 0)
 * @return Empty VescPacketPtr
 */
VescPacketPtr createFailed(
  int * num_bytes_needed,
  std::string * what,
  const std::string & message,
  int bytes_needed = 0)
{
  if (num_bytes_needed != nullptr) {
    *num_bytes_needed = bytes_needed;
  }
  if (what != nullptr) {
    *what = message;
  }
  return VescPacketPtr();
}
}  // namespace

VescPacketPtr VescPacketFactory::createPacket(
  const Buffer::const_iterator & begin,
  const Buffer::const_iterator & end,
  int * num_bytes_needed,
  std::string * what)
{
  // Initialize output variables
  if (num_bytes_needed != nullptr) {
    *num_bytes_needed = 0;
  }
  if (what != nullptr) {
    what->clear();
  }

  // Requires at least VESC_MIN_FRAME_SIZE bytes in buffer
  const int buffer_size = static_cast<int>(std::distance(begin, end));
  if (buffer_size < VescFrame::VESC_MIN_FRAME_SIZE) {
    return createFailed(
      num_bytes_needed, what,
      "Buffer does not contain a complete frame",
      VescFrame::VESC_MIN_FRAME_SIZE - buffer_size);
  }

  // Check whether buffer begins with a start-of-frame
  if (*begin != VescFrame::VESC_SOF_VAL_SMALL_FRAME &&
    *begin != VescFrame::VESC_SOF_VAL_LARGE_FRAME)
  {
    return createFailed(
      num_bytes_needed, what,
      "Buffer must begin with start-of-frame character");
  }

  // Get a view of the payload
  BufferRangeConst view_payload;
  if (*begin == VescFrame::VESC_SOF_VAL_SMALL_FRAME) {
    // Payload size field is one byte
    view_payload.first = begin + 2;
    view_payload.second = view_payload.first + *(begin + 1);
  } else {
    assert(*begin == VescFrame::VESC_SOF_VAL_LARGE_FRAME);
    // Payload size field is two bytes
    view_payload.first = begin + 3;
    view_payload.second = view_payload.first + (*(begin + 1) << 8) + *(begin + 2);
  }

  // Check the length
  if (boost::distance(view_payload) > VescFrame::VESC_MAX_PAYLOAD_SIZE) {
    return createFailed(num_bytes_needed, what, "Invalid payload length");
  }

  // Get iterators to CRC field, end-of-frame field, and a view of the whole frame
  auto iter_crc = view_payload.second;
  auto iter_eof = iter_crc + 2;
  BufferRangeConst view_frame(begin, iter_eof + 1);

  // Check whether enough data is loaded in the buffer to complete the frame
  const int frame_size = static_cast<int>(boost::distance(view_frame));
  if (buffer_size < frame_size) {
    return createFailed(
      num_bytes_needed, what,
      "Buffer does not contain a complete frame",
      frame_size - buffer_size);
  }

  // Check whether the end-of-frame character is valid
  if (*iter_eof != VescFrame::VESC_EOF_VAL) {
    return createFailed(num_bytes_needed, what, "Invalid end-of-frame character");
  }

  // Check whether the CRC is valid
  const uint16_t crc = (static_cast<uint16_t>(*iter_crc) << 8) + *(iter_crc + 1);
  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*view_payload.first), boost::distance(view_payload));
  if (crc != crc_calc.checksum()) {
    return createFailed(num_bytes_needed, what, "Invalid checksum");
  }

  // Construct the raw frame
  auto raw_frame = std::make_shared<VescFrame>(view_frame, view_payload);

  // Construct the corresponding subclass if the packet has a payload
  if (boost::distance(view_payload) > 0) {
    // Get constructor function from payload ID
    FactoryMap * p_map = getMap();
    auto search = p_map->find(*view_payload.first);

    if (search != p_map->end()) {
      return search->second(raw_frame);
    } else {
      // No subclass constructor for this packet
      return createFailed(num_bytes_needed, what, "Unknown payload type.");
    }
  } else {
    // No payload
    return createFailed(num_bytes_needed, what, "Frame does not have a payload");
  }
}

void VescPacketFactory::registerPacketType(int payload_id, CreateFn fn)
{
  FactoryMap * p_map = getMap();
  assert(p_map->count(payload_id) == 0);
  (*p_map)[payload_id] = fn;
}

VescPacketFactory::FactoryMap * VescPacketFactory::getMap()
{
  // Meyer's singleton - construct on first use, thread-safe in C++11+
  static FactoryMap map;
  return &map;
}

// Register packet types with the factory
REGISTER_PACKET_TYPE(COMM_FW_VERSION, VescPacketFWVersion)
REGISTER_PACKET_TYPE(COMM_GET_VALUES, VescPacketValues)

}  // namespace vesc_driver
