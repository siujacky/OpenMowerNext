/**
 * @file vesc_packet_factory.hpp
 * @brief VESC Packet Factory - Creates packets from raw serial data
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

#ifndef VESC_DRIVER__VESC_PACKET_FACTORY_HPP_
#define VESC_DRIVER__VESC_PACKET_FACTORY_HPP_

#include <cassert>
#include <cstdint>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <boost/range/begin.hpp>
#include <boost/range/distance.hpp>
#include <boost/range/end.hpp>

#include "vesc_driver/data_map.hpp"
#include "vesc_driver/vesc_packet.hpp"

namespace vesc_driver
{

/**
 * @class VescPacketFactory
 * @brief Creates VESC packets from raw serial data
 * 
 * This is a static factory class that parses raw bytes from the serial
 * interface and constructs the appropriate VescPacket subclass.
 */
class VescPacketFactory
{
public:
  // Delete copy/move operations (singleton pattern)
  VescPacketFactory(const VescPacketFactory &) = delete;
  VescPacketFactory & operator=(const VescPacketFactory &) = delete;
  VescPacketFactory(VescPacketFactory &&) = delete;
  VescPacketFactory & operator=(VescPacketFactory &&) = delete;

  /**
   * @brief Creates a VescPacket from a buffer
   * 
   * @param begin Iterator to buffer at the start-of-frame character
   * @param end Iterator to the buffer past-the-end element
   * @param num_bytes_needed[out] Number of bytes needed to complete the frame (if incomplete)
   * @param what[out] Message string giving a reason why the packet was not found
   * @return Pointer to a valid VescPacket if successful; otherwise nullptr
   */
  static VescPacketPtr createPacket(
    const Buffer::const_iterator & begin,
    const Buffer::const_iterator & end,
    int * num_bytes_needed,
    std::string * what);

  /**
   * @brief Function type for creating packet instances
   */
  using CreateFn = std::function<VescPacketPtr(std::shared_ptr<VescFrame>)>;

  /**
   * @brief Register a packet type with the factory
   * @param payload_id The payload ID for this packet type
   * @param fn Function to create instances of this packet type
   */
  static void registerPacketType(int payload_id, CreateFn fn);

private:
  using FactoryMap = std::map<int, CreateFn>;

  /**
   * @brief Get the singleton factory map (construct on first use)
   * @return Pointer to the factory map
   */
  static FactoryMap * getMap();
};

/**
 * @def REGISTER_PACKET_TYPE
 * @brief Macro to register a packet type with the factory
 * 
 * This macro creates a static factory class that registers itself
 * with VescPacketFactory at static initialization time.
 */
#define REGISTER_PACKET_TYPE(id, klass) \
  class klass##Factory \
  { \
public: \
    klass##Factory() \
    { \
      VescPacketFactory::registerPacketType((id), &klass##Factory::create); \
    } \
    static VescPacketPtr create(std::shared_ptr<VescFrame> frame) \
    { \
      return std::make_shared<klass>(frame); \
    } \
  }; \
  static klass##Factory global_##klass##Factory;

}  // namespace vesc_driver

#endif  // VESC_DRIVER__VESC_PACKET_FACTORY_HPP_
