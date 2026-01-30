/**
 * @file vesc_driver.hpp
 * @brief VESC Driver - ROS2 wrapper implementing XescInterface
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

#ifndef VESC_DRIVER__VESC_DRIVER_HPP_
#define VESC_DRIVER__VESC_DRIVER_HPP_

#include <cassert>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "vesc_driver/vesc_interface.hpp"
#include "vesc_driver/vesc_packet.hpp"
#include "xesc_interface/xesc_interface.hpp"

namespace vesc_driver
{

/**
 * @class VescDriver
 * @brief VESC motor controller driver implementing the XescInterface
 * 
 * This class provides a ROS2-compatible driver for VESC motor controllers,
 * implementing the common XescInterface for use with the OpenMower system.
 */
class VescDriver : public xesc_interface::XescInterface
{
public:
  /**
   * @brief Construct a VescDriver
   * @param node Pointer to the ROS2 node for parameter handling and logging
   */
  explicit VescDriver(rclcpp::Node * node);

  /**
   * @brief Destructor
   */
  ~VescDriver() override = default;

  // Delete copy/move operations
  VescDriver(const VescDriver &) = delete;
  VescDriver & operator=(const VescDriver &) = delete;
  VescDriver(VescDriver &&) = delete;
  VescDriver & operator=(VescDriver &&) = delete;

  /**
   * @brief Get the current motor controller status (non-blocking)
   * @param[out] state The state message to populate
   */
  void getStatus(xesc_msgs::msg::XescStateStamped & state) override;

  /**
   * @brief Get the current motor controller status (blocking)
   * @param[out] state The state message to populate
   */
  void getStatusBlocking(xesc_msgs::msg::XescStateStamped & state) override;

  /**
   * @brief Set the motor duty cycle
   * @param duty_cycle Duty cycle value between -1.0 and 1.0
   */
  void setDutyCycle(float duty_cycle) override;

  /**
   * @brief Stop the motor and close the interface
   */
  void stop() override;

private:
  /**
   * @brief Callback for VESC error messages
   * @param error The error message
   */
  void vescErrorCallback(const std::string & error);

  /**
   * @brief Helper class for command limits
   */
  struct CommandLimit
  {
    /**
     * @brief Construct a CommandLimit
     * @param node ROS2 node for parameter handling
     * @param str Parameter name base
     * @param min_lower Minimum allowed lower bound
     * @param max_upper Maximum allowed upper bound
     */
    CommandLimit(
      rclcpp::Node * node,
      const std::string & str,
      const std::optional<double> & min_lower = std::nullopt,
      const std::optional<double> & max_upper = std::nullopt);

    /**
     * @brief Clip a value to the configured limits
     * @param value The value to clip
     * @return The clipped value
     */
    double clip(double value);

    std::string name;
    std::optional<double> lower;
    std::optional<double> upper;
    rclcpp::Node * node_;
  };

  // ROS2 node reference for logging
  rclcpp::Node * node_;

  // Interface to the VESC
  VescInterface vesc_;

  // Command limits
  CommandLimit duty_cycle_limit_;

  // Current status
  VescStatusStruct vesc_status_;

  // Motor configuration
  int pole_pairs_;
};

}  // namespace vesc_driver

#endif  // VESC_DRIVER__VESC_DRIVER_HPP_
