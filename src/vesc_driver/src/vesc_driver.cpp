/**
 * @file vesc_driver.cpp
 * @brief VESC Driver implementation
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

#include "vesc_driver/vesc_driver.hpp"

namespace vesc_driver
{

VescDriver::VescDriver(rclcpp::Node* node)
  : node_(node)
  , vesc_(std::bind(&VescDriver::vescErrorCallback, this, std::placeholders::_1))
  , duty_cycle_limit_(node, "duty_cycle", -1.0, 1.0)
  , vesc_status_{}
  , pole_pairs_(4)
{
  // Declare and get parameters
  node_->declare_parameter<std::string>("serial_port", "");
  node_->declare_parameter<int>("motor_pole_pairs", 4);

  // Get VESC serial port address
  std::string port;
  if (!node_->get_parameter("serial_port", port) || port.empty())
  {
    RCLCPP_FATAL(node_->get_logger(), "VESC communication port parameter required.");
    throw std::runtime_error("VESC communication port parameter required.");
  }

  // Get motor pole pairs, just needed for eRPM to RPM calculation
  node_->get_parameter("motor_pole_pairs", pole_pairs_);
  if (pole_pairs_ == 0)
  {
    RCLCPP_WARN(node_->get_logger(), "VESC config has wrong motor_pole_pairs value %d. Forced to 1.", pole_pairs_);
    pole_pairs_ = 1;
  }

  RCLCPP_INFO(node_->get_logger(), "Starting VESC driver on port: %s with %d pole pairs", port.c_str(), pole_pairs_);

  vesc_.start(port);
}

void VescDriver::vescErrorCallback(const std::string& error)
{
  RCLCPP_ERROR(node_->get_logger(), "%s", error.c_str());
}

void VescDriver::stop()
{
  vesc_.stop();
}

void VescDriver::getStatus(xesc_msgs::msg::XescStateStamped& state_msg)
{
  vesc_.getStatus(&vesc_status_);

  state_msg.header.stamp = node_->now();
  state_msg.state.connection_state = static_cast<uint8_t>(vesc_status_.connection_state);
  state_msg.state.fw_major = vesc_status_.fw_version_major;
  state_msg.state.fw_minor = vesc_status_.fw_version_minor;
  state_msg.state.voltage_input = vesc_status_.voltage_input;
  state_msg.state.temperature_pcb = vesc_status_.temperature_pcb;
  state_msg.state.temperature_motor = vesc_status_.temperature_motor;
  state_msg.state.current_input = vesc_status_.current_input;
  state_msg.state.duty_cycle = vesc_status_.duty_cycle;
  state_msg.state.tacho = vesc_status_.tacho;
  state_msg.state.fault_code = vesc_status_.fault_code;
  state_msg.state.tacho_absolute = vesc_status_.tacho_absolute;
  state_msg.state.direction = vesc_status_.direction;
  state_msg.state.rpm = vesc_status_.speed_erpm / pole_pairs_;
}

void VescDriver::getStatusBlocking(xesc_msgs::msg::XescStateStamped& state_msg)
{
  vesc_.waitForStatus(&vesc_status_);

  state_msg.header.stamp = node_->now();
  state_msg.state.connection_state = static_cast<uint8_t>(vesc_status_.connection_state);
  state_msg.state.fw_major = vesc_status_.fw_version_major;
  state_msg.state.fw_minor = vesc_status_.fw_version_minor;
  state_msg.state.voltage_input = vesc_status_.voltage_input;
  state_msg.state.temperature_pcb = vesc_status_.temperature_pcb;
  state_msg.state.temperature_motor = vesc_status_.temperature_motor;
  state_msg.state.current_input = vesc_status_.current_input;
  state_msg.state.duty_cycle = vesc_status_.duty_cycle;
  state_msg.state.tacho = vesc_status_.tacho;
  state_msg.state.fault_code = vesc_status_.fault_code;
  state_msg.state.tacho_absolute = vesc_status_.tacho_absolute;
  state_msg.state.direction = vesc_status_.direction;
  state_msg.state.rpm = vesc_status_.speed_erpm / pole_pairs_;
}

void VescDriver::setDutyCycle(float duty_cycle)
{
  vesc_.setDutyCycle(duty_cycle_limit_.clip(static_cast<double>(duty_cycle)));
}

VescDriver::CommandLimit::CommandLimit(rclcpp::Node* node, const std::string& str,
                                       const std::optional<double>& min_lower, const std::optional<double>& max_upper)
  : name(str), node_(node)
{
  // Declare parameters
  node_->declare_parameter<double>(name + "_min", min_lower.value_or(-1e10));
  node_->declare_parameter<double>(name + "_max", max_upper.value_or(1e10));

  // Check if user's minimum value is outside of the range min_lower to max_upper
  double param_min;
  if (node_->get_parameter(name + "_min", param_min))
  {
    if (min_lower && param_min < *min_lower)
    {
      lower = *min_lower;
      RCLCPP_WARN(node_->get_logger(), "Parameter %s_min (%f) is less than the feasible minimum (%f).", name.c_str(),
                  param_min, *min_lower);
    }
    else if (max_upper && param_min > *max_upper)
    {
      lower = *max_upper;
      RCLCPP_WARN(node_->get_logger(), "Parameter %s_min (%f) is greater than the feasible maximum (%f).", name.c_str(),
                  param_min, *max_upper);
    }
    else
    {
      lower = param_min;
    }
  }
  else if (min_lower)
  {
    lower = *min_lower;
  }

  // Check if the user's maximum value is outside of the range min_lower to max_upper
  double param_max;
  if (node_->get_parameter(name + "_max", param_max))
  {
    if (min_lower && param_max < *min_lower)
    {
      upper = *min_lower;
      RCLCPP_WARN(node_->get_logger(), "Parameter %s_max (%f) is less than the feasible minimum (%f).", name.c_str(),
                  param_max, *min_lower);
    }
    else if (max_upper && param_max > *max_upper)
    {
      upper = *max_upper;
      RCLCPP_WARN(node_->get_logger(), "Parameter %s_max (%f) is greater than the feasible maximum (%f).", name.c_str(),
                  param_max, *max_upper);
    }
    else
    {
      upper = param_max;
    }
  }
  else if (max_upper)
  {
    upper = *max_upper;
  }

  // Check for min > max
  if (upper && lower && *lower > *upper)
  {
    RCLCPP_WARN(node_->get_logger(), "Parameter %s_max (%f) is less than parameter %s_min (%f).", name.c_str(), *upper,
                name.c_str(), *lower);
    double temp = *lower;
    lower = *upper;
    upper = temp;
  }

  std::ostringstream oss;
  oss << "  " << name << " limit: ";
  if (lower)
  {
    oss << *lower << " ";
  }
  else
  {
    oss << "(none) ";
  }
  if (upper)
  {
    oss << *upper;
  }
  else
  {
    oss << "(none)";
  }
  RCLCPP_DEBUG(node_->get_logger(), "%s", oss.str().c_str());
}

double VescDriver::CommandLimit::clip(double value)
{
  if (lower && value < *lower)
  {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                         "%s command value (%f) below minimum limit (%f), clipping.", name.c_str(), value, *lower);
    return *lower;
  }
  if (upper && value > *upper)
  {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                         "%s command value (%f) above maximum limit (%f), clipping.", name.c_str(), value, *upper);
    return *upper;
  }
  return value;
}

}  // namespace vesc_driver
