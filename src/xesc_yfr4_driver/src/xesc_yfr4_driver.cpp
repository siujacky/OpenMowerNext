/**
 * @file xesc_yfr4_driver.cpp
 * @brief ROS2 driver implementation for XESC YardForce R4 motor controller
 *
 * Ported from open_mower_ros (ROS1) to ROS2
 * Original author: Clemens Elflein
 *
 * Changes from ROS1:
 * - ros::NodeHandle replaced with rclcpp::Node
 * - ROS_* macros replaced with RCLCPP_* macros
 * - ros::Time::now() replaced with node_->now()
 * - Parameter handling via declare_parameter/get_parameter
 */

#include "xesc_yfr4_driver/xesc_yfr4_driver.hpp"

#include <functional>

namespace xesc_yfr4_driver
{

void XescYFR4Driver::errorCallback(const std::string& error_msg)
{
  RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str());
}

XescYFR4Driver::XescYFR4Driver(rclcpp::Node::SharedPtr node) : node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "Starting xesc YardForce R4 adapter driver");

  xesc_interface_ =
      std::make_unique<XescYFR4Interface>(std::bind(&XescYFR4Driver::errorCallback, this, std::placeholders::_1));

  // Declare and get parameters
  std::string serial_port = node_->declare_parameter<std::string>("serial_port", "");
  if (serial_port.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "You need to provide parameter serial_port.");
    throw std::runtime_error("Missing required parameter: serial_port");
  }

  double motor_current_limit = node_->declare_parameter<double>("motor_current_limit", -1.0);
  if (motor_current_limit < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "You need to provide parameter motor_current_limit");
    throw std::runtime_error("Missing required parameter: motor_current_limit");
  }

  double min_pcb_temp = node_->declare_parameter<double>("min_pcb_temp", 0.0);
  double max_pcb_temp = node_->declare_parameter<double>("max_pcb_temp", 100.0);

  xesc_interface_->updateSettings(static_cast<float>(motor_current_limit), static_cast<float>(min_pcb_temp),
                                  static_cast<float>(max_pcb_temp));

  xesc_interface_->start(serial_port);
}

void XescYFR4Driver::getStatus(xesc_msgs::msg::XescStateStamped& state_msg)
{
  if (!xesc_interface_)
  {
    return;
  }
  xesc_interface_->getStatus(&status_);

  state_msg.header.stamp = node_->now();
  state_msg.state.connection_state = static_cast<uint8_t>(status_.connection_state);
  state_msg.state.fw_major = status_.fw_version_major;
  state_msg.state.fw_minor = status_.fw_version_minor;
  state_msg.state.temperature_pcb = status_.temperature_pcb;
  state_msg.state.current_input = status_.current_input;
  state_msg.state.duty_cycle = status_.duty_cycle;
  state_msg.state.direction = status_.direction;
  state_msg.state.tacho = status_.tacho;
  state_msg.state.tacho_absolute = status_.tacho_absolute;
  state_msg.state.rpm = status_.rpm;
  state_msg.state.fault_code = status_.fault_code;
}

void XescYFR4Driver::getStatusBlocking(xesc_msgs::msg::XescStateStamped& state_msg)
{
  if (!xesc_interface_)
  {
    return;
  }
  xesc_interface_->waitForStatus(&status_);

  state_msg.header.stamp = node_->now();
  state_msg.state.connection_state = static_cast<uint8_t>(status_.connection_state);
  state_msg.state.fw_major = status_.fw_version_major;
  state_msg.state.fw_minor = status_.fw_version_minor;
  state_msg.state.temperature_pcb = status_.temperature_pcb;
  state_msg.state.current_input = status_.current_input;
  state_msg.state.duty_cycle = status_.duty_cycle;
  state_msg.state.direction = status_.direction;
  state_msg.state.tacho = status_.tacho;
  state_msg.state.tacho_absolute = status_.tacho_absolute;
  state_msg.state.rpm = status_.rpm;
  state_msg.state.fault_code = status_.fault_code;
}

void XescYFR4Driver::stop()
{
  RCLCPP_INFO(node_->get_logger(), "Stopping xesc YardForce R4 adapter driver");
  if (xesc_interface_)
  {
    xesc_interface_->stop();
    xesc_interface_.reset();
  }
}

void XescYFR4Driver::setDutyCycle(float duty_cycle)
{
  if (xesc_interface_)
  {
    xesc_interface_->setDutyCycle(duty_cycle);
  }
}

}  // namespace xesc_yfr4_driver
