/**
 * @file xesc_2040_driver.cpp
 * @brief ROS2 driver implementation for XESC 2040 motor controller
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

#include "xesc_2040_driver/xesc_2040_driver.hpp"

#include <functional>

namespace xesc_2040_driver
{

void Xesc2040Driver::errorCallback(const std::string& error_msg)
{
  RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str());
}

Xesc2040Driver::Xesc2040Driver(rclcpp::Node::SharedPtr node) : node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "Starting xesc 2040 driver");

  xesc_interface_ =
      std::make_unique<Xesc2040Interface>(std::bind(&Xesc2040Driver::errorCallback, this, std::placeholders::_1));

  // Declare and get parameters
  std::string serial_port = node_->declare_parameter<std::string>("serial_port", "");
  if (serial_port.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "You need to provide parameter serial_port.");
    throw std::runtime_error("Missing required parameter: serial_port");
  }

  uint8_t hall_table[8];
  for (int i = 0; i < 8; i++)
  {
    std::string param_name = "hall_table_" + std::to_string(i);
    int64_t tmp = node_->declare_parameter<int64_t>(param_name, -1);
    if (tmp < 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "You need to provide parameter %s", param_name.c_str());
      throw std::runtime_error("Missing required parameter: " + param_name);
    }
    hall_table[i] = static_cast<uint8_t>(tmp);
  }

  double motor_current_limit = node_->declare_parameter<double>("motor_current_limit", -1.0);
  if (motor_current_limit < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "You need to provide parameter motor_current_limit");
    throw std::runtime_error("Missing required parameter: motor_current_limit");
  }

  double acceleration = node_->declare_parameter<double>("acceleration", -1.0);
  if (acceleration < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "You need to provide parameter acceleration");
    throw std::runtime_error("Missing required parameter: acceleration");
  }

  bool has_motor_temp = node_->declare_parameter<bool>("has_motor_temp", false);

  double min_motor_temp = node_->declare_parameter<double>("min_motor_temp", 0.0);
  double max_motor_temp = node_->declare_parameter<double>("max_motor_temp", 100.0);
  double min_pcb_temp = node_->declare_parameter<double>("min_pcb_temp", 0.0);
  double max_pcb_temp = node_->declare_parameter<double>("max_pcb_temp", 100.0);

  xesc_interface_->updateSettings(hall_table, static_cast<float>(motor_current_limit), static_cast<float>(acceleration),
                                  has_motor_temp, static_cast<float>(min_motor_temp),
                                  static_cast<float>(max_motor_temp), static_cast<float>(min_pcb_temp),
                                  static_cast<float>(max_pcb_temp));

  xesc_interface_->start(serial_port);
}

void Xesc2040Driver::getStatus(xesc_msgs::msg::XescStateStamped& state_msg)
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
  state_msg.state.voltage_input = status_.voltage_input;
  state_msg.state.temperature_pcb = status_.temperature_pcb;
  state_msg.state.temperature_motor = status_.temperature_motor;
  state_msg.state.current_input = status_.current_input;
  state_msg.state.duty_cycle = status_.duty_cycle;
  state_msg.state.tacho = status_.tacho;
  state_msg.state.tacho_absolute = status_.tacho_absolute;
  state_msg.state.direction = status_.direction;
  state_msg.state.fault_code = status_.fault_code;
}

void Xesc2040Driver::getStatusBlocking(xesc_msgs::msg::XescStateStamped& state_msg)
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
  state_msg.state.voltage_input = status_.voltage_input;
  state_msg.state.temperature_pcb = status_.temperature_pcb;
  state_msg.state.temperature_motor = status_.temperature_motor;
  state_msg.state.current_input = status_.current_input;
  state_msg.state.duty_cycle = status_.duty_cycle;
  state_msg.state.tacho = status_.tacho;
  state_msg.state.tacho_absolute = status_.tacho_absolute;
  state_msg.state.direction = status_.direction;
  state_msg.state.fault_code = status_.fault_code;
}

void Xesc2040Driver::stop()
{
  RCLCPP_INFO(node_->get_logger(), "Stopping XESC2040 driver");
  if (xesc_interface_)
  {
    xesc_interface_->stop();
    xesc_interface_.reset();
  }
}

void Xesc2040Driver::setDutyCycle(float duty_cycle)
{
  if (xesc_interface_)
  {
    xesc_interface_->setDutyCycle(duty_cycle);
  }
}

}  // namespace xesc_2040_driver
