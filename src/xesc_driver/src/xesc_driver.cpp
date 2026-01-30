/**
 * @file xesc_driver.cpp
 * @brief Generic XESC Driver implementation
 * 
 * Ported from open_mower_ros (ROS1) to ROS2
 * Original author: Clemens Elflein
 */

#include "xesc_driver/xesc_driver.hpp"

#include "vesc_driver/vesc_driver.hpp"
// TODO: Add these when ported
// #include "xesc_2040_driver/xesc_2040_driver.hpp"
// #include "xesc_yfr4_driver/xesc_yfr4_driver.hpp"

namespace xesc_driver
{

XescDriver::XescDriver(rclcpp::Node * node)
: node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "Starting xesc driver");

  // Declare and get xesc_type parameter
  node_->declare_parameter<std::string>("xesc_type", "");

  std::string xesc_type;
  if (!node_->get_parameter("xesc_type", xesc_type) || xesc_type.empty()) {
    RCLCPP_FATAL(node_->get_logger(), "Error: xesc_type not set.");
    throw std::runtime_error("xesc_type parameter is required");
  }

  RCLCPP_INFO(node_->get_logger(), "XESC type: %s", xesc_type.c_str());

  if (xesc_type == "xesc_mini" || xesc_type == "vesc") {
    driver_ = std::make_unique<vesc_driver::VescDriver>(node_);
  }
  // TODO: Uncomment when xesc_2040_driver is ported
  // else if (xesc_type == "xesc_2040") {
  //   driver_ = std::make_unique<xesc_2040_driver::Xesc2040Driver>(node_);
  // }
  // TODO: Uncomment when xesc_yfr4_driver is ported
  // else if (xesc_type == "xesc_yfr4") {
  //   driver_ = std::make_unique<xesc_yfr4_driver::XescYFR4Driver>(node_);
  // }
  else {
    RCLCPP_FATAL(
      node_->get_logger(),
      "Error: xesc_type invalid. Type was: %s. "
      "Supported types: xesc_mini, vesc",
      xesc_type.c_str());
    throw std::runtime_error("Invalid xesc_type: " + xesc_type);
  }
}

XescDriver::~XescDriver()
{
  if (driver_) {
    driver_->stop();
  }
}

void XescDriver::getStatus(xesc_msgs::msg::XescStateStamped & state_msg)
{
  if (driver_) {
    driver_->getStatus(state_msg);
  }
}

void XescDriver::getStatusBlocking(xesc_msgs::msg::XescStateStamped & state_msg)
{
  if (driver_) {
    driver_->getStatusBlocking(state_msg);
  }
}

void XescDriver::stop()
{
  if (driver_) {
    driver_->stop();
  }
}

void XescDriver::setDutyCycle(float duty_cycle)
{
  if (driver_) {
    driver_->setDutyCycle(duty_cycle);
  }
}

}  // namespace xesc_driver
