/**
 * @file xesc_2040_driver.hpp
 * @brief ROS2 driver wrapper for XESC 2040 motor controller
 *
 * Ported from open_mower_ros (ROS1) to ROS2
 * Original author: Clemens Elflein
 */

#ifndef XESC_2040_DRIVER__XESC_2040_DRIVER_HPP_
#define XESC_2040_DRIVER__XESC_2040_DRIVER_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <xesc_interface/xesc_interface.hpp>

#include "xesc_2040_driver/xesc_2040_interface.hpp"

namespace xesc_2040_driver
{

/**
 * @brief ROS2 driver class for XESC 2040 motor controller
 *
 * This class implements the XescInterface for the XESC 2040 RP2040-based
 * motor controller, providing status reporting and duty cycle control.
 */
class Xesc2040Driver : public xesc_interface::XescInterface
{
public:
  /**
   * @brief Constructor
   * @param node Shared pointer to the ROS2 node for logging and parameters
   */
  explicit Xesc2040Driver(rclcpp::Node::SharedPtr node);

  /**
   * @brief Destructor
   */
  ~Xesc2040Driver() override = default;

  // Delete copy operations
  Xesc2040Driver(const Xesc2040Driver&) = delete;
  Xesc2040Driver& operator=(const Xesc2040Driver&) = delete;

  /**
   * @brief Get current status (non-blocking)
   * @param[out] state State message to populate
   */
  void getStatus(xesc_msgs::msg::XescStateStamped& state) override;

  /**
   * @brief Get current status (blocking until new data available)
   * @param[out] state State message to populate
   */
  void getStatusBlocking(xesc_msgs::msg::XescStateStamped& state) override;

  /**
   * @brief Set motor duty cycle
   * @param duty_cycle Duty cycle value (-1.0 to 1.0)
   */
  void setDutyCycle(float duty_cycle) override;

  /**
   * @brief Stop the motor and interface
   */
  void stop() override;

private:
  /**
   * @brief Error callback for interface errors
   * @param error_msg Error message string
   */
  void errorCallback(const std::string& error_msg);

  rclcpp::Node::SharedPtr node_;
  Xesc2040StatusStruct status_{};
  std::unique_ptr<Xesc2040Interface> xesc_interface_;
};

}  // namespace xesc_2040_driver

#endif  // XESC_2040_DRIVER__XESC_2040_DRIVER_HPP_
