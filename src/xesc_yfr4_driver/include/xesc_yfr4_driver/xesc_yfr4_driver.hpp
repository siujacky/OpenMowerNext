/**
 * @file xesc_yfr4_driver.hpp
 * @brief ROS2 driver wrapper for XESC YardForce R4 motor controller
 * 
 * Ported from open_mower_ros (ROS1) to ROS2
 * Original author: Clemens Elflein
 */

#ifndef XESC_YFR4_DRIVER__XESC_YFR4_DRIVER_HPP_
#define XESC_YFR4_DRIVER__XESC_YFR4_DRIVER_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <xesc_interface/xesc_interface.hpp>

#include "xesc_yfr4_driver/xesc_yfr4_interface.hpp"

namespace xesc_yfr4_driver
{

/**
 * @brief ROS2 driver class for XESC YardForce R4 motor controller
 * 
 * This class implements the XescInterface for the XESC YFR4 adapter,
 * providing status reporting and duty cycle control.
 */
class XescYFR4Driver : public xesc_interface::XescInterface
{
public:
  /**
   * @brief Constructor
   * @param node Shared pointer to the ROS2 node for logging and parameters
   */
  explicit XescYFR4Driver(rclcpp::Node::SharedPtr node);

  /**
   * @brief Destructor
   */
  ~XescYFR4Driver() override = default;

  // Delete copy operations
  XescYFR4Driver(const XescYFR4Driver &) = delete;
  XescYFR4Driver & operator=(const XescYFR4Driver &) = delete;

  /**
   * @brief Get current status (non-blocking)
   * @param[out] state State message to populate
   */
  void getStatus(xesc_msgs::msg::XescStateStamped & state) override;

  /**
   * @brief Get current status (blocking until new data available)
   * @param[out] state State message to populate
   */
  void getStatusBlocking(xesc_msgs::msg::XescStateStamped & state) override;

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
  void errorCallback(const std::string & error_msg);

  rclcpp::Node::SharedPtr node_;
  XescYFR4StatusStruct status_{};
  std::unique_ptr<XescYFR4Interface> xesc_interface_;
};

}  // namespace xesc_yfr4_driver

#endif  // XESC_YFR4_DRIVER__XESC_YFR4_DRIVER_HPP_
