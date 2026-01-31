/**
 * @file xesc_driver.hpp
 * @brief Generic XESC Driver - Factory for different ESC types
 *
 * Ported from open_mower_ros (ROS1) to ROS2
 * Original author: Clemens Elflein
 */

#ifndef XESC_DRIVER__XESC_DRIVER_HPP_
#define XESC_DRIVER__XESC_DRIVER_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "xesc_interface/xesc_interface.hpp"

namespace xesc_driver
{

/**
 * @class XescDriver
 * @brief Factory class that creates the appropriate ESC driver based on configuration
 *
 * This class implements the XescInterface and acts as a factory/wrapper that
 * selects and instantiates the appropriate ESC driver based on the 'xesc_type'
 * parameter:
 *
 * - "xesc_mini" / "vesc" -> VescDriver (VESC motor controllers)
 * - "xesc_2040" -> Xesc2040Driver (RP2040-based ESC) [TODO]
 * - "xesc_yfr4" -> XescYFR4Driver (YardForce R4 ESC) [TODO]
 */
class XescDriver : public xesc_interface::XescInterface
{
public:
  /**
   * @brief Construct an XescDriver
   * @param node Pointer to the ROS2 node for parameter handling and logging
   *
   * The constructor reads the 'xesc_type' parameter and instantiates the
   * appropriate driver implementation.
   */
  explicit XescDriver(rclcpp::Node* node);

  /**
   * @brief Destructor
   */
  ~XescDriver() override;

  // Delete copy/move operations
  XescDriver(const XescDriver&) = delete;
  XescDriver& operator=(const XescDriver&) = delete;
  XescDriver(XescDriver&&) = delete;
  XescDriver& operator=(XescDriver&&) = delete;

  /**
   * @brief Get the current motor controller status (non-blocking)
   * @param[out] state The state message to populate
   */
  void getStatus(xesc_msgs::msg::XescStateStamped& state) override;

  /**
   * @brief Get the current motor controller status (blocking)
   * @param[out] state The state message to populate
   */
  void getStatusBlocking(xesc_msgs::msg::XescStateStamped& state) override;

  /**
   * @brief Set the motor duty cycle
   * @param duty_cycle Duty cycle value between -1.0 and 1.0
   */
  void setDutyCycle(float duty_cycle) override;

  /**
   * @brief Stop the motor
   */
  void stop() override;

private:
  rclcpp::Node* node_;
  std::unique_ptr<xesc_interface::XescInterface> driver_;
};

}  // namespace xesc_driver

#endif  // XESC_DRIVER__XESC_DRIVER_HPP_
