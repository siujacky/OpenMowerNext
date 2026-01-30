/**
 * @file xesc_interface.hpp
 * @brief XESC Motor Controller Interface
 * 
 * Ported from open_mower_ros (ROS1) to ROS2
 * Original author: Clemens Elflein
 */

#ifndef XESC_INTERFACE__XESC_INTERFACE_HPP_
#define XESC_INTERFACE__XESC_INTERFACE_HPP_

#include <xesc_msgs/msg/xesc_state_stamped.hpp>

namespace xesc_interface
{

/**
 * @class XescInterface
 * @brief Abstract interface for XESC motor controllers
 * 
 * This interface provides a common API for different XESC motor controller
 * implementations (VESC, xESC 2040, YFR4, etc.)
 */
class XescInterface
{
public:
  virtual ~XescInterface() = default;

  /**
   * @brief Get the current motor controller status (non-blocking)
   * @param[out] state The state message to populate
   */
  virtual void getStatus(xesc_msgs::msg::XescStateStamped & state) = 0;

  /**
   * @brief Get the current motor controller status (blocking)
   * @param[out] state The state message to populate
   * 
   * This method will block until a new state is available from the controller.
   */
  virtual void getStatusBlocking(xesc_msgs::msg::XescStateStamped & state) = 0;

  /**
   * @brief Set the motor duty cycle
   * @param duty_cycle Duty cycle value between -1.0 and 1.0
   */
  virtual void setDutyCycle(float duty_cycle) = 0;

  /**
   * @brief Stop the motor
   * 
   * This immediately stops the motor by setting duty cycle to 0.
   */
  virtual void stop() = 0;
};

}  // namespace xesc_interface

#endif  // XESC_INTERFACE__XESC_INTERFACE_HPP_
