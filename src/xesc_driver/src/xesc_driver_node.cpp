/**
 * @file xesc_driver_node.cpp
 * @brief XESC Driver ROS2 Node
 *
 * Ported from open_mower_ros (ROS1) to ROS2
 * Original author: Clemens Elflein
 *
 * This node provides:
 * - Subscription to duty_cycle commands (std_msgs/Float32)
 * - Publisher for motor state telemetry (xesc_msgs/XescStateStamped)
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <xesc_msgs/msg/xesc_state_stamped.hpp>

#include "xesc_driver/xesc_driver.hpp"

namespace xesc_driver
{

/**
 * @class XescDriverNode
 * @brief ROS2 Node wrapper for XescDriver
 */
class XescDriverNode : public rclcpp::Node
{
public:
  XescDriverNode() : Node("xesc_driver_node")
  {
    // Create the driver (reads parameters from this node)
    driver_ = std::make_unique<XescDriver>(this);

    // Create publisher for motor state telemetry
    state_pub_ = this->create_publisher<xesc_msgs::msg::XescStateStamped>("sensors/core", 10);

    // Create subscription for duty cycle commands
    duty_cycle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "duty_cycle", rclcpp::QoS(10).best_effort(),
        std::bind(&XescDriverNode::dutyCycleCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "XESC driver node started");
  }

  ~XescDriverNode() override
  {
    RCLCPP_INFO(this->get_logger(), "Stopping XESC driver node");
    if (driver_)
    {
      driver_->stop();
    }
  }

  /**
   * @brief Main loop - blocking status reads and publishes
   *
   * This should be called from a separate thread as it blocks waiting for
   * status updates from the motor controller.
   */
  void run()
  {
    xesc_msgs::msg::XescStateStamped state_msg;

    while (rclcpp::ok())
    {
      driver_->getStatusBlocking(state_msg);
      state_pub_->publish(state_msg);
    }
  }

private:
  void dutyCycleCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (driver_)
    {
      driver_->setDutyCycle(msg->data);
    }
  }

  std::unique_ptr<XescDriver> driver_;
  rclcpp::Publisher<xesc_msgs::msg::XescStateStamped>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr duty_cycle_sub_;
};

}  // namespace xesc_driver

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<xesc_driver::XescDriverNode>();

  // Create a multi-threaded executor to handle callbacks
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Start a thread for the executor (handles duty_cycle callbacks)
  std::thread executor_thread([&executor]() { executor.spin(); });

  // Run the blocking status read loop in main thread
  node->run();

  // Cleanup
  rclcpp::shutdown();
  executor_thread.join();

  return 0;
}
