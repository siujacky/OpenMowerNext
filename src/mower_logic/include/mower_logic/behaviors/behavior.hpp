// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: GPL-3.0
//
// ROS2 port of Behavior base class

#ifndef MOWER_LOGIC__BEHAVIORS__BEHAVIOR_HPP_
#define MOWER_LOGIC__BEHAVIORS__BEHAVIOR_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mower_msgs/msg/high_level_status.hpp"

namespace mower_logic
{

enum class AutoMode
{
  MANUAL = 0,
  SEMIAUTO = 1,
  AUTO = 2
};

enum PauseType
{
  PAUSE_MANUAL = 0b1,
  PAUSE_EMERGENCY = 0b10
};

struct SharedState
{
  bool active_semiautomatic_task = false;
};

// Forward declaration
class MowerLogicNode;

/**
 * Configuration for mower logic - replaces dynamic_reconfigure
 */
struct MowerLogicConfig
{
  bool enable_mower = false;
  bool manual_pause_mowing = false;
  double max_position_accuracy = 0.1;
  double gps_timeout = 5.0;
  double motor_hot_temperature = 60.0;
  bool ignore_gps_errors = false;
  int rain_mode = 0;
  int rain_check_seconds = 30;
  int rain_delay_minutes = 10;
  int docking_approach_distance = 1;
  int docking_retry_count = 3;
  double undock_distance = 2.0;
  int mow_area = 0;
  int mow_direction = 0;
};

/**
 * Base class for all behaviors
 */
class Behavior
{
public:
  Behavior() = default;
  virtual ~Behavior() = default;

  // Non-copyable
  Behavior(const Behavior&) = delete;
  Behavior& operator=(const Behavior&) = delete;

  virtual std::string state_name() = 0;
  virtual std::string sub_state_name()
  {
    return "";
  }

  bool hasGoodGPS() const
  {
    return is_gps_good_.load();
  }
  void setGoodGPS(bool is_good)
  {
    is_gps_good_.store(is_good);
  }

  void requestContinue(PauseType reason = PAUSE_MANUAL)
  {
    requested_pause_flag_ &= ~reason;
  }

  void requestPause(PauseType reason = PAUSE_MANUAL)
  {
    requested_pause_flag_ |= reason;
  }

  void start(const MowerLogicConfig& config, std::shared_ptr<SharedState> state, rclcpp::Node* node)
  {
    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "--------------------------------------");
    RCLCPP_INFO(node->get_logger(), "- Entered state: %s", state_name().c_str());
    RCLCPP_INFO(node->get_logger(), "--------------------------------------");

    aborted_.store(false);
    paused_.store(false);
    requested_pause_flag_.store(0);
    config_ = config;
    shared_state_ = std::move(state);
    node_ = node;
    start_time_ = node_->now();
    is_gps_good_.store(false);
    sub_state_.store(0);
    enter();
  }

  /**
   * Execute the behavior. This call should block until the behavior is executed fully.
   * @returns the pointer to the next behavior (can return itself).
   */
  virtual Behavior* execute() = 0;

  virtual void exit() = 0;
  virtual void reset() = 0;

  void abort()
  {
    if (!aborted_.load())
    {
      RCLCPP_INFO(node_->get_logger(), "- Behavior: abort() called");
    }
    aborted_.store(true);
  }

  virtual bool needs_gps() = 0;
  virtual bool mower_enabled() = 0;
  virtual bool redirect_joystick() = 0;

  virtual void command_home() = 0;
  virtual void command_start() = 0;
  virtual void command_s1() = 0;
  virtual void command_s2() = 0;

  virtual uint8_t get_sub_state() = 0;
  virtual uint8_t get_state() = 0;

  virtual void handle_action(const std::string& action) = 0;

protected:
  virtual void enter() = 0;

  double time_in_state() const
  {
    return (node_->now() - start_time_).seconds();
  }

  std::atomic<bool> aborted_{ false };
  std::atomic<bool> paused_{ false };
  std::atomic<uint8_t> requested_pause_flag_{ 0 };
  std::atomic<bool> is_gps_good_{ false };
  std::atomic<uint8_t> sub_state_{ 0 };

  MowerLogicConfig config_;
  std::shared_ptr<SharedState> shared_state_;
  rclcpp::Node* node_ = nullptr;
  rclcpp::Time start_time_;
};

}  // namespace mower_logic

#endif  // MOWER_LOGIC__BEHAVIORS__BEHAVIOR_HPP_
