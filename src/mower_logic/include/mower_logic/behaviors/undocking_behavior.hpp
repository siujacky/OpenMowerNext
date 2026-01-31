// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: GPL-3.0
//
// ROS2 port of UndockingBehavior

#ifndef MOWER_LOGIC__BEHAVIORS__UNDOCKING_BEHAVIOR_HPP_
#define MOWER_LOGIC__BEHAVIORS__UNDOCKING_BEHAVIOR_HPP_

#include <vector>
#include <string>
#include <chrono>
#include <atomic>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "mower_logic/behaviors/behavior.hpp"
#include "xbot_msgs/msg/action_info.hpp"
#include "xbot_msgs/msg/absolute_pose.hpp"
#include "mower_map_msgs/srv/get_docking_point_srv.hpp"

// Nav2 includes
#include "nav2_msgs/action/follow_path.hpp"

namespace mower_logic
{

// Forward declarations
class MowingBehavior;
class DockingBehavior;
class IdleBehavior;

/**
 * Extended config for undocking behavior
 */
struct UndockingConfig
{
  double undock_distance = 2.0;
  double undock_angled_distance = 1.0;
  double undock_angle = 20.0;
  bool undock_fixed_angle = false;
  bool undock_use_curve = true;
  double undocking_waiting_time = 0.5;
  double gps_wait_time = 5.0;
};

class UndockingBehavior : public Behavior
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using FollowPathClient = rclcpp_action::Client<FollowPath>;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  UndockingBehavior();
  explicit UndockingBehavior(Behavior* next_behavior);

  static UndockingBehavior INSTANCE;
  static UndockingBehavior RETRY_INSTANCE;

  std::string state_name() override;
  Behavior* execute() override;
  void enter() override;
  void exit() override;
  void reset() override;
  bool needs_gps() override;
  bool mower_enabled() override;
  bool redirect_joystick() override;
  void command_home() override;
  void command_start() override;
  void command_s1() override;
  void command_s2() override;
  uint8_t get_sub_state() override;
  uint8_t get_state() override;
  void handle_action(const std::string& action) override;

private:
  bool wait_for_gps();

  // Nav2 action client
  FollowPathClient::SharedPtr follow_path_client_;

  // Service clients
  rclcpp::Client<mower_map_msgs::srv::GetDockingPointSrv>::SharedPtr docking_point_client_;

  // Subscribers
  rclcpp::Subscription<xbot_msgs::msg::AbsolutePose>::SharedPtr pose_sub_;
  xbot_msgs::msg::AbsolutePose last_pose_;
  std::atomic<bool> has_pose_{ false };

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // State
  std::vector<xbot_msgs::msg::ActionInfo> actions_;
  Behavior* next_behavior_ = nullptr;
  geometry_msgs::msg::PoseStamped docking_pose_stamped_;
  std::atomic<bool> gps_required_{ false };

  // Config
  UndockingConfig undock_config_;

  // Random number generator
  std::mt19937 rng_;
  bool rng_seeded_ = false;
};

}  // namespace mower_logic

#endif  // MOWER_LOGIC__BEHAVIORS__UNDOCKING_BEHAVIOR_HPP_
