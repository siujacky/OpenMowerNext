// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: GPL-3.0
//
// ROS2 port of DockingBehavior

#include "mower_logic/behaviors/docking_behavior.hpp"
#include "mower_logic/behaviors/idle_behavior.hpp"
#include "mower_logic/behaviors/undocking_behavior.hpp"
#include "mower_logic/behaviors/perimeter_docking.hpp"
#include "mower_msgs/msg/high_level_status.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace mower_logic
{

DockingBehavior DockingBehavior::INSTANCE;

DockingBehavior::DockingBehavior()
{
  xbot_msgs::msg::ActionInfo abort_docking_action;
  abort_docking_action.action_id = "abort_docking";
  abort_docking_action.enabled = true;
  abort_docking_action.action_name = "Stop Docking";
  actions_.push_back(abort_docking_action);
}

std::string DockingBehavior::state_name()
{
  return "DOCKING";
}

void DockingBehavior::enter()
{
  paused_.store(false);
  aborted_.store(false);
  in_approach_mode_ = true;

  // Initialize Nav2 action clients
  nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
  follow_path_client_ = rclcpp_action::create_client<FollowPath>(node_, "follow_path");

  // Initialize service clients
  docking_point_client_ =
      node_->create_client<mower_map_msgs::srv::GetDockingPointSrv>("mower_map_service/get_docking_point");

  // Initialize publisher
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/logic_vel", 1);

  // Load config from parameters
  dock_config_.docking_approach_distance = node_->get_parameter_or("docking_approach_distance", 1.5);
  dock_config_.docking_distance = node_->get_parameter_or("docking_distance", 1.0);
  dock_config_.docking_extra_time = node_->get_parameter_or("docking_extra_time", 0.5);
  dock_config_.docking_waiting_time = node_->get_parameter_or("docking_waiting_time", 0.5);
  dock_config_.docking_retry_count = node_->get_parameter_or("docking_retry_count", 3);

  // Get docking pose from map service
  if (docking_point_client_->wait_for_service(5s))
  {
    auto request = std::make_shared<mower_map_msgs::srv::GetDockingPointSrv::Request>();
    auto future = docking_point_client_->async_send_request(request);

    if (future.wait_for(5s) == std::future_status::ready)
    {
      auto response = future.get();
      docking_pose_stamped_.pose = response->docking_pose;
      docking_pose_stamped_.header.frame_id = "map";
      docking_pose_stamped_.header.stamp = node_->now();
    }
  }

  for (auto& a : actions_)
  {
    a.enabled = true;
  }
}

void DockingBehavior::exit()
{
  for (auto& a : actions_)
  {
    a.enabled = false;
  }
}

void DockingBehavior::reset()
{
  retry_count_ = 0;
}

bool DockingBehavior::needs_gps()
{
  return in_approach_mode_;
}

bool DockingBehavior::mower_enabled()
{
  return false;
}

bool DockingBehavior::redirect_joystick()
{
  return false;
}

bool DockingBehavior::approach_docking_point()
{
  RCLCPP_INFO(node_->get_logger(), "Calculating approach path");

  // Wait for Nav2 action servers
  if (!nav_to_pose_client_->wait_for_action_server(10s))
  {
    RCLCPP_ERROR(node_->get_logger(), "DockingBehavior: Nav2 action server not available");
    return false;
  }
  if (!follow_path_client_->wait_for_action_server(10s))
  {
    RCLCPP_ERROR(node_->get_logger(), "DockingBehavior: Follow path action server not available");
    return false;
  }

  // Extract yaw from docking pose
  tf2::Quaternion quat;
  tf2::fromMsg(docking_pose_stamped_.pose.orientation, quat);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Navigate to approach point (behind docking point)
  {
    geometry_msgs::msg::PoseStamped docking_approach_point = docking_pose_stamped_;
    docking_approach_point.pose.position.x -= std::cos(yaw) * dock_config_.docking_approach_distance;
    docking_approach_point.pose.position.y -= std::sin(yaw) * dock_config_.docking_approach_distance;
    docking_approach_point.header.stamp = node_->now();

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = docking_approach_point;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    auto goal_future = nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);

    if (goal_future.wait_for(5s) != std::future_status::ready)
    {
      RCLCPP_ERROR(node_->get_logger(), "DockingBehavior: Failed to send approach goal");
      return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
      RCLCPP_ERROR(node_->get_logger(), "DockingBehavior: Approach goal rejected");
      return false;
    }

    // Wait for result
    while (rclcpp::ok())
    {
      auto status = goal_handle->get_status();

      if (status == rclcpp_action::GoalStatus::STATUS_SUCCEEDED)
      {
        break;
      }
      else if (status == rclcpp_action::GoalStatus::STATUS_ABORTED ||
               status == rclcpp_action::GoalStatus::STATUS_CANCELED)
      {
        RCLCPP_ERROR(node_->get_logger(), "DockingBehavior: Approach navigation failed");
        return false;
      }

      if (aborted_.load())
      {
        nav_to_pose_client_->async_cancel_all_goals();
        return false;
      }

      std::this_thread::sleep_for(100ms);
    }
  }

  // Wait a moment at approach point
  auto start_wait_time = node_->now();
  rclcpp::Rate loop_rate(100);
  while (rclcpp::ok() && (node_->now() - start_wait_time).seconds() < dock_config_.docking_waiting_time)
  {
    loop_rate.sleep();
  }

  // Execute approach path to docking point
  {
    auto follow_goal = FollowPath::Goal();
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();

    int dock_point_count = static_cast<int>(dock_config_.docking_approach_distance * 10.0);
    for (int i = 0; i <= dock_point_count; i++)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose = docking_pose_stamped_.pose;
      pose.pose.position.x -= std::cos(yaw) * ((dock_point_count - i) / 10.0);
      pose.pose.position.y -= std::sin(yaw) * ((dock_point_count - i) / 10.0);
      path.poses.push_back(pose);
    }

    follow_goal.path = path;
    follow_goal.controller_id = "FTCPlanner";

    RCLCPP_INFO(node_->get_logger(), "Executing Docking Approach");

    auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    auto goal_future = follow_path_client_->async_send_goal(follow_goal, send_goal_options);

    if (goal_future.wait_for(5s) != std::future_status::ready)
    {
      RCLCPP_ERROR(node_->get_logger(), "DockingBehavior: Failed to send approach path goal");
      return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
      RCLCPP_ERROR(node_->get_logger(), "DockingBehavior: Approach path goal rejected");
      return false;
    }

    // Wait for result
    while (rclcpp::ok())
    {
      auto status = goal_handle->get_status();

      if (status == rclcpp_action::GoalStatus::STATUS_SUCCEEDED)
      {
        break;
      }
      else if (status == rclcpp_action::GoalStatus::STATUS_ABORTED ||
               status == rclcpp_action::GoalStatus::STATUS_CANCELED)
      {
        RCLCPP_ERROR(node_->get_logger(), "DockingBehavior: Approach path failed");
        return false;
      }

      if (aborted_.load())
      {
        follow_path_client_->async_cancel_all_goals();
        return false;
      }

      std::this_thread::sleep_for(100ms);
    }
  }

  return true;
}

bool DockingBehavior::dock_straight()
{
  // Extract yaw from docking pose
  tf2::Quaternion quat;
  tf2::fromMsg(docking_pose_stamped_.pose.orientation, quat);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  auto follow_goal = FollowPath::Goal();
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = node_->now();

  int dock_point_count = static_cast<int>(dock_config_.docking_distance * 10.0);
  for (int i = 0; i < dock_point_count; i++)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose = docking_pose_stamped_.pose;
    pose.pose.position.x += std::cos(yaw) * (i / 10.0);
    pose.pose.position.y += std::sin(yaw) * (i / 10.0);
    path.poses.push_back(pose);
  }

  follow_goal.path = path;
  follow_goal.controller_id = "DockingFTCPlanner";

  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  auto goal_future = follow_path_client_->async_send_goal(follow_goal, send_goal_options);

  if (goal_future.wait_for(5s) != std::future_status::ready)
  {
    return false;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle)
  {
    return false;
  }

  bool docking_success = false;
  rclcpp::Rate rate(10);

  // Wait for docking completion (charging voltage detected)
  while (rclcpp::ok())
  {
    auto status = goal_handle->get_status();

    // Check for charging - this would need access to power message
    // For now, we check if path completed
    if (status == rclcpp_action::GoalStatus::STATUS_SUCCEEDED)
    {
      RCLCPP_INFO(node_->get_logger(), "Docking path completed");
      docking_success = true;
      break;
    }
    else if (status == rclcpp_action::GoalStatus::STATUS_ABORTED ||
             status == rclcpp_action::GoalStatus::STATUS_CANCELED)
    {
      RCLCPP_WARN(node_->get_logger(), "Docking path failed");
      break;
    }

    if (aborted_.load())
    {
      RCLCPP_INFO(node_->get_logger(), "Docking aborted");
      follow_path_client_->async_cancel_all_goals();
      // Stop robot
      geometry_msgs::msg::Twist stop;
      cmd_vel_pub_->publish(stop);
      break;
    }

    rate.sleep();
  }

  // Stop the robot
  geometry_msgs::msg::Twist stop;
  cmd_vel_pub_->publish(stop);

  return docking_success;
}

Behavior* DockingBehavior::execute()
{
  // Note: In full implementation, we would check if already docked via power message
  // if (get_power_func_ && get_power_func_().v_charge > 5.0) {
  //     return &IdleBehavior::DOCKED_INSTANCE;
  // }

  // Wait for good GPS
  while (!hasGoodGPS())
  {
    if (aborted_.load())
    {
      RCLCPP_INFO(node_->get_logger(), "Docking aborted");
      geometry_msgs::msg::Twist stop;
      cmd_vel_pub_->publish(stop);
      return &IdleBehavior::INSTANCE;
    }

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Waiting for good GPS");
    std::this_thread::sleep_for(1s);
  }

  bool approach_success = approach_docking_point();

  if (aborted_.load())
  {
    RCLCPP_INFO(node_->get_logger(), "Docking aborted");
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    return &IdleBehavior::INSTANCE;
  }

  if (!approach_success)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error during docking approach");
    retry_count_++;
    if (retry_count_ <= static_cast<uint32_t>(dock_config_.docking_retry_count))
    {
      RCLCPP_ERROR(node_->get_logger(), "Retrying docking approach");
      return &DockingBehavior::INSTANCE;
    }
    RCLCPP_ERROR(node_->get_logger(), "Giving up on docking");
    return &IdleBehavior::INSTANCE;
  }

  // Disable GPS for straight docking
  in_approach_mode_ = false;
  // set_gps_func_(false);  // Would disable GPS

  // Check for perimeter docking mode
  // if (PerimeterSearchBehavior::configured(config_)) {
  //     return &PerimeterSearchBehavior::INSTANCE;
  // }

  bool docked = dock_straight();

  if (aborted_.load())
  {
    RCLCPP_INFO(node_->get_logger(), "Docking aborted");
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    return &IdleBehavior::INSTANCE;
  }

  if (!docked)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error during docking");
    retry_count_++;
    if (retry_count_ <= static_cast<uint32_t>(dock_config_.docking_retry_count) && !aborted_.load())
    {
      RCLCPP_ERROR(node_->get_logger(), "Retrying docking. Try %d/%d", retry_count_, dock_config_.docking_retry_count);
      return &UndockingBehavior::RETRY_INSTANCE;
    }
    RCLCPP_ERROR(node_->get_logger(), "Giving up on docking");
    reset();
    return &IdleBehavior::INSTANCE;
  }

  reset();
  return &IdleBehavior::DOCKED_INSTANCE;
}

void DockingBehavior::command_home()
{
  // Already going home
}

void DockingBehavior::command_start()
{
  abort();
}

void DockingBehavior::command_s1()
{
  // Not used
}

void DockingBehavior::command_s2()
{
  // Not used
}

uint8_t DockingBehavior::get_sub_state()
{
  return 1;
}

uint8_t DockingBehavior::get_state()
{
  return mower_msgs::msg::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

void DockingBehavior::handle_action(const std::string& action)
{
  if (action == "mower_logic:docking/abort_docking")
  {
    RCLCPP_INFO(node_->get_logger(), "Got abort docking command");
    command_start();
  }
}

}  // namespace mower_logic
