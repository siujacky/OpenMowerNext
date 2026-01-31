// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: GPL-3.0
//
// ROS2 port of UndockingBehavior

#include "mower_logic/behaviors/undocking_behavior.hpp"
#include "mower_logic/behaviors/mowing_behavior.hpp"
#include "mower_logic/behaviors/docking_behavior.hpp"
#include "mower_logic/behaviors/idle_behavior.hpp"
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

// Static instances
UndockingBehavior UndockingBehavior::INSTANCE{};
UndockingBehavior UndockingBehavior::RETRY_INSTANCE{ &DockingBehavior::INSTANCE };

UndockingBehavior::UndockingBehavior() : next_behavior_(&MowingBehavior::INSTANCE)
{
  xbot_msgs::msg::ActionInfo abort_undocking_action;
  abort_undocking_action.action_id = "abort_undocking";
  abort_undocking_action.enabled = true;
  abort_undocking_action.action_name = "Stop Undocking";
  actions_.push_back(abort_undocking_action);
}

UndockingBehavior::UndockingBehavior(Behavior* next) : next_behavior_(next)
{
  xbot_msgs::msg::ActionInfo abort_undocking_action;
  abort_undocking_action.action_id = "abort_undocking";
  abort_undocking_action.enabled = true;
  abort_undocking_action.action_name = "Stop Undocking";
  actions_.push_back(abort_undocking_action);
}

std::string UndockingBehavior::state_name()
{
  return "UNDOCKING";
}

void UndockingBehavior::enter()
{
  reset();
  paused_.store(false);
  aborted_.store(false);
  has_pose_.store(false);

  // Initialize Nav2 action client
  follow_path_client_ = rclcpp_action::create_client<FollowPath>(node_, "follow_path");

  // Initialize service client
  docking_point_client_ =
      node_->create_client<mower_map_msgs::srv::GetDockingPointSrv>("mower_map_service/get_docking_point");

  // Initialize publisher
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/logic_vel", 1);

  // Subscribe to pose
  pose_sub_ = node_->create_subscription<xbot_msgs::msg::AbsolutePose>(
      "/xbot_positioning/xb_pose", 10, [this](const xbot_msgs::msg::AbsolutePose::SharedPtr msg) {
        last_pose_ = *msg;
        has_pose_.store(true);
      });

  // Load config from parameters
  undock_config_.undock_distance = node_->get_parameter_or("undock_distance", 2.0);
  undock_config_.undock_angled_distance = node_->get_parameter_or("undock_angled_distance", 1.0);
  undock_config_.undock_angle = node_->get_parameter_or("undock_angle", 20.0);
  undock_config_.undock_fixed_angle = node_->get_parameter_or("undock_fixed_angle", false);
  undock_config_.undock_use_curve = node_->get_parameter_or("undock_use_curve", true);
  undock_config_.undocking_waiting_time = node_->get_parameter_or("undocking_waiting_time", 0.5);
  undock_config_.gps_wait_time = node_->get_parameter_or("gps_wait_time", 5.0);

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

  // Note: In full implementation, we would check if docked and set robot pose
  // if (get_power_func_ && get_power_func_().v_charge > 5.0) {
  //     set_robot_pose_(docking_pose_stamped_.pose);
  // }

  for (auto& a : actions_)
  {
    a.enabled = true;
  }
}

void UndockingBehavior::exit()
{
  pose_sub_.reset();
  for (auto& a : actions_)
  {
    a.enabled = false;
  }
}

void UndockingBehavior::reset()
{
  gps_required_.store(false);
}

bool UndockingBehavior::needs_gps()
{
  return gps_required_.load();
}

bool UndockingBehavior::mower_enabled()
{
  return false;
}

bool UndockingBehavior::redirect_joystick()
{
  return false;
}

bool UndockingBehavior::wait_for_gps()
{
  gps_required_.store(false);
  // Note: Would call set_gps_(true) to enable GPS

  rclcpp::Rate rate(1.0);
  while (rclcpp::ok() && !aborted_.load())
  {
    if (hasGoodGPS())
    {
      RCLCPP_INFO(node_->get_logger(), "Got good GPS, let's go");
      break;
    }
    else
    {
      if (has_pose_.load())
      {
        RCLCPP_INFO(node_->get_logger(), "Waiting for GPS. Current accuracy: %f", last_pose_.position_accuracy);
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(), "Waiting for GPS...");
      }
      rate.sleep();
    }
  }

  if (!rclcpp::ok() || aborted_.load())
  {
    return false;
  }

  // Wait additional time for odometry filters to converge
  std::this_thread::sleep_for(std::chrono::duration<double>(undock_config_.gps_wait_time));

  gps_required_.store(true);
  return true;
}

Behavior* UndockingBehavior::execute()
{
  // Wait for pose
  while (!has_pose_.load() && rclcpp::ok() && !aborted_.load())
  {
    std::this_thread::sleep_for(100ms);
  }

  if (aborted_.load())
  {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    return &IdleBehavior::INSTANCE;
  }

  // Wait for action server
  if (!follow_path_client_->wait_for_action_server(10s))
  {
    RCLCPP_ERROR(node_->get_logger(), "UndockingBehavior: Follow path action server not available");
    return &IdleBehavior::INSTANCE;
  }

  // Get robot's current pose from odometry
  auto pose = last_pose_;
  tf2::Quaternion quat;
  tf2::fromMsg(pose.pose.pose.orientation, quat);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Wait a moment
  auto start_wait_time = node_->now();
  rclcpp::Rate loop_rate(100);
  while (rclcpp::ok() && (node_->now() - start_wait_time).seconds() < undock_config_.undocking_waiting_time)
  {
    loop_rate.sleep();
  }

  // Build undocking path
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = node_->now();

  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.pose = pose.pose.pose;
  current_pose.header = path.header;

  // Straight undocking portion
  const int straight_undock_point_count = 3;  // FTC planner requires at least 3 points
  double incremental_distance = undock_config_.undock_distance / straight_undock_point_count;
  path.poses.push_back(current_pose);  // Start from current position

  for (int i = 0; i < straight_undock_point_count; i++)
  {
    current_pose.pose.position.x -= std::cos(yaw) * incremental_distance;
    current_pose.pose.position.y -= std::sin(yaw) * incremental_distance;
    path.poses.push_back(current_pose);
  }

  // Calculate undock angle
  double angle;
  if (undock_config_.undock_fixed_angle)
  {
    angle = undock_config_.undock_angle * M_PI / 180.0;
    RCLCPP_INFO(node_->get_logger(), "Fixed angle undock: %f", undock_config_.undock_angle);
  }
  else
  {
    // Seed RNG based on time
    if (!rng_seeded_)
    {
      rng_.seed(static_cast<uint32_t>(node_->now().seconds()));
      RCLCPP_INFO(node_->get_logger(), "Random angle undock: Seeded RNG");
      rng_seeded_ = true;
    }

    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    double random_number = dist(rng_);
    double random_angle_deg = std::abs(undock_config_.undock_angle) * random_number;
    RCLCPP_INFO(node_->get_logger(), "Random angle undock: %f", random_angle_deg);
    angle = random_angle_deg * M_PI / 180.0;
  }

  // Angled undocking portion
  const int angled_undock_point_count = 10;
  incremental_distance = undock_config_.undock_angled_distance / angled_undock_point_count;

  for (int i = 0; i < angled_undock_point_count; i++)
  {
    double orientation =
        yaw + angle * (undock_config_.undock_use_curve ? ((i + 1.0) / angled_undock_point_count) : 1.0);

    current_pose.pose.position.x -= std::cos(orientation) * incremental_distance;
    current_pose.pose.position.y -= std::sin(orientation) * incremental_distance;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, orientation);
    current_pose.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(current_pose);
  }

  // Execute undocking path
  auto follow_goal = FollowPath::Goal();
  follow_goal.path = path;
  follow_goal.controller_id = "DockingFTCPlanner";

  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  auto goal_future = follow_path_client_->async_send_goal(follow_goal, send_goal_options);

  if (goal_future.wait_for(5s) != std::future_status::ready)
  {
    RCLCPP_ERROR(node_->get_logger(), "UndockingBehavior: Failed to send undock path goal");
    return &IdleBehavior::INSTANCE;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(node_->get_logger(), "UndockingBehavior: Undock path goal rejected");
    return &IdleBehavior::INSTANCE;
  }

  bool success = false;
  while (rclcpp::ok())
  {
    auto status = goal_handle->get_status();

    if (status == rclcpp_action::GoalStatus::STATUS_SUCCEEDED)
    {
      success = true;
      break;
    }
    else if (status == rclcpp_action::GoalStatus::STATUS_ABORTED ||
             status == rclcpp_action::GoalStatus::STATUS_CANCELED)
    {
      break;
    }

    if (aborted_.load())
    {
      RCLCPP_INFO(node_->get_logger(), "Undocking aborted");
      follow_path_client_->async_cancel_all_goals();
      geometry_msgs::msg::Twist stop;
      cmd_vel_pub_->publish(stop);
      return &IdleBehavior::INSTANCE;
    }

    std::this_thread::sleep_for(100ms);
  }

  // Stop the robot
  geometry_msgs::msg::Twist stop;
  cmd_vel_pub_->publish(stop);

  if (!success)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error during undock");
    return &IdleBehavior::INSTANCE;
  }

  RCLCPP_INFO(node_->get_logger(), "Undock success. Waiting for GPS.");
  bool has_gps = wait_for_gps();

  if (!has_gps)
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not get GPS");
    return &IdleBehavior::INSTANCE;
  }

  return next_behavior_;
}

void UndockingBehavior::command_home()
{
  abort();
}

void UndockingBehavior::command_start()
{
  // Not used
}

void UndockingBehavior::command_s1()
{
  // Not used
}

void UndockingBehavior::command_s2()
{
  // Not used
}

uint8_t UndockingBehavior::get_sub_state()
{
  return 2;
}

uint8_t UndockingBehavior::get_state()
{
  return mower_msgs::msg::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

void UndockingBehavior::handle_action(const std::string& action)
{
  if (action == "mower_logic:undocking/abort_undocking")
  {
    RCLCPP_INFO(node_->get_logger(), "Got abort undocking command");
    command_home();
  }
}

}  // namespace mower_logic
