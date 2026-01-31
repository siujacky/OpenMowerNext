// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: GPL-3.0
//
// ROS2 port of AreaRecordingBehavior

#ifndef MOWER_LOGIC__BEHAVIORS__AREA_RECORDING_BEHAVIOR_HPP_
#define MOWER_LOGIC__BEHAVIORS__AREA_RECORDING_BEHAVIOR_HPP_

#include <vector>
#include <string>
#include <chrono>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "mower_logic/behaviors/behavior.hpp"
#include "xbot_msgs/msg/action_info.hpp"
#include "xbot_msgs/msg/absolute_pose.hpp"
#include "xbot_msgs/msg/map_overlay.hpp"
#include "mower_map_msgs/msg/map_area.hpp"
#include "mower_map_msgs/srv/add_mowing_area_srv.hpp"
#include "mower_map_msgs/srv/set_docking_point_srv.hpp"

namespace mower_logic
{

// Forward declarations
class IdleBehavior;

// Minimum distance between recorded points
constexpr double NEW_POINT_MIN_DISTANCE = 0.1;

class AreaRecordingBehavior : public Behavior
{
public:
  AreaRecordingBehavior();

  static AreaRecordingBehavior INSTANCE;

  std::string state_name() override;
  std::string sub_state_name() override;
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
  bool record_new_polygon(geometry_msgs::msg::Polygon& polygon, xbot_msgs::msg::MapOverlay& result_overlay);
  bool get_docking_position(geometry_msgs::msg::Pose& pos);
  void pose_received(const xbot_msgs::msg::AbsolutePose::SharedPtr msg);
  void joy_received(const sensor_msgs::msg::Joy::SharedPtr msg);
  void record_dock_received(const std_msgs::msg::Bool::SharedPtr msg);
  void record_polygon_received(const std_msgs::msg::Bool::SharedPtr msg);
  void record_mowing_received(const std_msgs::msg::Bool::SharedPtr msg);
  void record_navigation_received(const std_msgs::msg::Bool::SharedPtr msg);
  void record_auto_point_collecting(const std_msgs::msg::Bool::SharedPtr msg);
  void record_collect_point(const std_msgs::msg::Bool::SharedPtr msg);
  void update_actions();

  // State
  std::atomic<bool> has_odom_{ false };
  std::vector<xbot_msgs::msg::ActionInfo> actions_;

  sensor_msgs::msg::Joy last_joy_;
  xbot_msgs::msg::AbsolutePose last_pose_;
  std::mutex pose_mutex_;

  // Publishers
  rclcpp::Publisher<xbot_msgs::msg::MapOverlay>::SharedPtr map_overlay_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<xbot_msgs::msg::AbsolutePose>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dock_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr polygon_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mow_area_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_area_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_point_collecting_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collect_point_sub_;

  // Service clients
  rclcpp::Client<mower_map_msgs::srv::AddMowingAreaSrv>::SharedPtr add_mowing_area_client_;
  rclcpp::Client<mower_map_msgs::srv::SetDockingPointSrv>::SharedPtr set_docking_point_client_;

  // Recording state
  std::atomic<bool> has_first_docking_pos_{ false };
  geometry_msgs::msg::Pose first_docking_pos_;

  std::atomic<bool> poly_recording_enabled_{ false };
  std::atomic<bool> is_mowing_area_{ false };
  std::atomic<bool> is_navigation_area_{ false };
  std::atomic<bool> finished_all_{ false };
  std::atomic<bool> set_docking_position_{ false };
  std::atomic<bool> has_outline_{ false };
  std::atomic<bool> auto_point_collecting_{ true };
  std::atomic<bool> collect_point_{ false };
  std::atomic<bool> manual_mowing_{ false };

  // Visualization
  visualization_msgs::msg::MarkerArray markers_;
  visualization_msgs::msg::Marker marker_;
};

}  // namespace mower_logic

#endif  // MOWER_LOGIC__BEHAVIORS__AREA_RECORDING_BEHAVIOR_HPP_
