// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: GPL-3.0
//
// ROS2 port of DockingBehavior

#ifndef MOWER_LOGIC__BEHAVIORS__DOCKING_BEHAVIOR_HPP_
#define MOWER_LOGIC__BEHAVIORS__DOCKING_BEHAVIOR_HPP_

#include <vector>
#include <string>
#include <chrono>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "mower_logic/behaviors/behavior.hpp"
#include "xbot_msgs/msg/action_info.hpp"
#include "mower_map_msgs/srv/get_docking_point_srv.hpp"
#include "mower_msgs/msg/status.hpp"
#include "mower_msgs/msg/power.hpp"

// Nav2 includes
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"

namespace mower_logic {

// Forward declarations
class IdleBehavior;
class UndockingBehavior;
class PerimeterSearchBehavior;

/**
 * Extended config for docking behavior
 */
struct DockingConfig {
    double docking_approach_distance = 1.5;
    double docking_distance = 1.0;
    double docking_extra_time = 0.5;
    double docking_waiting_time = 0.5;
    int docking_retry_count = 3;
};

class DockingBehavior : public Behavior {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using NavigateToPoseClient = rclcpp_action::Client<NavigateToPose>;
    using FollowPath = nav2_msgs::action::FollowPath;
    using FollowPathClient = rclcpp_action::Client<FollowPath>;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

    DockingBehavior();

    static DockingBehavior INSTANCE;

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
    bool approach_docking_point();
    bool dock_straight();

    // Nav2 action clients
    NavigateToPoseClient::SharedPtr nav_to_pose_client_;
    FollowPathClient::SharedPtr follow_path_client_;

    // Service clients
    rclcpp::Client<mower_map_msgs::srv::GetDockingPointSrv>::SharedPtr docking_point_client_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Functions provided by main logic node (accessed via callbacks)
    std::function<mower_msgs::msg::Power()> get_power_func_;
    std::function<void()> stop_moving_func_;
    std::function<void(bool)> set_gps_func_;

    // State
    std::vector<xbot_msgs::msg::ActionInfo> actions_;
    uint32_t retry_count_ = 0;
    bool in_approach_mode_ = true;
    geometry_msgs::msg::PoseStamped docking_pose_stamped_;

    // Docking-specific config
    DockingConfig dock_config_;
};

}  // namespace mower_logic

#endif  // MOWER_LOGIC__BEHAVIORS__DOCKING_BEHAVIOR_HPP_
