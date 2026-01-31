// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: GPL-3.0
//
// ROS2 port of MowingBehavior

#ifndef MOWER_LOGIC__BEHAVIORS__MOWING_BEHAVIOR_HPP_
#define MOWER_LOGIC__BEHAVIORS__MOWING_BEHAVIOR_HPP_

#include <vector>
#include <string>
#include <chrono>
#include <atomic>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "mower_logic/behaviors/behavior.hpp"
#include "xbot_msgs/msg/action_info.hpp"
#include "slic3r_coverage_planner_msgs/msg/path.hpp"
#include "slic3r_coverage_planner_msgs/srv/plan_path.hpp"
#include "mower_map_msgs/srv/get_mowing_area_srv.hpp"
#include "mower_map_msgs/srv/set_nav_point_srv.hpp"
#include "mower_map_msgs/srv/clear_nav_point_srv.hpp"
#include "ftc_local_planner_msgs/srv/planner_get_progress.hpp"

// Nav2 includes
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"

namespace mower_logic {

// Forward declarations
class DockingBehavior;
class IdleBehavior;

/**
 * Extended config for mowing behavior
 */
struct MowingConfig {
    double mow_angle_offset = 0.0;
    double mow_angle_increment = 0.0;
    bool mow_angle_offset_is_absolute = false;
    int outline_count = 4;
    int outline_overlap_count = 0;
    double outline_offset = 0.05;
    double tool_width = 0.13;
    bool add_fake_obstacle = false;
    int max_first_point_attempts = 3;
    int max_first_point_trim_attempts = 3;
};

class MowingBehavior : public Behavior {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using NavigateToPoseClient = rclcpp_action::Client<NavigateToPose>;
    using FollowPath = nav2_msgs::action::FollowPath;
    using FollowPathClient = rclcpp_action::Client<FollowPath>;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

    MowingBehavior();

    static MowingBehavior INSTANCE;

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

    int16_t get_current_area() const { return current_mowing_area_; }
    int16_t get_current_path() const { return current_mowing_path_; }
    int16_t get_current_path_index() const { return current_mowing_path_index_; }

private:
    bool create_mowing_plan(int area_index);
    bool execute_mowing_plan();
    int get_current_mow_path_index();
    void update_actions();
    void checkpoint();
    bool restore_checkpoint();

    // Nav2 action clients - initialized in enter()
    NavigateToPoseClient::SharedPtr nav_to_pose_client_;
    FollowPathClient::SharedPtr follow_path_client_;

    // Service clients
    rclcpp::Client<mower_map_msgs::srv::GetMowingAreaSrv>::SharedPtr map_client_;
    rclcpp::Client<slic3r_coverage_planner_msgs::srv::PlanPath>::SharedPtr path_client_;
    rclcpp::Client<ftc_local_planner_msgs::srv::PlannerGetProgress>::SharedPtr path_progress_client_;
    rclcpp::Client<mower_map_msgs::srv::SetNavPointSrv>::SharedPtr set_nav_point_client_;
    rclcpp::Client<mower_map_msgs::srv::ClearNavPointSrv>::SharedPtr clear_nav_point_client_;

    // State
    std::vector<xbot_msgs::msg::ActionInfo> actions_;
    std::atomic<bool> skip_area_{false};
    std::atomic<bool> skip_path_{false};
    std::atomic<bool> mower_enabled_{false};

    // Progress
    std::vector<slic3r_coverage_planner_msgs::msg::Path> current_mowing_paths_;
    rclcpp::Time last_checkpoint_;
    int current_mowing_path_ = 0;
    int current_mowing_area_ = 0;
    int current_mowing_path_index_ = 0;
    std::string current_mowing_plan_digest_;
    double current_mowing_angle_increment_sum_ = 0.0;

    // Mowing-specific config
    MowingConfig mow_config_;
};

}  // namespace mower_logic

#endif  // MOWER_LOGIC__BEHAVIORS__MOWING_BEHAVIOR_HPP_
