// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: GPL-3.0
//
// ROS2 port of MowingBehavior

#include "mower_logic/behaviors/mowing_behavior.hpp"
#include "mower_logic/behaviors/docking_behavior.hpp"
#include "mower_logic/behaviors/idle_behavior.hpp"
#include "mower_msgs/msg/high_level_status.hpp"

#include <cmath>
#include <fstream>
#include <functional>
#include <chrono>
#include <thread>

// For SHA256 hash - using simple implementation for checkpoint
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace mower_logic {

MowingBehavior MowingBehavior::INSTANCE;

MowingBehavior::MowingBehavior() {
    // Build actions list
    xbot_msgs::msg::ActionInfo pause_action;
    pause_action.action_id = "pause";
    pause_action.enabled = false;
    pause_action.action_name = "Pause Mowing";
    actions_.push_back(pause_action);

    xbot_msgs::msg::ActionInfo continue_action;
    continue_action.action_id = "continue";
    continue_action.enabled = false;
    continue_action.action_name = "Continue Mowing";
    actions_.push_back(continue_action);

    xbot_msgs::msg::ActionInfo abort_mowing_action;
    abort_mowing_action.action_id = "abort_mowing";
    abort_mowing_action.enabled = false;
    abort_mowing_action.action_name = "Stop Mowing";
    actions_.push_back(abort_mowing_action);

    xbot_msgs::msg::ActionInfo skip_area_action;
    skip_area_action.action_id = "skip_area";
    skip_area_action.enabled = false;
    skip_area_action.action_name = "Skip Area";
    actions_.push_back(skip_area_action);

    xbot_msgs::msg::ActionInfo skip_path_action;
    skip_path_action.action_id = "skip_path";
    skip_path_action.enabled = false;
    skip_path_action.action_name = "Skip Path";
    actions_.push_back(skip_path_action);

    restore_checkpoint();
}

std::string MowingBehavior::state_name() {
    if (paused_.load()) {
        return "PAUSED";
    }
    return "MOWING";
}

void MowingBehavior::enter() {
    skip_area_.store(false);
    skip_path_.store(false);
    paused_.store(false);
    aborted_.store(false);

    // Initialize Nav2 action clients
    nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
        node_, "navigate_to_pose");
    follow_path_client_ = rclcpp_action::create_client<FollowPath>(
        node_, "follow_path");

    // Initialize service clients
    map_client_ = node_->create_client<mower_map_msgs::srv::GetMowingAreaSrv>(
        "mower_map_service/get_mowing_area");
    path_client_ = node_->create_client<slic3r_coverage_planner_msgs::srv::PlanPath>(
        "slic3r_coverage_planner/plan_path");
    path_progress_client_ = node_->create_client<ftc_local_planner_msgs::srv::PlannerGetProgress>(
        "ftc_local_planner/get_progress");
    set_nav_point_client_ = node_->create_client<mower_map_msgs::srv::SetNavPointSrv>(
        "mower_map_service/set_nav_point");
    clear_nav_point_client_ = node_->create_client<mower_map_msgs::srv::ClearNavPointSrv>(
        "mower_map_service/clear_nav_point");

    // Load mowing config from parameters
    mow_config_.mow_angle_offset = node_->get_parameter_or("mow_angle_offset", 0.0);
    mow_config_.mow_angle_increment = node_->get_parameter_or("mow_angle_increment", 0.0);
    mow_config_.mow_angle_offset_is_absolute = node_->get_parameter_or("mow_angle_offset_is_absolute", false);
    mow_config_.outline_count = node_->get_parameter_or("outline_count", 4);
    mow_config_.outline_overlap_count = node_->get_parameter_or("outline_overlap_count", 0);
    mow_config_.outline_offset = node_->get_parameter_or("outline_offset", 0.05);
    mow_config_.tool_width = node_->get_parameter_or("tool_width", 0.13);
    mow_config_.add_fake_obstacle = node_->get_parameter_or("add_fake_obstacle", false);
    mow_config_.max_first_point_attempts = node_->get_parameter_or("max_first_point_attempts", 3);
    mow_config_.max_first_point_trim_attempts = node_->get_parameter_or("max_first_point_trim_attempts", 3);

    for (auto& a : actions_) {
        a.enabled = true;
    }
    // Would register actions here via service call
}

void MowingBehavior::exit() {
    for (auto& a : actions_) {
        a.enabled = false;
    }
    // Would unregister actions here
}

void MowingBehavior::reset() {
    current_mowing_paths_.clear();
    current_mowing_area_ = 0;
    current_mowing_path_ = 0;
    current_mowing_path_index_ = 0;
    // Increase cumulative mowing angle offset increment
    current_mowing_angle_increment_sum_ = std::fmod(
        current_mowing_angle_increment_sum_ + mow_config_.mow_angle_increment, 360.0);
    checkpoint();

    if (config_.enable_mower) {
        RCLCPP_INFO(node_->get_logger(), "MowingBehavior: Finished semiautomatic task");
        shared_state_->active_semiautomatic_task = false;
    }
}

bool MowingBehavior::needs_gps() {
    return true;
}

bool MowingBehavior::mower_enabled() {
    return mower_enabled_.load();
}

bool MowingBehavior::redirect_joystick() {
    return false;
}

Behavior* MowingBehavior::execute() {
    shared_state_->active_semiautomatic_task = true;

    while (rclcpp::ok() && !aborted_.load()) {
        if (current_mowing_paths_.empty() && !create_mowing_plan(current_mowing_area_)) {
            RCLCPP_INFO(node_->get_logger(), "MowingBehavior: Could not create mowing plan, docking");
            reset();
            return &DockingBehavior::INSTANCE;
        }

        // We have a plan, execute it
        RCLCPP_INFO(node_->get_logger(), "MowingBehavior: Executing mowing plan");
        bool finished = execute_mowing_plan();
        if (finished) {
            RCLCPP_INFO(node_->get_logger(), "MowingBehavior: Executing mowing plan - finished");
            current_mowing_area_++;
            current_mowing_paths_.clear();
            current_mowing_path_ = 0;
            current_mowing_path_index_ = 0;
        }
    }

    if (!rclcpp::ok()) {
        return nullptr;
    }
    // Aborted, go to docking
    return &DockingBehavior::INSTANCE;
}

bool MowingBehavior::create_mowing_plan(int area_index) {
    RCLCPP_INFO(node_->get_logger(), "MowingBehavior: Creating mowing plan for area: %d", area_index);
    
    current_mowing_paths_.clear();

    // Get the mowing area
    if (!map_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(node_->get_logger(), "MowingBehavior: Map service not available");
        return false;
    }

    auto map_request = std::make_shared<mower_map_msgs::srv::GetMowingAreaSrv::Request>();
    map_request->index = area_index;

    auto map_future = map_client_->async_send_request(map_request);
    if (map_future.wait_for(10s) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "MowingBehavior: Timeout getting mowing area");
        return false;
    }

    auto map_response = map_future.get();
    if (!map_response || map_response->area.area.points.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "MowingBehavior: Error loading mowing area");
        return false;
    }

    // Calculate area orientation from first points
    double angle = 0.0;
    auto& points = map_response->area.area.points;
    if (points.size() >= 2) {
        double first_x = points[0].x;
        double first_y = points[0].y;
        for (const auto& point : points) {
            double dx = point.x - first_x;
            double dy = point.y - first_y;
            double length = std::sqrt(dx * dx + dy * dy);
            if (length > 2.0) {
                angle = std::atan2(dy, dx);
                RCLCPP_INFO(node_->get_logger(), "MowingBehavior: Detected mow angle: %f", angle);
                break;
            }
        }
    }

    // Apply mowing angle offset
    double mow_angle_offset = std::fmod(
        mow_config_.mow_angle_offset + current_mowing_angle_increment_sum_ + 180.0, 360.0);
    if (mow_angle_offset < 0) mow_angle_offset += 360.0;
    mow_angle_offset -= 180.0;
    RCLCPP_INFO(node_->get_logger(), "MowingBehavior: mowing angle offset (deg): %f", mow_angle_offset);

    if (mow_config_.mow_angle_offset_is_absolute) {
        angle = mow_angle_offset * (M_PI / 180.0);
        RCLCPP_INFO(node_->get_logger(), "MowingBehavior: Custom mowing angle: %f", angle);
    } else {
        angle = angle + mow_angle_offset * (M_PI / 180.0);
        RCLCPP_INFO(node_->get_logger(), "MowingBehavior: Auto-detected mowing angle + offset: %f", angle);
    }

    // Calculate coverage path
    if (!path_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(node_->get_logger(), "MowingBehavior: Coverage planner service not available");
        return false;
    }

    auto path_request = std::make_shared<slic3r_coverage_planner_msgs::srv::PlanPath::Request>();
    path_request->angle = angle;
    path_request->outline_count = mow_config_.outline_count;
    path_request->outline_overlap_count = mow_config_.outline_overlap_count;
    path_request->outline = map_response->area.area;
    path_request->holes = map_response->area.obstacles;
    path_request->fill_type = slic3r_coverage_planner_msgs::srv::PlanPath::Request::FILL_LINEAR;
    path_request->outer_offset = mow_config_.outline_offset;
    path_request->distance = mow_config_.tool_width;

    auto path_future = path_client_->async_send_request(path_request);
    if (path_future.wait_for(30s) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "MowingBehavior: Timeout during coverage planning");
        return false;
    }

    auto path_response = path_future.get();
    if (!path_response || path_response->paths.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "MowingBehavior: Error during coverage planning");
        return false;
    }

    current_mowing_paths_ = path_response->paths;

    // Calculate simple digest for checkpoint comparison
    std::stringstream digest_ss;
    for (const auto& path : current_mowing_paths_) {
        digest_ss << path.path.poses.size() << "_";
        if (!path.path.poses.empty()) {
            const auto& p = path.path.poses.front().pose.position;
            digest_ss << std::fixed << std::setprecision(2) << p.x << "_" << p.y << "_";
        }
    }
    std::string mowing_plan_digest = digest_ss.str();

    // Check if we should use checkpoint
    if (mowing_plan_digest == current_mowing_plan_digest_) {
        RCLCPP_INFO(node_->get_logger(), 
            "MowingBehavior: Advancing to checkpoint, path: %d index: %d",
            current_mowing_path_, current_mowing_path_index_);
    } else {
        RCLCPP_INFO(node_->get_logger(),
            "MowingBehavior: Ignoring checkpoint, plan changed");
        current_mowing_plan_digest_ = mowing_plan_digest;
        current_mowing_path_ = 0;
        current_mowing_path_index_ = 0;
    }

    return true;
}

int MowingBehavior::get_current_mow_path_index() {
    if (!path_progress_client_->wait_for_service(1s)) {
        return -1;
    }

    auto request = std::make_shared<ftc_local_planner_msgs::srv::PlannerGetProgress::Request>();
    auto future = path_progress_client_->async_send_request(request);
    
    if (future.wait_for(2s) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), 
            "MowingBehavior: Error getting progress from planner");
        return -1;
    }

    auto response = future.get();
    return response ? response->index : -1;
}

bool MowingBehavior::execute_mowing_plan() {
    int first_point_attempt_counter = 0;
    int first_point_trim_counter = 0;
    rclcpp::Time paused_time;

    // Wait for Nav2 action server
    if (!nav_to_pose_client_->wait_for_action_server(10s)) {
        RCLCPP_ERROR(node_->get_logger(), "MowingBehavior: Nav2 navigate_to_pose action server not available");
        return false;
    }
    if (!follow_path_client_->wait_for_action_server(10s)) {
        RCLCPP_ERROR(node_->get_logger(), "MowingBehavior: Nav2 follow_path action server not available");
        return false;
    }

    while (static_cast<size_t>(current_mowing_path_) < current_mowing_paths_.size() && 
           rclcpp::ok() && !aborted_.load()) {
        
        // PAUSE HANDLING
        if (requested_pause_flag_.load()) {
            paused_.store(true);
            mower_enabled_.store(false);
            uint8_t last_pause_flags = 0;
            
            while (requested_pause_flag_.load() && !aborted_.load()) {
                if (last_pause_flags != requested_pause_flag_.load()) {
                    update_actions();
                }
                last_pause_flags = requested_pause_flag_.load();
                
                std::string pause_reason;
                if (requested_pause_flag_.load() & PAUSE_EMERGENCY) {
                    pause_reason += "on EMERGENCY";
                    if (requested_pause_flag_.load() & PAUSE_MANUAL) {
                        pause_reason += " and ";
                    }
                }
                if (requested_pause_flag_.load() & PAUSE_MANUAL) {
                    pause_reason += "waiting for CONTINUE";
                }
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 30000,
                    "MowingBehavior: PAUSED (%s)", pause_reason.c_str());
                std::this_thread::sleep_for(1s);
            }
        }

        if (paused_.load()) {
            paused_time = node_->now();
            while (!hasGoodGPS() && !aborted_.load()) {
                RCLCPP_INFO(node_->get_logger(), "MowingBehavior: PAUSED (%.1fs) (waiting for GPS)",
                    (node_->now() - paused_time).seconds());
                std::this_thread::sleep_for(1s);
            }
            RCLCPP_INFO(node_->get_logger(), "MowingBehavior: CONTINUING");
            paused_.store(false);
            update_actions();
        }

        auto& path = current_mowing_paths_[current_mowing_path_];
        RCLCPP_INFO(node_->get_logger(), "MowingBehavior: Path segment length: %zu poses",
            path.path.poses.size());

        // Check if path is empty
        if (static_cast<size_t>(current_mowing_path_index_) >= path.path.poses.size()) {
            RCLCPP_INFO(node_->get_logger(), "MowingBehavior: Skipping empty path");
            current_mowing_path_++;
            current_mowing_path_index_ = 0;
            continue;
        }

        // DRIVE TO FIRST POINT
        {
            RCLCPP_INFO(node_->get_logger(), "MowingBehavior: (FIRST POINT) Moving to path segment start");

            if (path.is_outline && mow_config_.add_fake_obstacle) {
                if (set_nav_point_client_->wait_for_service(2s)) {
                    auto nav_request = std::make_shared<mower_map_msgs::srv::SetNavPointSrv::Request>();
                    nav_request->nav_pose = path.path.poses[current_mowing_path_index_].pose;
                    set_nav_point_client_->async_send_request(nav_request);
                    std::this_thread::sleep_for(1s);
                }
            }

            // Navigate to start point using Nav2
            auto goal_msg = NavigateToPose::Goal();
            goal_msg.pose = path.path.poses[current_mowing_path_index_];
            goal_msg.pose.header.frame_id = "map";
            goal_msg.pose.header.stamp = node_->now();

            auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
            send_goal_options.result_callback = [](const GoalHandleNavigate::WrappedResult&) {};

            auto goal_future = nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
            
            if (goal_future.wait_for(5s) != std::future_status::ready) {
                RCLCPP_ERROR(node_->get_logger(), "MowingBehavior: Failed to send navigation goal");
                first_point_attempt_counter++;
                if (first_point_attempt_counter < mow_config_.max_first_point_attempts) {
                    paused_.store(true);
                    update_actions();
                    continue;
                }
            }

            auto goal_handle = goal_future.get();
            if (!goal_handle) {
                RCLCPP_ERROR(node_->get_logger(), "MowingBehavior: Goal rejected");
                first_point_attempt_counter++;
                paused_.store(true);
                continue;
            }

            // Wait for result
            bool navigation_succeeded = false;
            while (rclcpp::ok()) {
                auto status = goal_handle->get_status();
                
                if (status == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                    navigation_succeeded = true;
                    break;
                } else if (status == rclcpp_action::GoalStatus::STATUS_ABORTED ||
                           status == rclcpp_action::GoalStatus::STATUS_CANCELED) {
                    break;
                }

                // Check for skip/abort/pause
                if (skip_area_.load()) {
                    RCLCPP_INFO(node_->get_logger(), "MowingBehavior: (FIRST POINT) SKIP AREA requested");
                    mower_enabled_.store(false);
                    nav_to_pose_client_->async_cancel_all_goals();
                    current_mowing_paths_.clear();
                    skip_area_.store(false);
                    return true;
                }
                if (skip_path_.load()) {
                    skip_path_.store(false);
                    nav_to_pose_client_->async_cancel_all_goals();
                    current_mowing_path_++;
                    current_mowing_path_index_ = 0;
                    break;
                }
                if (aborted_.load()) {
                    nav_to_pose_client_->async_cancel_all_goals();
                    mower_enabled_.store(false);
                    return false;
                }
                if (requested_pause_flag_.load()) {
                    nav_to_pose_client_->async_cancel_all_goals();
                    mower_enabled_.store(false);
                    break;
                }

                std::this_thread::sleep_for(100ms);
            }

            if (!navigation_succeeded && !requested_pause_flag_.load() && !skip_path_.load()) {
                first_point_attempt_counter++;
                if (first_point_attempt_counter < mow_config_.max_first_point_attempts) {
                    RCLCPP_WARN(node_->get_logger(), 
                        "MowingBehavior: (FIRST POINT) Attempt %d/%d failed",
                        first_point_attempt_counter, mow_config_.max_first_point_attempts);
                    paused_.store(true);
                    update_actions();
                } else if (first_point_trim_counter < mow_config_.max_first_point_trim_attempts) {
                    RCLCPP_WARN(node_->get_logger(),
                        "MowingBehavior: (FIRST POINT) Trimming start point, attempt %d/%d",
                        first_point_trim_counter, mow_config_.max_first_point_trim_attempts);
                    current_mowing_path_index_++;
                    first_point_trim_counter++;
                    first_point_attempt_counter = 0;
                    paused_.store(true);
                    update_actions();
                } else {
                    RCLCPP_ERROR(node_->get_logger(),
                        "MowingBehavior: Max retries reached, aborting");
                    abort();
                }
                continue;
            }

            // Clear nav point
            if (clear_nav_point_client_->wait_for_service(1s)) {
                auto clear_request = std::make_shared<mower_map_msgs::srv::ClearNavPointSrv::Request>();
                clear_nav_point_client_->async_send_request(clear_request);
            }

            first_point_attempt_counter = 0;
            first_point_trim_counter = 0;
        }

        // EXECUTE MOW PATH
        {
            mower_enabled_.store(true);

            auto follow_goal = FollowPath::Goal();
            nav_msgs::msg::Path exe_path;
            exe_path.header = path.path.header;
            exe_path.header.frame_id = "map";
            exe_path.header.stamp = node_->now();
            
            for (size_t i = current_mowing_path_index_; i < path.path.poses.size(); ++i) {
                exe_path.poses.push_back(path.path.poses[i]);
            }
            
            int exe_path_start_index = current_mowing_path_index_;
            follow_goal.path = exe_path;
            follow_goal.controller_id = "FTCPlanner";

            RCLCPP_INFO(node_->get_logger(),
                "MowingBehavior: (MOW) Executing path with %zu poses from index %d",
                path.path.poses.size(), exe_path_start_index);

            auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
            auto goal_future = follow_path_client_->async_send_goal(follow_goal, send_goal_options);
            
            if (goal_future.wait_for(5s) != std::future_status::ready) {
                RCLCPP_ERROR(node_->get_logger(), "MowingBehavior: Failed to send follow path goal");
                mower_enabled_.store(false);
                paused_.store(true);
                continue;
            }

            auto goal_handle = goal_future.get();
            if (!goal_handle) {
                RCLCPP_ERROR(node_->get_logger(), "MowingBehavior: Follow path goal rejected");
                mower_enabled_.store(false);
                paused_.store(true);
                continue;
            }

            // Wait for result
            bool path_succeeded = false;
            while (rclcpp::ok()) {
                auto status = goal_handle->get_status();

                if (status == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                    path_succeeded = true;
                    break;
                } else if (status == rclcpp_action::GoalStatus::STATUS_ABORTED ||
                           status == rclcpp_action::GoalStatus::STATUS_CANCELED) {
                    break;
                }

                // Check for skip/abort/pause
                if (skip_area_.load()) {
                    RCLCPP_INFO(node_->get_logger(), "MowingBehavior: (MOW) SKIP AREA requested");
                    mower_enabled_.store(false);
                    follow_path_client_->async_cancel_all_goals();
                    current_mowing_paths_.clear();
                    skip_area_.store(false);
                    return true;
                }
                if (skip_path_.load()) {
                    skip_path_.store(false);
                    follow_path_client_->async_cancel_all_goals();
                    current_mowing_path_++;
                    current_mowing_path_index_ = 0;
                    break;
                }
                if (aborted_.load()) {
                    follow_path_client_->async_cancel_all_goals();
                    mower_enabled_.store(false);
                    break;
                }
                if (requested_pause_flag_.load()) {
                    follow_path_client_->async_cancel_all_goals();
                    mower_enabled_.store(false);
                    break;
                }

                // Update progress
                if (status == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
                    int current_index = get_current_mow_path_index();
                    if (current_index >= 0) {
                        current_mowing_path_index_ = exe_path_start_index + current_index;
                    }
                    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                        "MowingBehavior: (MOW) Progress: %d/%zu",
                        current_mowing_path_index_, path.path.poses.size());
                    
                    if ((node_->now() - last_checkpoint_).seconds() > 30.0) {
                        checkpoint();
                    }
                }

                std::this_thread::sleep_for(100ms);
            }

            if (path_succeeded || 
                current_mowing_path_index_ >= static_cast<int>(path.path.poses.size()) ||
                (path.path.poses.size() - current_mowing_path_index_) < 5) {
                RCLCPP_INFO(node_->get_logger(), "MowingBehavior: (MOW) Path finished, moving to next");
                current_mowing_path_++;
                current_mowing_path_index_ = 0;
            } else if (!requested_pause_flag_.load()) {
                if (current_mowing_path_index_ == 0) {
                    current_mowing_path_index_++;
                }
                RCLCPP_INFO(node_->get_logger(), 
                    "MowingBehavior: (MOW) PAUSED at index %d", current_mowing_path_index_);
                paused_.store(true);
                update_actions();
            }
        }
    }

    mower_enabled_.store(false);
    return static_cast<size_t>(current_mowing_path_) >= current_mowing_paths_.size();
}

void MowingBehavior::update_actions() {
    for (auto& a : actions_) {
        a.enabled = true;
    }

    // pause / resume switch
    actions_[0].enabled = !(requested_pause_flag_.load() & PAUSE_MANUAL);
    actions_[1].enabled = requested_pause_flag_.load() & PAUSE_MANUAL;

    // Would register actions via service
}

void MowingBehavior::command_home() {
    if (shared_state_->active_semiautomatic_task) {
        RCLCPP_INFO(node_->get_logger(), "Manually pausing semiautomatic task");
        // Would update config.manual_pause_mowing = true
    }
    if (paused_.load()) {
        requestContinue();
    }
    abort();
}

void MowingBehavior::command_start() {
    RCLCPP_INFO(node_->get_logger(), "MowingBehavior: MANUAL CONTINUE");
    requestContinue();
}

void MowingBehavior::command_s1() {
    RCLCPP_INFO(node_->get_logger(), "MowingBehavior: MANUAL PAUSED");
    requestPause();
}

void MowingBehavior::command_s2() {
    skip_area_.store(true);
}

uint8_t MowingBehavior::get_sub_state() {
    return 0;
}

uint8_t MowingBehavior::get_state() {
    return mower_msgs::msg::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

void MowingBehavior::handle_action(const std::string& action) {
    if (action == "mower_logic:mowing/pause") {
        RCLCPP_INFO(node_->get_logger(), "Got pause command");
        requestPause();
    } else if (action == "mower_logic:mowing/continue") {
        RCLCPP_INFO(node_->get_logger(), "Got continue command");
        requestContinue();
    } else if (action == "mower_logic:mowing/abort_mowing") {
        RCLCPP_INFO(node_->get_logger(), "Got abort mowing command");
        command_home();
    } else if (action == "mower_logic:mowing/skip_area") {
        RCLCPP_INFO(node_->get_logger(), "Got skip_area command");
        skip_area_.store(true);
    } else if (action == "mower_logic:mowing/skip_path") {
        RCLCPP_INFO(node_->get_logger(), "Got skip_path command");
        skip_path_.store(true);
    }
    update_actions();
}

void MowingBehavior::checkpoint() {
    // Simple checkpoint using JSON file
    std::ofstream file("checkpoint.json");
    if (file.is_open()) {
        file << "{\n";
        file << "  \"currentMowingPath\": " << current_mowing_path_ << ",\n";
        file << "  \"currentMowingArea\": " << current_mowing_area_ << ",\n";
        file << "  \"currentMowingPathIndex\": " << current_mowing_path_index_ << ",\n";
        file << "  \"currentMowingPlanDigest\": \"" << current_mowing_plan_digest_ << "\",\n";
        file << "  \"currentMowingAngleIncrementSum\": " << current_mowing_angle_increment_sum_ << "\n";
        file << "}\n";
        file.close();
    }
    last_checkpoint_ = node_->now();
}

bool MowingBehavior::restore_checkpoint() {
    std::ifstream file("checkpoint.json");
    if (!file.is_open()) {
        current_mowing_area_ = 0;
        current_mowing_path_ = 0;
        current_mowing_path_index_ = 0;
        current_mowing_angle_increment_sum_ = 0.0;
        return false;
    }

    // Simple JSON parsing - would use nlohmann/json in production
    std::string content((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());
    file.close();

    // Basic extraction - this is simplified
    auto extract_int = [&content](const std::string& key) -> int {
        auto pos = content.find("\"" + key + "\":");
        if (pos != std::string::npos) {
            pos = content.find(":", pos) + 1;
            return std::stoi(content.substr(pos));
        }
        return 0;
    };

    auto extract_double = [&content](const std::string& key) -> double {
        auto pos = content.find("\"" + key + "\":");
        if (pos != std::string::npos) {
            pos = content.find(":", pos) + 1;
            return std::stod(content.substr(pos));
        }
        return 0.0;
    };

    auto extract_string = [&content](const std::string& key) -> std::string {
        auto pos = content.find("\"" + key + "\": \"");
        if (pos != std::string::npos) {
            pos = content.find(": \"", pos) + 3;
            auto end = content.find("\"", pos);
            return content.substr(pos, end - pos);
        }
        return "";
    };

    current_mowing_path_ = extract_int("currentMowingPath");
    current_mowing_area_ = extract_int("currentMowingArea");
    current_mowing_path_index_ = extract_int("currentMowingPathIndex");
    current_mowing_plan_digest_ = extract_string("currentMowingPlanDigest");
    current_mowing_angle_increment_sum_ = extract_double("currentMowingAngleIncrementSum");

    return true;
}

}  // namespace mower_logic
