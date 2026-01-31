// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: GPL-3.0
//
// ROS2 port of IdleBehavior

#include "mower_logic/behaviors/idle_behavior.hpp"
#include "mower_logic/behaviors/mowing_behavior.hpp"
#include "mower_logic/behaviors/undocking_behavior.hpp"
#include "mower_logic/behaviors/area_recording_behavior.hpp"
#include "mower_msgs/msg/high_level_status.hpp"

namespace mower_logic {

IdleBehavior IdleBehavior::INSTANCE{false};
IdleBehavior IdleBehavior::DOCKED_INSTANCE{true};

IdleBehavior::IdleBehavior(bool stay_docked) : stay_docked_(stay_docked) {
    // Build actions list
    xbot_msgs::msg::ActionInfo start_mowing_action;
    start_mowing_action.action_id = "start_mowing";
    start_mowing_action.action_name = "Start Mowing";
    start_mowing_action.enabled = true;
    actions_.push_back(start_mowing_action);

    xbot_msgs::msg::ActionInfo start_area_recording_action;
    start_area_recording_action.action_id = "start_area_recording";
    start_area_recording_action.action_name = "Start Area Recording";
    start_area_recording_action.enabled = true;
    actions_.push_back(start_area_recording_action);
}

std::string IdleBehavior::state_name() {
    return stay_docked_ ? "IDLE (Docked)" : "IDLE";
}

void IdleBehavior::enter() {
    manual_start_mowing_ = false;
    start_area_recorder_ = false;
    // In ROS2, we would register actions via a service here
}

void IdleBehavior::exit() {
    // Cleanup
}

void IdleBehavior::reset() {
    manual_start_mowing_ = false;
    start_area_recorder_ = false;
}

Behavior* IdleBehavior::execute() {
    // Check for abort
    while (!aborted_.load()) {
        if (manual_start_mowing_) {
            RCLCPP_INFO(node_->get_logger(), "Starting mowing (manual trigger)");
            manual_start_mowing_ = false;
            // If docked, undock first; otherwise go straight to mowing
            if (stay_docked_) {
                return &UndockingBehavior::INSTANCE;
            } else {
                return &MowingBehavior::INSTANCE;
            }
        }

        if (start_area_recorder_) {
            RCLCPP_INFO(node_->get_logger(), "Starting area recording");
            start_area_recorder_ = false;
            return &AreaRecordingBehavior::INSTANCE;
        }

        // Sleep a bit
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return stay_docked_ ? &DOCKED_INSTANCE : &INSTANCE;
}

bool IdleBehavior::needs_gps() {
    return false;
}

bool IdleBehavior::mower_enabled() {
    return false;
}

bool IdleBehavior::redirect_joystick() {
    return true;  // Allow joystick control while idle
}

void IdleBehavior::command_home() {
    // Already idle/docked, nothing to do
}

void IdleBehavior::command_start() {
    manual_start_mowing_ = true;
}

void IdleBehavior::command_s1() {
    start_area_recorder_ = true;
}

void IdleBehavior::command_s2() {
    // Not used in idle
}

uint8_t IdleBehavior::get_sub_state() {
    return sub_state_.load();
}

uint8_t IdleBehavior::get_state() {
    return stay_docked_ ? 
           mower_msgs::msg::HighLevelStatus::HIGH_LEVEL_STATE_IDLE : 
           mower_msgs::msg::HighLevelStatus::HIGH_LEVEL_STATE_IDLE;
}

void IdleBehavior::handle_action(const std::string& action) {
    if (action == "mower_logic:idle/start_mowing") {
        manual_start_mowing_ = true;
    } else if (action == "mower_logic:idle/start_area_recording") {
        start_area_recorder_ = true;
    }
}

}  // namespace mower_logic
