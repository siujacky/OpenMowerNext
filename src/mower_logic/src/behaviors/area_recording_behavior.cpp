// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: GPL-3.0
//
// ROS2 port of AreaRecordingBehavior

#include "mower_logic/behaviors/area_recording_behavior.hpp"
#include "mower_logic/behaviors/idle_behavior.hpp"
#include "mower_msgs/msg/high_level_status.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace mower_logic {

AreaRecordingBehavior AreaRecordingBehavior::INSTANCE;

AreaRecordingBehavior::AreaRecordingBehavior() {
    xbot_msgs::msg::ActionInfo start_recording_action;
    start_recording_action.action_id = "start_recording";
    start_recording_action.enabled = false;
    start_recording_action.action_name = "Start Recording";
    actions_.push_back(start_recording_action);

    xbot_msgs::msg::ActionInfo stop_recording_action;
    stop_recording_action.action_id = "stop_recording";
    stop_recording_action.enabled = false;
    stop_recording_action.action_name = "Stop Recording";
    actions_.push_back(stop_recording_action);

    xbot_msgs::msg::ActionInfo finish_navigation_area_action;
    finish_navigation_area_action.action_id = "finish_navigation_area";
    finish_navigation_area_action.enabled = false;
    finish_navigation_area_action.action_name = "Save Navigation Area";
    actions_.push_back(finish_navigation_area_action);

    xbot_msgs::msg::ActionInfo finish_mowing_area_action;
    finish_mowing_area_action.action_id = "finish_mowing_area";
    finish_mowing_area_action.enabled = false;
    finish_mowing_area_action.action_name = "Save Mowing Area";
    actions_.push_back(finish_mowing_area_action);

    xbot_msgs::msg::ActionInfo exit_recording_mode_action;
    exit_recording_mode_action.action_id = "exit_recording_mode";
    exit_recording_mode_action.enabled = false;
    exit_recording_mode_action.action_name = "Exit";
    actions_.push_back(exit_recording_mode_action);

    xbot_msgs::msg::ActionInfo finish_discard_action;
    finish_discard_action.action_id = "finish_discard";
    finish_discard_action.enabled = false;
    finish_discard_action.action_name = "Discard Area";
    actions_.push_back(finish_discard_action);

    xbot_msgs::msg::ActionInfo record_dock_action;
    record_dock_action.action_id = "record_dock";
    record_dock_action.enabled = false;
    record_dock_action.action_name = "Record Docking point";
    actions_.push_back(record_dock_action);

    xbot_msgs::msg::ActionInfo auto_point_enable;
    auto_point_enable.action_id = "auto_point_collecting_enable";
    auto_point_enable.enabled = false;
    auto_point_enable.action_name = "Enable automatic point collecting";
    actions_.push_back(auto_point_enable);

    xbot_msgs::msg::ActionInfo auto_point_disable;
    auto_point_disable.action_id = "auto_point_collecting_disable";
    auto_point_disable.enabled = false;
    auto_point_disable.action_name = "Disable automatic point collecting";
    actions_.push_back(auto_point_disable);

    xbot_msgs::msg::ActionInfo collect_point_action;
    collect_point_action.action_id = "collect_point";
    collect_point_action.enabled = false;
    collect_point_action.action_name = "Collect point";
    actions_.push_back(collect_point_action);

    xbot_msgs::msg::ActionInfo start_manual_mowing_action;
    start_manual_mowing_action.action_id = "start_manual_mowing";
    start_manual_mowing_action.enabled = false;
    start_manual_mowing_action.action_name = "Start manual mowing";
    actions_.push_back(start_manual_mowing_action);

    xbot_msgs::msg::ActionInfo stop_manual_mowing_action;
    stop_manual_mowing_action.action_id = "stop_manual_mowing";
    stop_manual_mowing_action.enabled = false;
    stop_manual_mowing_action.action_name = "Stop manual mowing";
    actions_.push_back(stop_manual_mowing_action);
}

std::string AreaRecordingBehavior::state_name() {
    return "AREA_RECORDING";
}

std::string AreaRecordingBehavior::sub_state_name() {
    if (has_first_docking_pos_.load()) {
        return "RECORD_DOCKING_POSITION";
    }
    switch (sub_state_.load()) {
        case 0: return "";
        case 1: return "RECORD_OUTLINE";
        case 2: return "RECORD_OBSTACLE";
        default: return "";
    }
}

void AreaRecordingBehavior::enter() {
    has_outline_.store(false);
    is_mowing_area_.store(false);
    is_navigation_area_.store(false);
    manual_mowing_.store(false);
    has_first_docking_pos_.store(false);
    has_odom_.store(false);
    poly_recording_enabled_.store(false);
    finished_all_.store(false);
    set_docking_position_.store(false);
    paused_.store(false);
    aborted_.store(false);
    markers_ = visualization_msgs::msg::MarkerArray();

    update_actions();

    // Create service clients
    add_mowing_area_client_ = node_->create_client<mower_map_msgs::srv::AddMowingAreaSrv>(
        "mower_map_service/add_mowing_area");
    set_docking_point_client_ = node_->create_client<mower_map_msgs::srv::SetDockingPointSrv>(
        "mower_map_service/set_docking_point");

    // Create publishers
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "area_recorder/progress_visualization", 10);
    map_overlay_pub_ = node_->create_publisher<xbot_msgs::msg::MapOverlay>(
        "xbot_monitoring/map_overlay", 10);
    marker_array_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "area_recorder/progress_visualization_array", 10);

    RCLCPP_INFO(node_->get_logger(), "Starting recording area");
    RCLCPP_INFO(node_->get_logger(), "Subscribing to /joy for user input");

    // Create subscribers
    joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 100, std::bind(&AreaRecordingBehavior::joy_received, this, _1));

    dock_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/record_dock", 100, std::bind(&AreaRecordingBehavior::record_dock_received, this, _1));
    polygon_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/record_polygon", 100, std::bind(&AreaRecordingBehavior::record_polygon_received, this, _1));
    mow_area_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/record_mowing", 100, std::bind(&AreaRecordingBehavior::record_mowing_received, this, _1));
    nav_area_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/record_navigation", 100, std::bind(&AreaRecordingBehavior::record_navigation_received, this, _1));
    auto_point_collecting_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/record_auto_point_collecting", 100, 
        std::bind(&AreaRecordingBehavior::record_auto_point_collecting, this, _1));
    collect_point_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/record_collect_point", 100, std::bind(&AreaRecordingBehavior::record_collect_point, this, _1));

    pose_sub_ = node_->create_subscription<xbot_msgs::msg::AbsolutePose>(
        "/xbot_positioning/xb_pose", 100, std::bind(&AreaRecordingBehavior::pose_received, this, _1));
}

void AreaRecordingBehavior::exit() {
    for (auto& a : actions_) {
        a.enabled = false;
    }

    // Shutdown subscribers and publishers
    map_overlay_pub_.reset();
    marker_pub_.reset();
    marker_array_pub_.reset();
    joy_sub_.reset();
    dock_sub_.reset();
    polygon_sub_.reset();
    mow_area_sub_.reset();
    nav_area_sub_.reset();
    auto_point_collecting_sub_.reset();
    collect_point_sub_.reset();
    pose_sub_.reset();
    add_mowing_area_client_.reset();
    set_docking_point_client_.reset();
}

void AreaRecordingBehavior::reset() {
}

bool AreaRecordingBehavior::needs_gps() {
    return false;
}

bool AreaRecordingBehavior::mower_enabled() {
    return manual_mowing_.load();
}

bool AreaRecordingBehavior::redirect_joystick() {
    return true;
}

void AreaRecordingBehavior::pose_received(const xbot_msgs::msg::AbsolutePose::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    last_pose_ = *msg;
    has_odom_.store(true);
}

void AreaRecordingBehavior::joy_received(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // B was pressed - toggle recording state
    if (!msg->buttons.empty() && msg->buttons.size() > 1 && 
        msg->buttons[1] && !last_joy_.buttons.empty() && !last_joy_.buttons[1]) {
        RCLCPP_INFO(node_->get_logger(), "B PRESSED");
        poly_recording_enabled_.store(!poly_recording_enabled_.load());
    }

    // Y + up - record navigation area
    if (!msg->buttons.empty() && msg->buttons.size() > 3 && !msg->axes.empty() && msg->axes.size() > 7 &&
        msg->buttons[3] && msg->axes[7] > 0.5) {
        RCLCPP_INFO(node_->get_logger(), "Y + UP PRESSED, recording navigation area");
        poly_recording_enabled_.store(false);
        is_mowing_area_.store(false);
        is_navigation_area_.store(true);
        finished_all_.store(true);
    }

    // Y + down - record mowing area
    if (!msg->buttons.empty() && msg->buttons.size() > 3 && !msg->axes.empty() && msg->axes.size() > 7 &&
        msg->buttons[3] && msg->axes[7] < -0.5) {
        RCLCPP_INFO(node_->get_logger(), "Y + DOWN PRESSED, recording mowing area");
        poly_recording_enabled_.store(false);
        is_mowing_area_.store(true);
        is_navigation_area_.store(false);
        finished_all_.store(true);
    }

    // X pressed - set base position
    if (!msg->buttons.empty() && msg->buttons.size() > 2 &&
        msg->buttons[2] && !last_joy_.buttons.empty() && !last_joy_.buttons[2]) {
        RCLCPP_INFO(node_->get_logger(), "X PRESSED");
        set_docking_position_.store(true);
    }

    // RB for manual point collecting, LB+RB for auto toggle
    if (!msg->buttons.empty() && msg->buttons.size() > 5 &&
        msg->buttons[5] && !last_joy_.buttons.empty() && !last_joy_.buttons[5]) {
        if (msg->buttons.size() > 4 && msg->buttons[4]) {
            RCLCPP_INFO(node_->get_logger(), "LB+RB PRESSED, toggle auto point collecting");
            auto_point_collecting_.store(!auto_point_collecting_.load());
            RCLCPP_INFO(node_->get_logger(), "Auto point collecting: %s",
                auto_point_collecting_.load() ? "true" : "false");
        } else {
            RCLCPP_INFO(node_->get_logger(), "RB PRESSED, collect point");
            collect_point_.store(true);
        }
    }

    last_joy_ = *msg;
}

void AreaRecordingBehavior::record_dock_received(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        RCLCPP_INFO(node_->get_logger(), "Record dock position");
        set_docking_position_.store(true);
    }
}

void AreaRecordingBehavior::record_polygon_received(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        RCLCPP_INFO(node_->get_logger(), "Toggle record polygon");
        poly_recording_enabled_.store(!poly_recording_enabled_.load());
    }
}

void AreaRecordingBehavior::record_mowing_received(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        RCLCPP_INFO(node_->get_logger(), "Save polygon as mowing area");
        poly_recording_enabled_.store(false);
        is_mowing_area_.store(true);
        is_navigation_area_.store(false);
        finished_all_.store(true);
    }
}

void AreaRecordingBehavior::record_navigation_received(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        RCLCPP_INFO(node_->get_logger(), "Save polygon as navigation area");
        poly_recording_enabled_.store(false);
        is_mowing_area_.store(false);
        is_navigation_area_.store(true);
        finished_all_.store(true);
    }
}

void AreaRecordingBehavior::record_auto_point_collecting(const std_msgs::msg::Bool::SharedPtr msg) {
    auto_point_collecting_.store(msg->data);
    RCLCPP_INFO(node_->get_logger(), "Recording auto point collecting: %s",
        msg->data ? "enabled" : "disabled");
}

void AreaRecordingBehavior::record_collect_point(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        RCLCPP_INFO(node_->get_logger(), "Recording collect point");
        collect_point_.store(true);
    }
}

bool AreaRecordingBehavior::get_docking_position(geometry_msgs::msg::Pose& pos) {
    if (!has_first_docking_pos_.load()) {
        RCLCPP_INFO(node_->get_logger(), "Recording first docking position");

        // Wait for pose
        rclcpp::Rate rate(10);
        auto start = node_->now();
        while (!has_odom_.load() && (node_->now() - start).seconds() < 1.0) {
            rate.sleep();
        }

        if (has_odom_.load()) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            first_docking_pos_ = last_pose_.pose.pose;
        }
        has_first_docking_pos_.store(true);
        update_actions();
        return false;
    } else {
        RCLCPP_INFO(node_->get_logger(), "Recording second docking position");

        // Wait for pose
        rclcpp::Rate rate(10);
        auto start = node_->now();
        while (!has_odom_.load() && (node_->now() - start).seconds() < 1.0) {
            rate.sleep();
        }

        if (has_odom_.load()) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            pos.position = last_pose_.pose.pose.position;
        }

        double yaw = std::atan2(
            pos.position.y - first_docking_pos_.position.y,
            pos.position.x - first_docking_pos_.position.x);
        
        tf2::Quaternion docking_orientation;
        docking_orientation.setRPY(0.0, 0.0, yaw);
        pos.orientation = tf2::toMsg(docking_orientation);

        update_actions();
        return true;
    }
}

bool AreaRecordingBehavior::record_new_polygon(geometry_msgs::msg::Polygon& polygon,
                                                xbot_msgs::msg::MapOverlay& result_overlay) {
    RCLCPP_INFO(node_->get_logger(), "recordNewPolygon");

    bool success = true;
    marker_ = visualization_msgs::msg::Marker();
    marker_.header.frame_id = "map";
    marker_.ns = "area_recorder";
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker_.action = 0;
    marker_.pose.orientation.w = 1.0f;
    marker_.scale.x = 0.05;
    marker_.scale.y = 0.05;
    marker_.scale.z = 0.05;
    marker_.frame_locked = true;
    marker_.color.b = 1.0f;
    marker_.color.a = 1.0f;

    rclcpp::Rate update_rate(10);
    has_odom_.store(false);

    // Push a new poly to the visualization overlay
    xbot_msgs::msg::MapOverlayPolygon poly_viz;
    poly_viz.closed = false;
    poly_viz.line_width = 0.1;
    poly_viz.color = "blue";
    result_overlay.polygons.push_back(poly_viz);
    auto& current_poly_viz = result_overlay.polygons.back();

    while (true) {
        if (!rclcpp::ok() || aborted_.load()) {
            RCLCPP_WARN(node_->get_logger(), "Preempting Area Recorder");
            success = false;
            break;
        }

        update_rate.sleep();

        if (!has_odom_.load()) continue;

        geometry_msgs::msg::Pose pose_in_map;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            pose_in_map = last_pose_.pose.pose;
        }

        if (polygon.points.empty()) {
            // Add the first point
            geometry_msgs::msg::Point32 pt;
            pt.x = pose_in_map.position.x;
            pt.y = pose_in_map.position.y;
            pt.z = 0.0;

            polygon.points.push_back(pt);
            
            geometry_msgs::msg::Point vpt;
            vpt.x = pt.x;
            vpt.y = pt.y;
            marker_.points.push_back(vpt);

            marker_.header.stamp = node_->now();
            marker_pub_->publish(marker_);

            polygon.points.push_back(pt);
            current_poly_viz.polygon.points.push_back(pt);
            map_overlay_pub_->publish(result_overlay);
        } else {
            auto last = polygon.points.back();
            tf2::Vector3 last_point(last.x, last.y, 0.0);
            tf2::Vector3 current_point(pose_in_map.position.x, pose_in_map.position.y, 0.0);

            bool is_new_point_far_enough = (current_point - last_point).length() > NEW_POINT_MIN_DISTANCE;
            bool is_point_auto_collected = auto_point_collecting_.load() && is_new_point_far_enough;
            bool is_point_manual_collected = !auto_point_collecting_.load() && 
                                              collect_point_.load() && is_new_point_far_enough;

            if (is_point_auto_collected || is_point_manual_collected) {
                geometry_msgs::msg::Point32 pt;
                pt.x = pose_in_map.position.x;
                pt.y = pose_in_map.position.y;
                pt.z = 0.0;

                polygon.points.push_back(pt);
                
                geometry_msgs::msg::Point vpt;
                vpt.x = pt.x;
                vpt.y = pt.y;
                marker_.points.push_back(vpt);

                marker_.header.stamp = node_->now();
                marker_pub_->publish(marker_);

                current_poly_viz.polygon.points.push_back(pt);
                map_overlay_pub_->publish(result_overlay);

                if (is_point_manual_collected) {
                    collect_point_.store(false);
                }
            }
        }

        if (!poly_recording_enabled_.load()) {
            if (polygon.points.size() > 2) {
                // Add first point to close the poly
                polygon.points.push_back(polygon.points.front());
            } else {
                success = false;
            }
            RCLCPP_INFO(node_->get_logger(), "Finished Recording polygon");
            break;
        }
    }

    marker_.action = visualization_msgs::msg::Marker::DELETE;
    marker_pub_->publish(marker_);

    // Close poly
    current_poly_viz.closed = true;
    current_poly_viz.line_width = 0.05;
    if (result_overlay.polygons.size() == 1) {
        current_poly_viz.color = "green";
    } else {
        current_poly_viz.color = "red";
    }
    map_overlay_pub_->publish(result_overlay);

    return success;
}

Behavior* AreaRecordingBehavior::execute() {
    // Enable GPS - would call setGPS(true)
    
    bool error = false;
    rclcpp::Rate input_delay(10);

    while (rclcpp::ok() && !aborted_.load()) {
        mower_map_msgs::msg::MapArea result;
        xbot_msgs::msg::MapOverlay result_overlay;

        // Clear overlay
        map_overlay_pub_->publish(result_overlay);

        has_outline_.store(false);
        sub_state_.store(0);

        while (rclcpp::ok() && !finished_all_.load() && !error && !aborted_.load()) {
            if (set_docking_position_.load()) {
                geometry_msgs::msg::Pose pos;
                if (get_docking_position(pos)) {
                    RCLCPP_INFO(node_->get_logger(), "New docking pos = (%.2f, %.2f)",
                        pos.position.x, pos.position.y);

                    if (set_docking_point_client_->wait_for_service(2s)) {
                        auto request = std::make_shared<mower_map_msgs::srv::SetDockingPointSrv::Request>();
                        request->docking_pose = pos;
                        set_docking_point_client_->async_send_request(request);
                    }

                    has_first_docking_pos_.store(false);
                    update_actions();
                }
                set_docking_position_.store(false);
            }

            if (poly_recording_enabled_.load()) {
                update_actions();
                geometry_msgs::msg::Polygon poly;

                if (has_outline_.load()) {
                    sub_state_.store(1);
                } else {
                    sub_state_.store(2);
                }

                bool success = record_new_polygon(poly, result_overlay);
                sub_state_.store(0);

                if (success) {
                    if (!has_outline_.load()) {
                        has_outline_.store(true);
                        result.area = poly;

                        marker_.color.r = 0.0f;
                        marker_.color.g = 1.0f;
                        marker_.color.b = 0.0f;
                        marker_.color.a = 1.0f;
                        marker_.id = markers_.markers.size() + 1;
                        marker_.action = visualization_msgs::msg::Marker::ADD;
                        markers_.markers.push_back(marker_);
                    } else {
                        result.obstacles.push_back(poly);

                        marker_.color.r = 1.0f;
                        marker_.color.g = 0.0f;
                        marker_.color.b = 0.0f;
                        marker_.color.a = 1.0f;
                        marker_.action = visualization_msgs::msg::Marker::ADD;
                        marker_.id = markers_.markers.size() + 1;
                        markers_.markers.push_back(marker_);
                    }
                } else {
                    error = true;
                    RCLCPP_ERROR(node_->get_logger(), "Error during poly record");
                }
                marker_array_pub_->publish(markers_);
                update_actions();
            }

            input_delay.sleep();
        }

        if (!error && has_outline_.load() && (is_mowing_area_.load() || is_navigation_area_.load())) {
            if (is_mowing_area_.load()) {
                RCLCPP_INFO(node_->get_logger(), "Area recording completed. Adding mowing area.");
            } else {
                RCLCPP_INFO(node_->get_logger(), "Area recording completed. Adding navigation area.");
            }

            if (add_mowing_area_client_->wait_for_service(5s)) {
                auto request = std::make_shared<mower_map_msgs::srv::AddMowingAreaSrv::Request>();
                request->is_navigation_area = !is_mowing_area_.load();
                request->area = result;

                auto future = add_mowing_area_client_->async_send_request(request);
                if (future.wait_for(5s) == std::future_status::ready) {
                    RCLCPP_INFO(node_->get_logger(), "Area added successfully");
                } else {
                    RCLCPP_ERROR(node_->get_logger(), "Error adding area - timeout");
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Error adding area - service unavailable");
            }
        }

        // Reset for next area
        error = false;
        finished_all_.store(false);
        has_outline_.store(false);
        update_actions();
    }

    return &IdleBehavior::INSTANCE;
}

void AreaRecordingBehavior::update_actions() {
    for (auto& a : actions_) {
        a.enabled = false;
    }

    if (has_first_docking_pos_.load()) {
        // We have recorded the first docking pose, only option is to finish by recording second one
        actions_[6].enabled = true;  // record_dock
    } else if (poly_recording_enabled_.load()) {
        // Currently recording a polygon
        actions_[1].enabled = true;  // stop_recording
        actions_[2].enabled = true;  // finish_navigation_area
        actions_[3].enabled = true;  // finish_mowing_area
        actions_[4].enabled = true;  // exit_recording_mode
        actions_[5].enabled = true;  // finish_discard

        // Enable/disable auto point collecting
        actions_[7].enabled = !auto_point_collecting_.load();  // auto_point_enable
        actions_[8].enabled = auto_point_collecting_.load();   // auto_point_disable
        actions_[9].enabled = !auto_point_collecting_.load();  // collect_point
    } else {
        if (has_outline_.load()) {
            actions_[0].enabled = true;  // start_recording
            actions_[2].enabled = true;  // finish_navigation_area
            actions_[3].enabled = true;  // finish_mowing_area
            actions_[4].enabled = true;  // exit_recording_mode
            actions_[5].enabled = true;  // finish_discard
        } else {
            // Enable start recording, discard area and record dock
            actions_[0].enabled = true;  // start_recording
            actions_[4].enabled = true;  // exit_recording_mode
            actions_[6].enabled = true;  // record_dock
        }
    }

    // Manual mowing controls
    actions_[10].enabled = !manual_mowing_.load();  // start_manual_mowing
    actions_[11].enabled = manual_mowing_.load();   // stop_manual_mowing

    // Would register actions via service
}

void AreaRecordingBehavior::command_home() {
    abort();
}

void AreaRecordingBehavior::command_start() {
    // Not used
}

void AreaRecordingBehavior::command_s1() {
    // Not used
}

void AreaRecordingBehavior::command_s2() {
    // Not used
}

uint8_t AreaRecordingBehavior::get_sub_state() {
    return sub_state_.load();
}

uint8_t AreaRecordingBehavior::get_state() {
    return mower_msgs::msg::HighLevelStatus::HIGH_LEVEL_STATE_RECORDING;
}

void AreaRecordingBehavior::handle_action(const std::string& action) {
    if (action == "mower_logic:area_recording/start_recording") {
        RCLCPP_INFO(node_->get_logger(), "Got start recording");
        poly_recording_enabled_.store(true);
    } else if (action == "mower_logic:area_recording/stop_recording") {
        RCLCPP_INFO(node_->get_logger(), "Got stop recording");
        poly_recording_enabled_.store(false);
    } else if (action == "mower_logic:area_recording/finish_navigation_area") {
        RCLCPP_INFO(node_->get_logger(), "Got save navigation area");
        poly_recording_enabled_.store(false);
        is_mowing_area_.store(false);
        is_navigation_area_.store(true);
        finished_all_.store(true);
    } else if (action == "mower_logic:area_recording/finish_mowing_area") {
        RCLCPP_INFO(node_->get_logger(), "Got save mowing area");
        poly_recording_enabled_.store(false);
        is_mowing_area_.store(true);
        is_navigation_area_.store(false);
        finished_all_.store(true);
    } else if (action == "mower_logic:area_recording/finish_discard") {
        RCLCPP_INFO(node_->get_logger(), "Got discard recorded area");
        poly_recording_enabled_.store(false);
        is_mowing_area_.store(false);
        is_navigation_area_.store(false);
        finished_all_.store(true);
    } else if (action == "mower_logic:area_recording/exit_recording_mode") {
        RCLCPP_INFO(node_->get_logger(), "Got exit without saving");
        poly_recording_enabled_.store(false);
        is_mowing_area_.store(false);
        is_navigation_area_.store(false);
        finished_all_.store(true);
        abort();
    } else if (action == "mower_logic:area_recording/record_dock") {
        RCLCPP_INFO(node_->get_logger(), "Got record dock");
        set_docking_position_.store(true);
    } else if (action == "mower_logic:area_recording/auto_point_collecting_enable") {
        RCLCPP_INFO(node_->get_logger(), "Got enable auto point collecting");
        auto_point_collecting_.store(true);
    } else if (action == "mower_logic:area_recording/auto_point_collecting_disable") {
        RCLCPP_INFO(node_->get_logger(), "Got disable auto point collecting");
        auto_point_collecting_.store(false);
    } else if (action == "mower_logic:area_recording/collect_point") {
        RCLCPP_INFO(node_->get_logger(), "Got collect point");
        collect_point_.store(true);
    } else if (action == "mower_logic:area_recording/start_manual_mowing") {
        RCLCPP_INFO(node_->get_logger(), "Starting manual mowing");
        manual_mowing_.store(true);
    } else if (action == "mower_logic:area_recording/stop_manual_mowing") {
        RCLCPP_INFO(node_->get_logger(), "Stopping manual mowing");
        manual_mowing_.store(false);
    }
    update_actions();
}

}  // namespace mower_logic
