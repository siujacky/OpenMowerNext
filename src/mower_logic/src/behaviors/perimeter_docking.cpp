// Copyright (c) 2023 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: GPL-3.0
//
// ROS2 port of PerimeterDocking behaviors

#include "mower_logic/behaviors/perimeter_docking.hpp"
#include "mower_logic/behaviors/idle_behavior.hpp"
#include "mower_logic/behaviors/mowing_behavior.hpp"
#include "mower_msgs/msg/high_level_status.hpp"

#include <cmath>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace mower_logic {

// Static member initialization
rclcpp::Subscription<mower_msgs::msg::Perimeter>::SharedPtr PerimeterBase::perimeter_sub_;
rclcpp::Client<mower_msgs::srv::PerimeterControlSrv>::SharedPtr PerimeterBase::perimeter_client_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr PerimeterBase::cmd_vel_pub_;
mower_msgs::msg::Perimeter PerimeterBase::last_perimeter_;
std::atomic<bool> PerimeterBase::perimeter_updated_{false};
int PerimeterBase::direction_ = 1;
double PerimeterBase::calibration_left_ = 1.0;
double PerimeterBase::calibration_right_ = 1.0;
double PerimeterBase::max_center_ = 1.0;
int PerimeterBase::sign_center_ = 1;

// Static instances
PerimeterSearchBehavior PerimeterSearchBehavior::INSTANCE;
PerimeterDockingBehavior PerimeterDockingBehavior::INSTANCE;
PerimeterUndockingBehavior PerimeterUndockingBehavior::INSTANCE;
PerimeterMoveToGpsBehavior PerimeterMoveToGpsBehavior::INSTANCE;

// Helper function for perimeter callback
static void perimeter_received(const mower_msgs::msg::Perimeter::SharedPtr msg) {
    PerimeterBase::last_perimeter_ = *msg;
    PerimeterBase::perimeter_updated_.store(true);
}

// PerimeterBase implementation
void PerimeterBase::enter() {
    paused_.store(false);
    aborted_.store(false);
}

void PerimeterBase::exit() {
}

void PerimeterBase::reset() {
}

bool PerimeterBase::needs_gps() {
    return false;
}

bool PerimeterBase::mower_enabled() {
    return false;  // No mower during docking
}

void PerimeterBase::command_home() {
}

void PerimeterBase::command_start() {
}

void PerimeterBase::command_s1() {
}

void PerimeterBase::command_s2() {
}

bool PerimeterBase::redirect_joystick() {
    return false;
}

uint8_t PerimeterBase::get_sub_state() {
    return 1;
}

uint8_t PerimeterBase::get_state() {
    return mower_msgs::msg::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

void PerimeterBase::handle_action(const std::string& /*action*/) {
}

int PerimeterBase::setup_connections() {
    // Create subscriber
    perimeter_sub_ = node_->create_subscription<mower_msgs::msg::Perimeter>(
        "/mower/perimeter", rclcpp::SensorDataQoS(), perimeter_received);

    // Create service client
    perimeter_client_ = node_->create_client<mower_msgs::srv::PerimeterControlSrv>(
        "/mower_service/perimeter_listen");

    // Create publisher
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/logic_vel", 1);

    // Load config
    perimeter_config_.perimeter_signal = node_->get_parameter_or("perimeter_signal", 0);
    perimeter_config_.docking_distance = node_->get_parameter_or("docking_distance", 20.0);
    perimeter_config_.undock_distance = node_->get_parameter_or("undock_distance", 2.0);

    // Activate perimeter listening
    if (!perimeter_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to connect to perimeter service");
        return 0;
    }

    direction_ = perimeter_config_.perimeter_signal > 0 ? 1 : -1;

    auto request = std::make_shared<mower_msgs::srv::PerimeterControlSrv::Request>();
    request->listen_on = direction_ * std::abs(perimeter_config_.perimeter_signal);

    auto future = perimeter_client_->async_send_request(request);
    if (future.wait_for(5s) == std::future_status::ready) {
        RCLCPP_INFO(node_->get_logger(), "Perimeter activated");
        return 1;
    }

    RCLCPP_ERROR(node_->get_logger(), "Failed to activate perimeter");
    return 0;
}

Behavior* PerimeterBase::shutdown_connections() {
    // Deactivate perimeter
    if (perimeter_client_) {
        auto request = std::make_shared<mower_msgs::srv::PerimeterControlSrv::Request>();
        request->listen_on = 0;
        perimeter_client_->async_send_request(request);
        RCLCPP_INFO(node_->get_logger(), "Perimeter deactivated");
    }

    perimeter_sub_.reset();
    perimeter_client_.reset();
    return &IdleBehavior::INSTANCE;
}

bool PerimeterBase::is_perimeter_updated() {
    if (perimeter_updated_.load()) {
        perimeter_updated_.store(false);
        return true;
    }
    return false;
}

float PerimeterBase::inner_signal() {
    return direction_ > 0 ? 
           last_perimeter_.left * calibration_left_ : 
           last_perimeter_.right * calibration_right_;
}

float PerimeterBase::outer_signal() {
    return direction_ > 0 ? 
           last_perimeter_.right * calibration_right_ : 
           last_perimeter_.left * calibration_left_;
}

// PerimeterSearchBehavior implementation
bool PerimeterSearchBehavior::configured(const MowerLogicConfig& /*config*/) {
    // Would check config.perimeter_signal != 0
    return false;  // Disabled by default
}

std::string PerimeterSearchBehavior::state_name() {
    return "DOCKING";
}

Behavior* PerimeterSearchBehavior::execute() {
    if (!setup_connections()) return shutdown_connections();

    rclcpp::Rate rate(10);
    int tries = 100;
    int to_find = 5;

    // Wait ten seconds for initial perimeter signal
    while (tries-- && to_find) {
        if (is_perimeter_updated()) {
            to_find--;
        }
        rate.sleep();
    }

    if (to_find) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to activate perimeter");
        return shutdown_connections();
    }

    // We expect to be currently outside the perimeter wire => negative signal
    calibration_left_ = calibration_right_ = sign_center_ =
        last_perimeter_.left < 0 ? 1 : -1;

    if (inner_signal() > -MIN_SIGNAL || outer_signal() > -MIN_SIGNAL) {
        RCLCPP_ERROR(node_->get_logger(), "Signal too weak");
        return shutdown_connections();
    }

    calibration_left_ /= std::fabs(last_perimeter_.left);
    max_center_ = std::fabs(last_perimeter_.center);
    calibration_right_ /= std::fabs(last_perimeter_.right);

    geometry_msgs::msg::Twist vel;

    // Move straight until one of the signals changes to positive (inside)
    vel.angular.z = 0;
    vel.linear.x = SEARCH_SPEED;
    tries = 10;

    while (tries-- > 0) {
        cmd_vel_pub_->publish(vel);
        rate.sleep();
        if (is_perimeter_updated()) {
            tries = 10;
            if (inner_signal() > 0 || outer_signal() > 0) break;
        }
    }

    if (!tries) {
        RCLCPP_ERROR(node_->get_logger(), "Signal timeout");
        return shutdown_connections();
    }

    // Move further until the wheels are over the perimeter (approx. 30 cm)
    for (tries = 20; tries-- > 0;) {
        cmd_vel_pub_->publish(vel);
        rate.sleep();
    }

    return &PerimeterDockingBehavior::INSTANCE;
}

// PerimeterUndockingBehavior implementation
bool PerimeterUndockingBehavior::configured(const MowerLogicConfig& /*config*/) {
    // Would check config.perimeter_signal != 0 && config.undock_distance > 1.0
    return false;  // Disabled by default
}

std::string PerimeterUndockingBehavior::state_name() {
    return "UNDOCKING";
}

Behavior* PerimeterUndockingBehavior::execute() {
    if (!setup_connections()) return shutdown_connections();

    rclcpp::Rate rate(10);
    int tries = 100;
    int to_find = 5;

    // Wait ten seconds for initial perimeter signal
    while (tries-- && to_find) {
        if (is_perimeter_updated()) {
            to_find--;
        }
        rate.sleep();
    }

    if (to_find) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to activate perimeter");
        return shutdown_connections();
    }

    geometry_msgs::msg::Twist vel;

    // Move straight back 0.5 m
    vel.angular.z = 0;
    vel.linear.x = -SEARCH_SPEED;
    double travelled = 0;

    while (travelled < 0.5) {
        cmd_vel_pub_->publish(vel);
        rate.sleep();
        travelled += 0.1 * SEARCH_SPEED;  // rate is 10Hz
    }

    // Determine polarity of the cable
    calibration_left_ = calibration_right_ = sign_center_ = 1;
    if (inner_signal() < 0) {
        calibration_left_ = calibration_right_ = sign_center_ = -1;
    }

    float max_left = last_perimeter_.left * calibration_left_;
    max_center_ = std::fabs(last_perimeter_.center);
    float max_right = last_perimeter_.right * calibration_right_;

    // Make a 180 degree turn inwards
    vel.angular.z = direction_ * ANGULAR_SPEED;
    vel.linear.x = 0;
    tries = 240;

    while (inner_signal() > 0 && --tries) {
        cmd_vel_pub_->publish(vel);
        rate.sleep();
        if (is_perimeter_updated()) {
            float x = last_perimeter_.left * calibration_left_;
            if (x > max_left) max_left = x;
            x = std::fabs(last_perimeter_.center);
            if (x > max_center_) max_center_ = x;
            x = last_perimeter_.right * calibration_right_;
            if (x > max_right) max_right = x;
        }
    }

    if (!tries) {
        RCLCPP_ERROR(node_->get_logger(), "Could not turn inwards");
        return shutdown_connections();
    }

    if (max_left < MIN_SIGNAL || max_right < MIN_SIGNAL) {
        RCLCPP_ERROR(node_->get_logger(), "Signal too weak");
        return shutdown_connections();
    }

    // After turning we toggle turning direction
    direction_ = -direction_;
    calibration_left_ /= max_left;
    calibration_right_ /= max_right;

    return &PerimeterMoveToGpsBehavior::INSTANCE;
}

// PerimeterDockingBehavior implementation
std::string PerimeterDockingBehavior::state_name() {
    return "DOCKING";
}

Behavior* PerimeterDockingBehavior::arrived() {
    if (travelled_ > perimeter_config_.docking_distance) {
        RCLCPP_WARN(node_->get_logger(), "Travelled %.0f meters before reaching the station", 
                    travelled_);
        return &IdleBehavior::INSTANCE;
    }

    // Check for charging (would need access to power message)
    // For now, just check if we've travelled enough
    // if (get_power_().v_charge > 5.0) {
    //     charge_seen_++;
    //     if (charge_seen_ >= 2) {
    //         charge_seen_ = 0;
    //         return &IdleBehavior::DOCKED_INSTANCE;
    //     }
    // } else {
    //     charge_seen_ = 0;
    // }

    return nullptr;  // Continue following
}

// PerimeterFollowBehavior implementation
Behavior* PerimeterFollowBehavior::execute() {
    rclcpp::Rate rate(10);
    geometry_msgs::msg::Twist vel;
    travelled_ = 0;
    int state = FOLLOW_STATE_FOLLOW;
    double travel_time_since_update = 0;
    double last_alpha0 = 0;
    int tries = 0;
    perimeter_updated_.store(true);  // Use first measurement
    Behavior* to_return;
    double drift = 0;             // Angular velocity if going straight
    double average_interval = 5;  // Average five seconds

    while (rclcpp::ok() && !(to_return = arrived())) {
        if (!is_perimeter_updated()) {
            cmd_vel_pub_->publish(vel);
            rate.sleep();
            if (tries && --tries == 0) {
                RCLCPP_ERROR(node_->get_logger(), "Timeout of action %d", state);
                to_return = &IdleBehavior::INSTANCE;
                break;
            }
            double d = 0.1;  // 10Hz rate
            travelled_ += d * vel.linear.x;
            travel_time_since_update += d;
            continue;
        }

        // Turn into direction of perimeter
        if (inner_signal() < 0) {
            // Inner coil is outside
            vel.linear.x = 0;
            vel.angular.z = ANGULAR_SPEED * direction_;
            if (state != FOLLOW_STATE_TURN_IN) {
                tries = 240;
                state = FOLLOW_STATE_TURN_IN;
            }
        } else if (outer_signal() > 0) {
            // Outer coil is inside
            vel.linear.x = 0;
            vel.angular.z = -ANGULAR_SPEED * direction_;
            if (state != FOLLOW_STATE_TURN_OUT) {
                tries = 240;
                state = FOLLOW_STATE_TURN_OUT;
            }
        } else {
            vel.linear.x = SEARCH_SPEED;
            if (std::fabs(last_perimeter_.center) > max_center_) {
                max_center_ = std::fabs(last_perimeter_.center);
            }
            double c = last_perimeter_.center * sign_center_;

            // Signal of center coil is proportional to distance from wire
            // y0: Position of the wire in mower coordinates
            double y0 = -direction_ * COIL_Y_OFFSET * c / max_center_;
            double alpha0 = y0 / COIL_X_OFFSET;  // deflection

            if (state != FOLLOW_STATE_FOLLOW) {
                state = FOLLOW_STATE_FOLLOW;
                tries = 0;
            } else {
                if (travel_time_since_update > 0) {
                    double d0 = -vel.angular.z + (alpha0 - last_alpha0) / travel_time_since_update;
                    double f = std::exp(-travel_time_since_update / average_interval);
                    drift = drift * f + d0 * (1 - f);
                }
            }

            vel.angular.z = drift + alpha0 / 2;  // Correct deflection within 2 seconds
            if (vel.angular.z > ANGULAR_SPEED) {
                vel.angular.z = ANGULAR_SPEED;
            } else if (vel.angular.z < -ANGULAR_SPEED) {
                vel.angular.z = -ANGULAR_SPEED;
            }
            travel_time_since_update = 0;
            last_alpha0 = alpha0;
        }
    }

    // Stop
    vel.linear.x = 0;
    vel.angular.z = 0;
    cmd_vel_pub_->publish(vel);
    shutdown_connections();
    return to_return;
}

// PerimeterMoveToGpsBehavior implementation
void PerimeterMoveToGpsBehavior::enter() {
    PerimeterBase::enter();
    // Would call setGPS(true)
}

std::string PerimeterMoveToGpsBehavior::state_name() {
    return "UNDOCKING";
}

Behavior* PerimeterMoveToGpsBehavior::arrived() {
    // Continue until we've travelled far enough
    return travelled_ >= perimeter_config_.undock_distance - 0.9 ? 
           &MowingBehavior::INSTANCE : nullptr;
}

}  // namespace mower_logic
