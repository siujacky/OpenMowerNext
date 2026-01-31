// Copyright (c) 2023 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: GPL-3.0
//
// ROS2 port of PerimeterDocking behaviors
// Perimeter-wire-based docking behaviors for mowers with perimeter wire

#ifndef MOWER_LOGIC__BEHAVIORS__PERIMETER_DOCKING_HPP_
#define MOWER_LOGIC__BEHAVIORS__PERIMETER_DOCKING_HPP_

#include <atomic>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "mower_logic/behaviors/behavior.hpp"
#include "mower_msgs/msg/perimeter.hpp"
#include "mower_msgs/srv/perimeter_control_srv.hpp"

namespace mower_logic {

// Forward declarations
class IdleBehavior;
class MowingBehavior;

// Constants for perimeter following
constexpr int MIN_SIGNAL = 5;
constexpr double SEARCH_SPEED = 0.1;
constexpr double ANGULAR_SPEED = 0.5;

// Distance from center to outer coil
constexpr double COIL_Y_OFFSET = 0.11;
// Distance from rear axis to coils
constexpr double COIL_X_OFFSET = 0.40;

// Follow states
constexpr int FOLLOW_STATE_FOLLOW = 0;
constexpr int FOLLOW_STATE_TURN_IN = 1;
constexpr int FOLLOW_STATE_TURN_OUT = 2;

/**
 * Extended config for perimeter behaviors
 */
struct PerimeterConfig {
    int perimeter_signal = 0;  // 0 = disabled, positive = counter-clockwise, negative = clockwise
    double docking_distance = 20.0;  // Maximum distance to travel while docking
    double undock_distance = 2.0;
};

/**
 * Base class for all perimeter-based behaviors
 */
class PerimeterBase : public Behavior {
public:
    void enter() override;
    void exit() override;
    void reset() override;
    bool needs_gps() override;
    bool mower_enabled() override;
    void command_home() override;
    void command_start() override;
    void command_s1() override;
    void command_s2() override;
    bool redirect_joystick() override;
    uint8_t get_sub_state() override;
    uint8_t get_state() override;
    void handle_action(const std::string& action) override;

protected:
    int setup_connections();
    Behavior* shutdown_connections();

    // Shared state
    static rclcpp::Subscription<mower_msgs::msg::Perimeter>::SharedPtr perimeter_sub_;
    static rclcpp::Client<mower_msgs::srv::PerimeterControlSrv>::SharedPtr perimeter_client_;
    static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    static mower_msgs::msg::Perimeter last_perimeter_;
    static std::atomic<bool> perimeter_updated_;
    static int direction_;  // 1 for counter-clockwise, -1 for clockwise
    static double calibration_left_;
    static double calibration_right_;
    static double max_center_;
    static int sign_center_;

    PerimeterConfig perimeter_config_;

    bool is_perimeter_updated();
    float inner_signal();
    float outer_signal();
};

/**
 * Base class for perimeter following behaviors
 */
class PerimeterFollowBehavior : public PerimeterBase {
public:
    Behavior* execute() override;

protected:
    double travelled_ = 0.0;

    /**
     * Called when we have arrived at destination
     * @return Next behavior, or nullptr to continue following
     */
    virtual Behavior* arrived() = 0;
};

/**
 * Docking behavior using perimeter wire
 */
class PerimeterDockingBehavior : public PerimeterFollowBehavior {
public:
    static PerimeterDockingBehavior INSTANCE;

    std::string state_name() override;

protected:
    Behavior* arrived() override;

private:
    int charge_seen_ = 0;
};

/**
 * Search behavior - find and align with perimeter wire
 */
class PerimeterSearchBehavior : public PerimeterBase {
public:
    static PerimeterSearchBehavior INSTANCE;

    /**
     * Check if perimeter usage is configured
     */
    static bool configured(const MowerLogicConfig& config);

    std::string state_name() override;
    Behavior* execute() override;
};

/**
 * Undocking behavior using perimeter wire
 */
class PerimeterUndockingBehavior : public PerimeterBase {
public:
    static PerimeterUndockingBehavior INSTANCE;

    /**
     * Check if perimeter undocking is configured
     */
    static bool configured(const MowerLogicConfig& config);

    std::string state_name() override;
    Behavior* execute() override;
};

/**
 * Move to GPS coverage area after undocking
 */
class PerimeterMoveToGpsBehavior : public PerimeterFollowBehavior {
public:
    static PerimeterMoveToGpsBehavior INSTANCE;

    std::string state_name() override;
    void enter() override;

protected:
    Behavior* arrived() override;
};

}  // namespace mower_logic

#endif  // MOWER_LOGIC__BEHAVIORS__PERIMETER_DOCKING_HPP_
