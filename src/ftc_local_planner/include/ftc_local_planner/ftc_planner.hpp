// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// ROS2 port of FTC (Follow The Carrot) Local Planner
// Adapted from move_base_flex to Nav2 controller interface

#ifndef FTC_LOCAL_PLANNER__FTC_PLANNER_HPP_
#define FTC_LOCAL_PLANNER__FTC_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <Eigen/Geometry>

#include "ftc_local_planner/oscillation_detector.hpp"
#include "ftc_local_planner_msgs/msg/pid.hpp"
#include "ftc_local_planner_msgs/srv/planner_get_progress.hpp"

namespace ftc_local_planner {

class FTCPlanner : public nav2_core::Controller {
public:
    FTCPlanner() = default;
    ~FTCPlanner() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped& pose,
        const geometry_msgs::msg::Twist& velocity,
        nav2_core::GoalChecker* goal_checker) override;

    void setPlan(const nav_msgs::msg::Path& path) override;
    void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

protected:
    enum class PlannerState {
        PRE_ROTATE,
        FOLLOWING,
        WAITING_FOR_GOAL_APPROACH,
        POST_ROTATE,
        FINISHED
    };

    // Configuration parameters
    struct Config {
        double speed_fast = 0.5;
        double speed_slow = 0.2;
        double speed_angular = 45.0;  // degrees/sec
        double speed_fast_threshold = 0.3;
        double speed_fast_threshold_angle = 10.0;  // degrees
        double acceleration = 0.5;
        double max_cmd_vel_speed = 0.5;
        double max_cmd_vel_ang = 1.0;
        double max_goal_distance_error = 0.05;
        double max_goal_angle_error = 5.0;  // degrees
        double max_follow_distance = 0.5;
        double goal_timeout = 30.0;
        int obstacle_lookahead = 10;
        bool check_obstacles = true;
        bool obstacle_footprint = true;
        bool forward_only = false;
        bool debug_pid = false;
        bool debug_obstacle = false;
        
        // PID gains
        double kp_lon = 1.0;
        double ki_lon = 0.0;
        double kd_lon = 0.0;
        double kp_lat = 1.0;
        double ki_lat = 0.0;
        double kd_lat = 0.0;
        double kp_ang = 1.0;
        double ki_ang = 0.0;
        double kd_ang = 0.0;
        double ki_lon_max = 1.0;
        double ki_lat_max = 1.0;
        double ki_ang_max = 1.0;

        // Oscillation recovery
        bool oscillation_recovery = true;
        double oscillation_recovery_min_duration = 2.0;
        double oscillation_v_eps = 0.1;
        double oscillation_omega_eps = 0.1;
    };

    void loadParameters();
    double distanceLookahead();
    PlannerState updatePlannerState();
    void updateControlPoint(double dt);
    void calculateVelocityCommands(double dt, geometry_msgs::msg::TwistStamped& cmd_vel);
    bool checkCollision(int max_points);
    bool checkOscillation(geometry_msgs::msg::TwistStamped& cmd_vel);
    void debugObstacle(visualization_msgs::msg::Marker& obstacle_points, 
                       double x, double y, unsigned char cost, int max_ids);
    double timeInCurrentState();

    void progressServiceCallback(
        const std::shared_ptr<ftc_local_planner_msgs::srv::PlannerGetProgress::Request> request,
        std::shared_ptr<ftc_local_planner_msgs::srv::PlannerGetProgress::Response> response);

    // ROS2 node components
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_{rclcpp::get_logger("FTCPlanner")};
    std::string plugin_name_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_;

    // Publishers
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_point_pub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_marker_pub_;
    rclcpp_lifecycle::LifecyclePublisher<ftc_local_planner_msgs::msg::PID>::SharedPtr pid_pub_;

    // Service
    rclcpp::Service<ftc_local_planner_msgs::srv::PlannerGetProgress>::SharedPtr progress_service_;

    // State
    Config config_;
    PlannerState current_state_ = PlannerState::PRE_ROTATE;
    rclcpp::Time state_entered_time_;
    bool is_crashed_ = false;

    std::vector<geometry_msgs::msg::PoseStamped> global_plan_;
    Eigen::Affine3d current_control_point_;
    Eigen::Affine3d local_control_point_;

    // PID state
    double lat_error_ = 0.0, lon_error_ = 0.0, angle_error_ = 0.0;
    double last_lon_error_ = 0.0, last_lat_error_ = 0.0, last_angle_error_ = 0.0;
    double i_lon_error_ = 0.0, i_lat_error_ = 0.0, i_angle_error_ = 0.0;
    rclcpp::Time last_time_;

    // Speed ramp
    double current_movement_speed_ = 0.0;

    // Path progress
    uint32_t current_index_ = 0;
    double current_progress_ = 0.0;

    // Oscillation detection
    FailureDetector failure_detector_;
    rclcpp::Time time_last_oscillation_;
    bool oscillation_detected_ = false;
    bool oscillation_warning_ = false;
};

}  // namespace ftc_local_planner

#endif  // FTC_LOCAL_PLANNER__FTC_PLANNER_HPP_
