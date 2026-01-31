// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// ROS2 port of FTC Local Planner

#include "ftc_local_planner/ftc_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

PLUGINLIB_EXPORT_CLASS(ftc_local_planner::FTCPlanner, nav2_core::Controller)

namespace ftc_local_planner {

void FTCPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    
    node_ = parent;
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error("Unable to lock node!");
    }

    logger_ = node->get_logger();
    clock_ = node->get_clock();
    plugin_name_ = name;
    tf_buffer_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    loadParameters();

    // Create publishers
    global_point_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        plugin_name_ + "/global_point", 1);
    global_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(
        plugin_name_ + "/global_plan", rclcpp::QoS(1).transient_local());
    obstacle_marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
        plugin_name_ + "/costmap_marker", 10);

    if (config_.debug_pid) {
        pid_pub_ = node->create_publisher<ftc_local_planner_msgs::msg::PID>(
            plugin_name_ + "/debug_pid", rclcpp::QoS(1).transient_local());
    }

    // Create service
    progress_service_ = node->create_service<ftc_local_planner_msgs::srv::PlannerGetProgress>(
        plugin_name_ + "/planner_get_progress",
        std::bind(&FTCPlanner::progressServiceCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Initialize oscillation detector
    failure_detector_.setBufferLength(
        static_cast<size_t>(std::round(config_.oscillation_recovery_min_duration * 10)));

    RCLCPP_INFO(logger_, "FTCPlanner: Version 2 (ROS2) initialized");
}

void FTCPlanner::loadParameters() {
    auto node = node_.lock();
    if (!node) return;

    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".speed_fast", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".speed_slow", rclcpp::ParameterValue(0.2));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".speed_angular", rclcpp::ParameterValue(45.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".speed_fast_threshold", rclcpp::ParameterValue(0.3));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".speed_fast_threshold_angle", rclcpp::ParameterValue(10.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".acceleration", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_cmd_vel_speed", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_cmd_vel_ang", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_goal_distance_error", rclcpp::ParameterValue(0.05));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_goal_angle_error", rclcpp::ParameterValue(5.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_follow_distance", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_timeout", rclcpp::ParameterValue(30.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".obstacle_lookahead", rclcpp::ParameterValue(10));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".check_obstacles", rclcpp::ParameterValue(true));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".obstacle_footprint", rclcpp::ParameterValue(true));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".forward_only", rclcpp::ParameterValue(false));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".debug_pid", rclcpp::ParameterValue(false));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".debug_obstacle", rclcpp::ParameterValue(false));
    
    // PID parameters
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".kp_lon", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".ki_lon", rclcpp::ParameterValue(0.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".kd_lon", rclcpp::ParameterValue(0.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".kp_lat", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".ki_lat", rclcpp::ParameterValue(0.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".kd_lat", rclcpp::ParameterValue(0.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".kp_ang", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".ki_ang", rclcpp::ParameterValue(0.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".kd_ang", rclcpp::ParameterValue(0.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".ki_lon_max", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".ki_lat_max", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".ki_ang_max", rclcpp::ParameterValue(1.0));
    
    // Oscillation recovery
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".oscillation_recovery", rclcpp::ParameterValue(true));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".oscillation_recovery_min_duration", rclcpp::ParameterValue(2.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".oscillation_v_eps", rclcpp::ParameterValue(0.1));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".oscillation_omega_eps", rclcpp::ParameterValue(0.1));

    node->get_parameter(plugin_name_ + ".speed_fast", config_.speed_fast);
    node->get_parameter(plugin_name_ + ".speed_slow", config_.speed_slow);
    node->get_parameter(plugin_name_ + ".speed_angular", config_.speed_angular);
    node->get_parameter(plugin_name_ + ".speed_fast_threshold", config_.speed_fast_threshold);
    node->get_parameter(plugin_name_ + ".speed_fast_threshold_angle", config_.speed_fast_threshold_angle);
    node->get_parameter(plugin_name_ + ".acceleration", config_.acceleration);
    node->get_parameter(plugin_name_ + ".max_cmd_vel_speed", config_.max_cmd_vel_speed);
    node->get_parameter(plugin_name_ + ".max_cmd_vel_ang", config_.max_cmd_vel_ang);
    node->get_parameter(plugin_name_ + ".max_goal_distance_error", config_.max_goal_distance_error);
    node->get_parameter(plugin_name_ + ".max_goal_angle_error", config_.max_goal_angle_error);
    node->get_parameter(plugin_name_ + ".max_follow_distance", config_.max_follow_distance);
    node->get_parameter(plugin_name_ + ".goal_timeout", config_.goal_timeout);
    node->get_parameter(plugin_name_ + ".obstacle_lookahead", config_.obstacle_lookahead);
    node->get_parameter(plugin_name_ + ".check_obstacles", config_.check_obstacles);
    node->get_parameter(plugin_name_ + ".obstacle_footprint", config_.obstacle_footprint);
    node->get_parameter(plugin_name_ + ".forward_only", config_.forward_only);
    node->get_parameter(plugin_name_ + ".debug_pid", config_.debug_pid);
    node->get_parameter(plugin_name_ + ".debug_obstacle", config_.debug_obstacle);
    node->get_parameter(plugin_name_ + ".kp_lon", config_.kp_lon);
    node->get_parameter(plugin_name_ + ".ki_lon", config_.ki_lon);
    node->get_parameter(plugin_name_ + ".kd_lon", config_.kd_lon);
    node->get_parameter(plugin_name_ + ".kp_lat", config_.kp_lat);
    node->get_parameter(plugin_name_ + ".ki_lat", config_.ki_lat);
    node->get_parameter(plugin_name_ + ".kd_lat", config_.kd_lat);
    node->get_parameter(plugin_name_ + ".kp_ang", config_.kp_ang);
    node->get_parameter(plugin_name_ + ".ki_ang", config_.ki_ang);
    node->get_parameter(plugin_name_ + ".kd_ang", config_.kd_ang);
    node->get_parameter(plugin_name_ + ".ki_lon_max", config_.ki_lon_max);
    node->get_parameter(plugin_name_ + ".ki_lat_max", config_.ki_lat_max);
    node->get_parameter(plugin_name_ + ".ki_ang_max", config_.ki_ang_max);
    node->get_parameter(plugin_name_ + ".oscillation_recovery", config_.oscillation_recovery);
    node->get_parameter(plugin_name_ + ".oscillation_recovery_min_duration", config_.oscillation_recovery_min_duration);
    node->get_parameter(plugin_name_ + ".oscillation_v_eps", config_.oscillation_v_eps);
    node->get_parameter(plugin_name_ + ".oscillation_omega_eps", config_.oscillation_omega_eps);

    current_movement_speed_ = config_.speed_slow;
}

void FTCPlanner::cleanup() {
    RCLCPP_INFO(logger_, "Cleaning up FTCPlanner");
    global_point_pub_.reset();
    global_plan_pub_.reset();
    obstacle_marker_pub_.reset();
    pid_pub_.reset();
    progress_service_.reset();
}

void FTCPlanner::activate() {
    RCLCPP_INFO(logger_, "Activating FTCPlanner");
    global_point_pub_->on_activate();
    global_plan_pub_->on_activate();
    obstacle_marker_pub_->on_activate();
    if (pid_pub_) pid_pub_->on_activate();
}

void FTCPlanner::deactivate() {
    RCLCPP_INFO(logger_, "Deactivating FTCPlanner");
    global_point_pub_->on_deactivate();
    global_plan_pub_->on_deactivate();
    obstacle_marker_pub_->on_deactivate();
    if (pid_pub_) pid_pub_->on_deactivate();
}

void FTCPlanner::setPlan(const nav_msgs::msg::Path& path) {
    current_state_ = PlannerState::PRE_ROTATE;
    state_entered_time_ = clock_->now();
    is_crashed_ = false;

    global_plan_ = path.poses;
    current_index_ = 0;
    current_progress_ = 0.0;

    last_time_ = clock_->now();
    current_movement_speed_ = config_.speed_slow;

    lat_error_ = lon_error_ = angle_error_ = 0.0;
    i_lon_error_ = i_lat_error_ = i_angle_error_ = 0.0;

    if (global_plan_.size() > 2) {
        global_plan_.push_back(global_plan_.back());
        global_plan_[global_plan_.size() - 2].pose.orientation = 
            global_plan_[global_plan_.size() - 3].pose.orientation;
    } else {
        RCLCPP_WARN(logger_, "FTCPlanner: Global plan too short. Need minimum 3 poses.");
        current_state_ = PlannerState::FINISHED;
        state_entered_time_ = clock_->now();
    }

    // Publish the plan
    nav_msgs::msg::Path pub_path;
    pub_path.header = path.header;
    pub_path.poses = global_plan_;
    global_plan_pub_->publish(pub_path);

    RCLCPP_INFO(logger_, "FTCPlanner: Got new global plan with %zu points", path.poses.size());
}

void FTCPlanner::setSpeedLimit(const double& speed_limit, const bool& percentage) {
    if (percentage) {
        config_.max_cmd_vel_speed *= speed_limit / 100.0;
    } else {
        config_.max_cmd_vel_speed = speed_limit;
    }
}

geometry_msgs::msg::TwistStamped FTCPlanner::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& /*velocity*/,
    nav2_core::GoalChecker* /*goal_checker*/) {
    
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.header.frame_id = pose.header.frame_id;

    rclcpp::Time now = clock_->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (is_crashed_) {
        cmd_vel.twist.linear.x = 0;
        cmd_vel.twist.angular.z = 0;
        throw nav2_core::PlannerException("FTCPlanner: Collision detected");
    }

    if (current_state_ == PlannerState::FINISHED) {
        cmd_vel.twist.linear.x = 0;
        cmd_vel.twist.angular.z = 0;
        return cmd_vel;
    }

    // Update control point and state
    updateControlPoint(dt);
    auto new_state = updatePlannerState();
    if (new_state != current_state_) {
        RCLCPP_INFO(logger_, "FTCPlanner: Switching to state %d", static_cast<int>(new_state));
        state_entered_time_ = clock_->now();
        current_state_ = new_state;
    }

    if (checkCollision(config_.obstacle_lookahead)) {
        cmd_vel.twist.linear.x = 0;
        cmd_vel.twist.angular.z = 0;
        is_crashed_ = true;
        throw nav2_core::PlannerException("FTCPlanner: Path blocked");
    }

    calculateVelocityCommands(dt, cmd_vel);

    return cmd_vel;
}

double FTCPlanner::timeInCurrentState() {
    return (clock_->now() - state_entered_time_).seconds();
}

double FTCPlanner::distanceLookahead() {
    if (global_plan_.size() < 2) return 0;

    Eigen::Quaternion<double> current_rot(current_control_point_.linear());
    double lookahead_distance = 0.0;
    Eigen::Affine3d last_straight_point = current_control_point_;
    Eigen::Affine3d current_point;

    for (uint32_t i = current_index_ + 1; i < global_plan_.size(); i++) {
        tf2::fromMsg(global_plan_[i].pose, current_point);

        Eigen::Quaternion<double> rot2(current_point.linear());

        if (lookahead_distance > config_.speed_fast_threshold ||
            std::abs(rot2.angularDistance(current_rot)) > 
            config_.speed_fast_threshold_angle * (M_PI / 180.0)) {
            break;
        }

        lookahead_distance += (current_point.translation() - last_straight_point.translation()).norm();
        last_straight_point = current_point;
    }

    return lookahead_distance;
}

FTCPlanner::PlannerState FTCPlanner::updatePlannerState() {
    switch (current_state_) {
        case PlannerState::PRE_ROTATE: {
            if (timeInCurrentState() > config_.goal_timeout) {
                RCLCPP_ERROR(logger_, "FTCPlanner: Timeout in PRE_ROTATE phase");
                is_crashed_ = true;
                return PlannerState::FINISHED;
            }
            if (std::abs(angle_error_) * (180.0 / M_PI) < config_.max_goal_angle_error) {
                RCLCPP_INFO(logger_, "FTCPlanner: PRE_ROTATE finished. Starting following");
                return PlannerState::FOLLOWING;
            }
            break;
        }
        case PlannerState::FOLLOWING: {
            double distance = local_control_point_.translation().norm();
            if (distance > config_.max_follow_distance) {
                RCLCPP_ERROR(logger_, "FTCPlanner: Robot far from path (%.2f > %.2f)", 
                             distance, config_.max_follow_distance);
                is_crashed_ = true;
                return PlannerState::FINISHED;
            }
            if (current_index_ == global_plan_.size() - 2) {
                RCLCPP_INFO(logger_, "FTCPlanner: Switching to position mode");
                return PlannerState::WAITING_FOR_GOAL_APPROACH;
            }
            break;
        }
        case PlannerState::WAITING_FOR_GOAL_APPROACH: {
            double distance = local_control_point_.translation().norm();
            if (timeInCurrentState() > config_.goal_timeout) {
                RCLCPP_WARN(logger_, "FTCPlanner: Goal approach timeout. Attempting final rotation.");
                return PlannerState::POST_ROTATE;
            }
            if (distance < config_.max_goal_distance_error) {
                RCLCPP_INFO(logger_, "FTCPlanner: Reached goal position.");
                return PlannerState::POST_ROTATE;
            }
            break;
        }
        case PlannerState::POST_ROTATE: {
            if (timeInCurrentState() > config_.goal_timeout) {
                RCLCPP_WARN(logger_, "FTCPlanner: Goal rotation timeout");
                return PlannerState::FINISHED;
            }
            if (std::abs(angle_error_) * (180.0 / M_PI) < config_.max_goal_angle_error) {
                RCLCPP_INFO(logger_, "FTCPlanner: POST_ROTATE finished");
                return PlannerState::FINISHED;
            }
            break;
        }
        case PlannerState::FINISHED:
            break;
    }
    return current_state_;
}

void FTCPlanner::updateControlPoint(double dt) {
    if (global_plan_.empty()) return;

    switch (current_state_) {
        case PlannerState::PRE_ROTATE:
            tf2::fromMsg(global_plan_[0].pose, current_control_point_);
            break;
            
        case PlannerState::FOLLOWING: {
            double straight_dist = distanceLookahead();
            double speed = (straight_dist >= config_.speed_fast_threshold) ? 
                           config_.speed_fast : config_.speed_slow;

            if (speed > current_movement_speed_) {
                current_movement_speed_ += dt * config_.acceleration;
                if (current_movement_speed_ > speed) current_movement_speed_ = speed;
            } else if (speed < current_movement_speed_) {
                current_movement_speed_ -= dt * config_.acceleration;
                if (current_movement_speed_ < speed) current_movement_speed_ = speed;
            }

            double distance_to_move = dt * current_movement_speed_;
            double angle_to_move = dt * config_.speed_angular * (M_PI / 180.0);

            Eigen::Affine3d nextPose, currentPose;
            while (angle_to_move > 0 && distance_to_move > 0 && current_index_ < global_plan_.size() - 2) {
                tf2::fromMsg(global_plan_[current_index_].pose, currentPose);
                tf2::fromMsg(global_plan_[current_index_ + 1].pose, nextPose);

                double pose_distance = (nextPose.translation() - currentPose.translation()).norm();

                Eigen::Quaternion<double> current_rot(currentPose.linear());
                Eigen::Quaternion<double> next_rot(nextPose.linear());
                double pose_distance_angular = current_rot.angularDistance(next_rot);

                if (pose_distance <= 0.0) {
                    RCLCPP_WARN(logger_, "FTCPlanner: Skipping duplicate point");
                    current_index_++;
                    continue;
                }

                double remaining_distance = pose_distance * (1.0 - current_progress_);
                double remaining_angular = pose_distance_angular * (1.0 - current_progress_);

                if (remaining_distance < distance_to_move && remaining_angular < angle_to_move) {
                    current_progress_ = 0.0;
                    current_index_++;
                    distance_to_move -= remaining_distance;
                    angle_to_move -= remaining_angular;
                } else {
                    double progress_dist = (pose_distance * current_progress_ + distance_to_move) / pose_distance;
                    double progress_ang = (pose_distance_angular * current_progress_ + angle_to_move) / pose_distance_angular;
                    current_progress_ = std::min(progress_ang, progress_dist);
                    distance_to_move = 0;
                    angle_to_move = 0;
                }
            }

            tf2::fromMsg(global_plan_[current_index_].pose, currentPose);
            tf2::fromMsg(global_plan_[current_index_ + 1].pose, nextPose);

            Eigen::Quaternion<double> rot1(currentPose.linear());
            Eigen::Quaternion<double> rot2(nextPose.linear());

            Eigen::Affine3d result;
            result.translation() = (1.0 - current_progress_) * currentPose.translation() + 
                                   current_progress_ * nextPose.translation();
            result.linear() = rot1.slerp(current_progress_, rot2).toRotationMatrix();
            current_control_point_ = result;
            break;
        }
        
        case PlannerState::POST_ROTATE:
            tf2::fromMsg(global_plan_.back().pose, current_control_point_);
            break;
            
        case PlannerState::WAITING_FOR_GOAL_APPROACH:
        case PlannerState::FINISHED:
            break;
    }

    // Publish control point
    geometry_msgs::msg::PoseStamped viz;
    viz.header.frame_id = global_plan_[current_index_].header.frame_id;
    viz.header.stamp = clock_->now();
    viz.pose = tf2::toMsg(current_control_point_);
    global_point_pub_->publish(viz);

    // Transform to base_link
    try {
        auto map_to_base = tf_buffer_->lookupTransform(
            "base_link", "map", tf2::TimePointZero);
        tf2::doTransform(current_control_point_, local_control_point_, map_to_base);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(logger_, "TF lookup failed: %s", ex.what());
        return;
    }

    lat_error_ = local_control_point_.translation().y();
    lon_error_ = local_control_point_.translation().x();
    angle_error_ = local_control_point_.rotation().eulerAngles(0, 1, 2).z();
}

void FTCPlanner::calculateVelocityCommands(double dt, geometry_msgs::msg::TwistStamped& cmd_vel) {
    if (current_state_ == PlannerState::FINISHED || is_crashed_) {
        cmd_vel.twist.linear.x = 0;
        cmd_vel.twist.angular.z = 0;
        return;
    }

    // Update integral terms
    i_lon_error_ += lon_error_ * dt;
    i_lat_error_ += lat_error_ * dt;
    i_angle_error_ += angle_error_ * dt;

    // Clamp integral terms
    i_lon_error_ = std::clamp(i_lon_error_, -config_.ki_lon_max, config_.ki_lon_max);
    i_lat_error_ = std::clamp(i_lat_error_, -config_.ki_lat_max, config_.ki_lat_max);
    i_angle_error_ = std::clamp(i_angle_error_, -config_.ki_ang_max, config_.ki_ang_max);

    // Calculate derivatives
    double d_lat = (lat_error_ - last_lat_error_) / dt;
    double d_lon = (lon_error_ - last_lon_error_) / dt;
    double d_angle = (angle_error_ - last_angle_error_) / dt;

    last_lat_error_ = lat_error_;
    last_lon_error_ = lon_error_;
    last_angle_error_ = angle_error_;

    // Linear velocity (only in following mode)
    if (current_state_ == PlannerState::FOLLOWING) {
        double lin_speed = lon_error_ * config_.kp_lon + 
                          i_lon_error_ * config_.ki_lon + 
                          d_lon * config_.kd_lon;
        
        if (lin_speed < 0 && config_.forward_only) {
            lin_speed = 0;
        } else {
            lin_speed = std::clamp(lin_speed, -config_.max_cmd_vel_speed, config_.max_cmd_vel_speed);
            if (lin_speed < 0) lat_error_ *= -1.0;
        }
        cmd_vel.twist.linear.x = lin_speed;
    } else {
        cmd_vel.twist.linear.x = 0.0;
    }

    // Angular velocity
    double ang_speed;
    if (current_state_ == PlannerState::FOLLOWING) {
        ang_speed = angle_error_ * config_.kp_ang + i_angle_error_ * config_.ki_ang + d_angle * config_.kd_ang +
                    lat_error_ * config_.kp_lat + i_lat_error_ * config_.ki_lat + d_lat * config_.kd_lat;
    } else {
        ang_speed = angle_error_ * config_.kp_ang + i_angle_error_ * config_.ki_ang + d_angle * config_.kd_ang;
        
        if (checkOscillation(cmd_vel)) {
            ang_speed = config_.max_cmd_vel_ang;
        }
    }

    cmd_vel.twist.angular.z = std::clamp(ang_speed, -config_.max_cmd_vel_ang, config_.max_cmd_vel_ang);

    // Debug PID
    if (config_.debug_pid && pid_pub_) {
        ftc_local_planner_msgs::msg::PID debug_msg;
        debug_msg.kp_lat_set = lat_error_ * config_.kp_lat;
        debug_msg.kp_lon_set = lon_error_ * config_.kp_lon;
        debug_msg.kp_ang_set = angle_error_ * config_.kp_ang;
        debug_msg.ki_lat_set = i_lat_error_ * config_.ki_lat;
        debug_msg.ki_lon_set = i_lon_error_ * config_.ki_lon;
        debug_msg.ki_ang_set = i_angle_error_ * config_.ki_ang;
        debug_msg.kd_lat_set = d_lat * config_.kd_lat;
        debug_msg.kd_lon_set = d_lon * config_.kd_lon;
        debug_msg.kd_ang_set = d_angle * config_.kd_ang;
        debug_msg.lon_err = lon_error_;
        debug_msg.lat_err = lat_error_;
        debug_msg.ang_err = angle_error_;
        debug_msg.ang_speed = cmd_vel.twist.angular.z;
        debug_msg.lin_speed = cmd_vel.twist.linear.x;
        pid_pub_->publish(debug_msg);
    }
}

bool FTCPlanner::checkCollision(int max_points) {
    if (!config_.check_obstacles) return false;

    unsigned int x, y;
    unsigned char previous_cost = 255;

    if (static_cast<int>(global_plan_.size()) < max_points) {
        max_points = static_cast<int>(global_plan_.size());
    }

    // Check footprint at current pose
    if (config_.obstacle_footprint) {
        auto footprint = costmap_ros_->getRobotFootprint();
        for (const auto& pt : footprint) {
            if (costmap_->worldToMap(pt.x, pt.y, x, y)) {
                unsigned char cost = costmap_->getCost(x, y);
                if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
                    RCLCPP_WARN(logger_, "FTCPlanner: Footprint collision at current pose");
                    return true;
                }
            }
        }
    }

    // Check path ahead
    for (int i = 0; i < max_points; i++) {
        size_t index = current_index_ + i;
        if (index >= global_plan_.size()) break;

        const auto& pose = global_plan_[index];
        if (costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, x, y)) {
            unsigned char cost = costmap_->getCost(x, y);
            
            if (cost > 0 && cost > 127 && cost > previous_cost) {
                RCLCPP_WARN(logger_, "FTCPlanner: Possible collision ahead");
                return true;
            }
            previous_cost = cost;
        }
    }
    return false;
}

bool FTCPlanner::checkOscillation(geometry_msgs::msg::TwistStamped& cmd_vel) {
    if (!config_.oscillation_recovery) return false;

    failure_detector_.update(cmd_vel, config_.max_cmd_vel_speed, config_.max_cmd_vel_speed,
                             config_.max_cmd_vel_ang, config_.oscillation_v_eps, config_.oscillation_omega_eps);

    bool oscillating = failure_detector_.isOscillating();

    if (oscillating) {
        if (!oscillation_detected_) {
            time_last_oscillation_ = clock_->now();
            oscillation_detected_ = true;
        }

        bool timeout = (clock_->now() - time_last_oscillation_).seconds() >= config_.oscillation_recovery_min_duration;
        if (timeout) {
            if (!oscillation_warning_) {
                RCLCPP_WARN(logger_, "FTCPlanner: Oscillation detected. Activating recovery.");
                oscillation_warning_ = true;
            }
            return true;
        }
        return false;
    } else {
        time_last_oscillation_ = clock_->now();
        oscillation_detected_ = false;
        oscillation_warning_ = false;
        return false;
    }
}

void FTCPlanner::progressServiceCallback(
    const std::shared_ptr<ftc_local_planner_msgs::srv::PlannerGetProgress::Request> /*request*/,
    std::shared_ptr<ftc_local_planner_msgs::srv::PlannerGetProgress::Response> response) {
    response->index = current_index_;
}

}  // namespace ftc_local_planner
