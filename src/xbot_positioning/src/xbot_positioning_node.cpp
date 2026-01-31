/**
 * @file xbot_positioning_node.cpp
 * @brief ROS2 positioning node with Extended Kalman Filter sensor fusion
 *
 * Ported from open_mower_ros (ROS1) to ROS2
 * Original author: Clemens Elflein
 *
 * Changes from ROS1:
 * - ros::NodeHandle → rclcpp::Node
 * - ROS_* macros → RCLCPP_* macros
 * - tf2 ROS2 API
 * - QoS settings
 */

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <xbot_msgs/msg/absolute_pose.hpp>
#include <xbot_msgs/msg/wheel_tick.hpp>
#include <xbot_positioning_msgs/msg/kalman_state.hpp>
#include <xbot_positioning_msgs/srv/gps_control_srv.hpp>
#include <xbot_positioning_msgs/srv/set_pose_srv.hpp>

#include "xbot_positioning_core.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class XbotPositioningNode : public rclcpp::Node
{
public:
  XbotPositioningNode() : Node("xbot_positioning"), tf_broadcaster_(this)
  {
    // Initialize state
    has_gps_ = false;
    gps_enabled_ = true;
    vx_ = 0.0;
    has_gyro_ = false;
    has_ticks_ = false;
    gyro_offset_ = 0.0;
    gyro_offset_samples_ = 0;
    valid_gps_samples_ = 0;
    gps_outlier_count_ = 0;
    antenna_offset_x_ = 0.0;
    antenna_offset_y_ = 0.0;

    // Declare parameters
    skip_gyro_calibration_ = this->declare_parameter("skip_gyro_calibration", false);
    gyro_offset_ = this->declare_parameter("gyro_offset", 0.0);
    min_speed_ = this->declare_parameter("min_speed", 0.01);
    max_gps_accuracy_ = this->declare_parameter("max_gps_accuracy", 0.1);
    publish_debug_ = this->declare_parameter("debug", false);
    antenna_offset_x_ = this->declare_parameter("antenna_offset_x", 0.0);
    antenna_offset_y_ = this->declare_parameter("antenna_offset_y", 0.0);
    gps_message_throttle_ = this->declare_parameter("gps_message_throttle", 1);

    core_.setAntennaOffset(antenna_offset_x_, antenna_offset_y_);

    RCLCPP_INFO(this->get_logger(), "Antenna offset: %f, %f", antenna_offset_x_, antenna_offset_y_);

    if (gyro_offset_ != 0.0 && skip_gyro_calibration_)
    {
      RCLCPP_WARN(this->get_logger(), "Using gyro offset of: %f", gyro_offset_);
    }

    // Create services
    gps_service_ = this->create_service<xbot_positioning_msgs::srv::GPSControlSrv>(
        "xbot_positioning/set_gps_state", std::bind(&XbotPositioningNode::setGpsState, this, _1, _2));

    pose_service_ = this->create_service<xbot_positioning_msgs::srv::SetPoseSrv>(
        "xbot_positioning/set_robot_pose", std::bind(&XbotPositioningNode::setPose, this, _1, _2));

    // Create publishers
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_out", 50);
    xbot_absolute_pose_pub_ = this->create_publisher<xbot_msgs::msg::AbsolutePose>("xb_pose_out", 50);

    if (publish_debug_)
    {
      dbg_expected_motion_vector_pub_ =
          this->create_publisher<geometry_msgs::msg::Vector3>("debug_expected_motion_vector", 50);
      kalman_state_pub_ = this->create_publisher<xbot_positioning_msgs::msg::KalmanState>("kalman_state", 50);
    }

    // Create subscriptions
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu_in", 10,
                                                                std::bind(&XbotPositioningNode::onImu, this, _1));

    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "twist_in", 10, std::bind(&XbotPositioningNode::onTwistIn, this, _1));

    pose_sub_ = this->create_subscription<xbot_msgs::msg::AbsolutePose>(
        "xb_pose_in", 10, std::bind(&XbotPositioningNode::onPose, this, _1));

    wheel_tick_sub_ = this->create_subscription<xbot_msgs::msg::WheelTick>(
        "wheel_ticks_in", 10, std::bind(&XbotPositioningNode::onWheelTicks, this, _1));

    RCLCPP_INFO(this->get_logger(), "xbot_positioning node initialized");
  }

private:
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!has_gyro_)
    {
      if (!skip_gyro_calibration_)
      {
        if (gyro_offset_samples_ == 0)
        {
          RCLCPP_INFO(this->get_logger(), "Started gyro calibration");
          gyro_calibration_start_ = msg->header.stamp;
          gyro_offset_ = 0;
        }
        gyro_offset_ += msg->angular_velocity.z;
        gyro_offset_samples_++;

        double elapsed = (rclcpp::Time(msg->header.stamp) - rclcpp::Time(gyro_calibration_start_)).seconds();
        if (elapsed < 5.0)
        {
          last_imu_ = *msg;
          return;
        }
        has_gyro_ = true;
        if (gyro_offset_samples_ > 0)
        {
          gyro_offset_ /= gyro_offset_samples_;
        }
        else
        {
          gyro_offset_ = 0;
        }
        gyro_offset_samples_ = 0;
        RCLCPP_INFO(this->get_logger(), "Calibrated gyro offset: %f", gyro_offset_);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Skipped gyro calibration");
        has_gyro_ = true;
        return;
      }
    }

    double dt = (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_imu_.header.stamp)).seconds();
    core_.predict(vx_, msg->angular_velocity.z - gyro_offset_, dt);
    auto x = core_.updateSpeed(vx_, msg->angular_velocity.z - gyro_offset_, 0.01);

    // Build odometry message
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = this->now();
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = x.x_pos();
    odometry.pose.pose.position.y = x.y_pos();
    odometry.pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, x.theta());
    odometry.pose.pose.orientation = tf2::toMsg(q);

    // Broadcast transform
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header = odometry.header;
    odom_trans.child_frame_id = odometry.child_frame_id;
    odom_trans.transform.translation.x = odometry.pose.pose.position.x;
    odom_trans.transform.translation.y = odometry.pose.pose.position.y;
    odom_trans.transform.translation.z = odometry.pose.pose.position.z;
    odom_trans.transform.rotation = odometry.pose.pose.orientation;
    tf_broadcaster_.sendTransform(odom_trans);

    // Publish debug state
    if (publish_debug_)
    {
      auto state = core_.getState();
      xbot_positioning_msgs::msg::KalmanState state_msg;
      state_msg.x = state.x_pos();
      state_msg.y = state.y_pos();
      state_msg.theta = state.theta();
      state_msg.vx = state.vx();
      state_msg.vr = state.vr();
      kalman_state_pub_->publish(state_msg);
    }

    odometry_pub_->publish(odometry);

    // Build absolute pose message
    xbot_msgs::msg::AbsolutePose xb_pose_msg;
    xb_pose_msg.header = odometry.header;
    xb_pose_msg.sensor_stamp = 0;
    xb_pose_msg.received_stamp = 0;
    xb_pose_msg.source = xbot_msgs::msg::AbsolutePose::SOURCE_SENSOR_FUSION;
    xb_pose_msg.flags = xbot_msgs::msg::AbsolutePose::FLAG_SENSOR_FUSION_DEAD_RECKONING;
    xb_pose_msg.orientation_valid = true;
    xb_pose_msg.motion_vector_valid = false;

    if (has_gps_)
    {
      xb_pose_msg.position_accuracy = last_gps_.position_accuracy;
    }
    else
    {
      xb_pose_msg.position_accuracy = 999;
    }

    double time_since_gps = (this->now() - last_gps_time_).seconds();
    if (time_since_gps < 10.0)
    {
      xb_pose_msg.flags |= xbot_msgs::msg::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE;
    }
    else
    {
      xb_pose_msg.position_accuracy = 999;
    }

    xb_pose_msg.orientation_accuracy = 0.01;
    xb_pose_msg.pose = odometry.pose;
    xb_pose_msg.vehicle_heading = x.theta();
    xb_pose_msg.motion_heading = x.theta();

    xbot_absolute_pose_pub_->publish(xb_pose_msg);

    last_imu_ = *msg;
  }

  void onWheelTicks(const xbot_msgs::msg::WheelTick::SharedPtr msg)
  {
    if (!has_ticks_)
    {
      last_ticks_ = *msg;
      has_ticks_ = true;
      return;
    }

    double dt = (rclcpp::Time(msg->stamp) - rclcpp::Time(last_ticks_.stamp)).seconds();

    double d_wheel_l = static_cast<double>(msg->wheel_ticks_rl - last_ticks_.wheel_ticks_rl) *
                       (1.0 / static_cast<double>(msg->wheel_tick_factor));
    double d_wheel_r = static_cast<double>(msg->wheel_ticks_rr - last_ticks_.wheel_ticks_rr) *
                       (1.0 / static_cast<double>(msg->wheel_tick_factor));

    if (msg->wheel_direction_rl)
    {
      d_wheel_l *= -1.0;
    }
    if (msg->wheel_direction_rr)
    {
      d_wheel_r *= -1.0;
    }

    double d_ticks = (d_wheel_l + d_wheel_r) / 2.0;
    vx_ = d_ticks / dt;

    last_ticks_ = *msg;
  }

  void onTwistIn(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    vx_ = msg->twist.linear.x;
  }

  void onPose(const xbot_msgs::msg::AbsolutePose::SharedPtr msg)
  {
    if (!gps_enabled_)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), gps_message_throttle_ * 1000,
                           "dropping GPS update, since gps_enabled = false.");
      return;
    }

    if ((msg->flags & xbot_msgs::msg::AbsolutePose::FLAG_GPS_RTK_FIXED) == 0)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Dropped GPS update, since it's not RTK Fixed");
      return;
    }

    if (msg->position_accuracy > max_gps_accuracy_)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Dropped GPS update, since it's not accurate enough. Accuracy was: %f, limit is: %f",
                           msg->position_accuracy, max_gps_accuracy_);
      return;
    }

    double time_since_last_gps = (this->now() - last_gps_time_).seconds();
    if (time_since_last_gps > 5.0)
    {
      RCLCPP_WARN(this->get_logger(), "Last GPS was %f seconds ago.", time_since_last_gps);
      has_gps_ = false;
      valid_gps_samples_ = 0;
      gps_outlier_count_ = 0;
      last_gps_ = *msg;
      last_gps_time_ = this->now();
      return;
    }

    tf2::Vector3 gps_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    tf2::Vector3 last_gps_pos(last_gps_.pose.pose.position.x, last_gps_.pose.pose.position.y,
                              last_gps_.pose.pose.position.z);

    double distance_to_last_gps = (last_gps_pos - gps_pos).length();

    if (distance_to_last_gps < 5.0)
    {
      // Inlier
      last_gps_ = *msg;
      last_gps_time_ = this->now();
      gps_outlier_count_ = 0;
      valid_gps_samples_++;

      if (!has_gps_ && valid_gps_samples_ > 10)
      {
        RCLCPP_INFO(this->get_logger(), "GPS data now valid");
        RCLCPP_INFO(this->get_logger(), "First GPS data, moving kalman filter to %f, %f", msg->pose.pose.position.x,
                    msg->pose.pose.position.y);
        core_.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.001);
        has_gps_ = true;
      }
      else if (has_gps_)
      {
        core_.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, 500.0);

        if (publish_debug_)
        {
          auto m = core_.om2.h(core_.ekf.getState());
          geometry_msgs::msg::Vector3 dbg;
          dbg.x = m.vx();
          dbg.y = m.vy();
          dbg_expected_motion_vector_pub_->publish(dbg);
        }

        double speed = std::sqrt(std::pow(msg->motion_vector.x, 2) + std::pow(msg->motion_vector.y, 2));
        if (speed >= min_speed_)
        {
          core_.updateOrientation2(msg->motion_vector.x, msg->motion_vector.y, 10000.0);
        }
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "GPS outlier found. Distance was: %f", distance_to_last_gps);
      gps_outlier_count_++;

      if (gps_outlier_count_ > 10)
      {
        RCLCPP_ERROR(this->get_logger(), "too many outliers, assuming that the current gps value is valid.");
        last_gps_ = *msg;
        last_gps_time_ = this->now();
        has_gps_ = false;
        valid_gps_samples_ = 0;
        gps_outlier_count_ = 0;
      }
    }
  }

  void setGpsState(const std::shared_ptr<xbot_positioning_msgs::srv::GPSControlSrv::Request> request,
                   std::shared_ptr<xbot_positioning_msgs::srv::GPSControlSrv::Response> /*response*/)
  {
    gps_enabled_ = request->gps_enabled;
    RCLCPP_INFO(this->get_logger(), "GPS enabled: %s", gps_enabled_ ? "true" : "false");
  }

  void setPose(const std::shared_ptr<xbot_positioning_msgs::srv::SetPoseSrv::Request> request,
               std::shared_ptr<xbot_positioning_msgs::srv::SetPoseSrv::Response> /*response*/)
  {
    tf2::Quaternion q;
    tf2::fromMsg(request->robot_pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double unused1, unused2, yaw;
    m.getRPY(unused1, unused2, yaw);

    core_.setState(request->robot_pose.position.x, request->robot_pose.position.y, yaw, 0, 0);
    RCLCPP_INFO(this->get_logger(), "Robot pose set to: x=%f, y=%f, yaw=%f", request->robot_pose.position.x,
                request->robot_pose.position.y, yaw);
  }

  // Kalman filter core
  xbot::positioning::xbot_positioning_core core_;

  // TF broadcaster
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // Services
  rclcpp::Service<xbot_positioning_msgs::srv::GPSControlSrv>::SharedPtr gps_service_;
  rclcpp::Service<xbot_positioning_msgs::srv::SetPoseSrv>::SharedPtr pose_service_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<xbot_msgs::msg::AbsolutePose>::SharedPtr xbot_absolute_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr dbg_expected_motion_vector_pub_;
  rclcpp::Publisher<xbot_positioning_msgs::msg::KalmanState>::SharedPtr kalman_state_pub_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<xbot_msgs::msg::AbsolutePose>::SharedPtr pose_sub_;
  rclcpp::Subscription<xbot_msgs::msg::WheelTick>::SharedPtr wheel_tick_sub_;

  // Parameters
  bool skip_gyro_calibration_;
  double min_speed_;
  double max_gps_accuracy_;
  bool publish_debug_;
  double antenna_offset_x_;
  double antenna_offset_y_;
  int gps_message_throttle_;

  // State variables
  bool has_ticks_;
  xbot_msgs::msg::WheelTick last_ticks_;
  bool has_gps_;
  xbot_msgs::msg::AbsolutePose last_gps_;
  bool has_gyro_;
  sensor_msgs::msg::Imu last_imu_;
  builtin_interfaces::msg::Time gyro_calibration_start_;
  double gyro_offset_;
  int gyro_offset_samples_;
  double vx_;
  int gps_outlier_count_;
  int valid_gps_samples_;
  rclcpp::Time last_gps_time_{ 0, 0, RCL_ROS_TIME };
  bool gps_enabled_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XbotPositioningNode>());
  rclcpp::shutdown();
  return 0;
}
