// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: GPL-3.0
//
// ROS2 port of mower_logic node
// Main state machine for OpenMower

#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

#include "mower_msgs/msg/emergency.hpp"
#include "mower_msgs/msg/status.hpp"
#include "mower_msgs/msg/power.hpp"
#include "mower_msgs/msg/esc_status.hpp"
#include "mower_msgs/msg/high_level_status.hpp"
#include "mower_msgs/srv/high_level_control_srv.hpp"
#include "mower_msgs/srv/emergency_stop_srv.hpp"
#include "mower_msgs/srv/mower_control_srv.hpp"
#include "xbot_msgs/msg/absolute_pose.hpp"
#include "xbot_msgs/msg/action_info.hpp"
#include "xbot_msgs/srv/register_actions_srv.hpp"
#include "xbot_positioning_msgs/srv/gps_control_srv.hpp"
#include "xbot_positioning_msgs/srv/set_pose_srv.hpp"

#include "mower_logic/state_subscriber.hpp"
#include "mower_logic/behaviors/behavior.hpp"
#include "mower_logic/behaviors/idle_behavior.hpp"
#include "mower_logic/behaviors/mowing_behavior.hpp"
#include "mower_logic/behaviors/docking_behavior.hpp"
#include "mower_logic/behaviors/undocking_behavior.hpp"
#include "mower_logic/behaviors/area_recording_behavior.hpp"
#include "mower_logic/behaviors/perimeter_docking.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace mower_logic
{

class MowerLogicNode : public rclcpp::Node
{
public:
  MowerLogicNode() : Node("mower_logic"),
    last_good_gps_(0, 0, RCL_ROS_TIME),
    joy_vel_time_(0, 0, RCL_ROS_TIME),
    gps_initialized_(false)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Mower Logic Node");

    // Declare and get parameters
    declareParameters();

    // Initialize state subscribers
    emergency_sub_ = std::make_unique<StateSubscriber<mower_msgs::msg::Emergency>>(this, "/ll/emergency");
    status_sub_ = std::make_unique<StateSubscriber<mower_msgs::msg::Status>>(this, "/ll/mower_status");
    power_sub_ = std::make_unique<StateSubscriber<mower_msgs::msg::Power>>(this, "/ll/power");
    left_esc_sub_ =
        std::make_unique<StateSubscriber<mower_msgs::msg::ESCStatus>>(this, "/ll/diff_drive/left_esc_status");
    right_esc_sub_ =
        std::make_unique<StateSubscriber<mower_msgs::msg::ESCStatus>>(this, "/ll/diff_drive/right_esc_status");
    pose_sub_ = std::make_unique<StateSubscriber<xbot_msgs::msg::AbsolutePose>>(this, "/xbot_positioning/xb_pose");

    // Start subscribers
    emergency_sub_->start();
    status_sub_->start();
    power_sub_->start();
    left_esc_sub_->start();
    right_esc_sub_->start();
    pose_sub_->start();

    // Create publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/logic_vel", 1);
    high_level_status_pub_ = this->create_publisher<mower_msgs::msg::HighLevelStatus>(
        "mower_logic/current_state", rclcpp::QoS(100).transient_local());

    // Create service clients
    gps_client_ = this->create_client<xbot_positioning_msgs::srv::GPSControlSrv>("xbot_positioning/set_gps_state");
    mow_client_ = this->create_client<mower_msgs::srv::MowerControlSrv>("ll/_service/mow_enabled");
    emergency_client_ = this->create_client<mower_msgs::srv::EmergencyStopSrv>("ll/_service/emergency");
    positioning_client_ =
        this->create_client<xbot_positioning_msgs::srv::SetPoseSrv>("xbot_positioning/set_robot_pose");
    action_registration_client_ = this->create_client<xbot_msgs::srv::RegisterActionsSrv>("xbot/register_actions");

    // Joystick velocity subscriber
    joy_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/joy_vel", 10, std::bind(&MowerLogicNode::joyVelCallback, this, _1));

    // Action subscriber
    action_sub_ = this->create_subscription<std_msgs::msg::String>(
        "xbot/action", 10, std::bind(&MowerLogicNode::actionCallback, this, _1));

    // Create services
    high_level_control_srv_ = this->create_service<mower_msgs::srv::HighLevelControlSrv>(
        "mower_service/high_level_control", std::bind(&MowerLogicNode::highLevelControlCallback, this, _1, _2));

    // Create timers
    safety_timer_ = this->create_wall_timer(500ms, std::bind(&MowerLogicNode::safetyTimerCallback, this));
    ui_timer_ = this->create_wall_timer(1s, std::bind(&MowerLogicNode::updateUI, this));

    // Initialize shared state
    shared_state_ = std::make_shared<SharedState>();
    shared_state_->active_semiautomatic_task = false;

    // Initialize with idle behavior
    current_behavior_ = &IdleBehavior::INSTANCE;

    // Build root actions
    buildRootActions();

    RCLCPP_INFO(this->get_logger(), "Mower Logic Node initialized");
  }

  void run()
  {
    // Wait for required messages
    RCLCPP_INFO(this->get_logger(), "Waiting for emergency message...");
    while (rclcpp::ok() && !emergency_sub_->hasMessage())
    {
      rclcpp::spin_some(this->shared_from_this());
      std::this_thread::sleep_for(100ms);
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for status message...");
    while (rclcpp::ok() && !status_sub_->hasMessage())
    {
      rclcpp::spin_some(this->shared_from_this());
      std::this_thread::sleep_for(100ms);
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for power message...");
    while (rclcpp::ok() && !power_sub_->hasMessage())
    {
      rclcpp::spin_some(this->shared_from_this());
      std::this_thread::sleep_for(100ms);
    }

    RCLCPP_INFO(this->get_logger(), "All required messages received");

    // Register actions
    registerActions("mower_logic", root_actions_);

    // Reset emergency if set
    setEmergencyMode(false);

    RCLCPP_INFO(this->get_logger(), "Starting behavior execution loop");

    // Main behavior loop
    while (rclcpp::ok())
    {
      Behavior* behavior = nullptr;
      {
        std::lock_guard<std::mutex> lock(behavior_mutex_);
        behavior = current_behavior_;
      }
      if (behavior != nullptr)
      {
        behavior->start(config_, shared_state_, this);
        Behavior* next_behavior = behavior->execute();
        behavior->exit();
        {
          std::lock_guard<std::mutex> lock(behavior_mutex_);
          current_behavior_ = next_behavior;
        }
      }
      else
      {
        high_level_status_.state_name = "NULL";
        high_level_status_.state = mower_msgs::msg::HighLevelStatus::HIGH_LEVEL_STATE_NULL;
        high_level_status_pub_->publish(high_level_status_);

        RCLCPP_ERROR(this->get_logger(), "NULL behavior - emergency mode");
        setEmergencyMode(true);
        std::this_thread::sleep_for(1s);
      }

      // Allow callbacks to process
      rclcpp::spin_some(this->shared_from_this());
    }
  }

private:
  void declareParameters()
  {
    this->declare_parameter<bool>("enable_mower", false);
    this->declare_parameter<bool>("manual_pause_mowing", false);
    this->declare_parameter<double>("max_position_accuracy", 0.1);
    this->declare_parameter<double>("gps_timeout", 5.0);
    this->declare_parameter<double>("motor_hot_temperature", 60.0);
    this->declare_parameter<bool>("ignore_gps_errors", false);
    this->declare_parameter<int>("rain_mode", 0);
    this->declare_parameter<int>("rain_check_seconds", 30);
    this->declare_parameter<int>("rain_delay_minutes", 10);
    this->declare_parameter<double>("battery_empty_voltage", 22.0);
    this->declare_parameter<double>("battery_full_voltage", 28.0);
    this->declare_parameter<double>("battery_critical_voltage", 21.0);

    config_.enable_mower = this->get_parameter("enable_mower").as_bool();
    config_.manual_pause_mowing = this->get_parameter("manual_pause_mowing").as_bool();
    config_.max_position_accuracy = this->get_parameter("max_position_accuracy").as_double();
    config_.gps_timeout = this->get_parameter("gps_timeout").as_double();
    config_.motor_hot_temperature = this->get_parameter("motor_hot_temperature").as_double();
    config_.ignore_gps_errors = this->get_parameter("ignore_gps_errors").as_bool();
    config_.rain_mode = this->get_parameter("rain_mode").as_int();
    config_.rain_check_seconds = this->get_parameter("rain_check_seconds").as_int();
    config_.rain_delay_minutes = this->get_parameter("rain_delay_minutes").as_int();

    battery_empty_voltage_ = this->get_parameter("battery_empty_voltage").as_double();
    battery_full_voltage_ = this->get_parameter("battery_full_voltage").as_double();
    battery_critical_voltage_ = this->get_parameter("battery_critical_voltage").as_double();
  }

  void buildRootActions()
  {
    xbot_msgs::msg::ActionInfo reset_emergency;
    reset_emergency.action_id = "reset_emergency";
    reset_emergency.enabled = true;
    reset_emergency.action_name = "Reset Emergency";
    root_actions_.push_back(reset_emergency);
  }

  void registerActions(const std::string& prefix, const std::vector<xbot_msgs::msg::ActionInfo>& actions)
  {
    if (!action_registration_client_->wait_for_service(5s))
    {
      RCLCPP_WARN(this->get_logger(), "Action registration service not available");
      return;
    }

    auto request = std::make_shared<xbot_msgs::srv::RegisterActionsSrv::Request>();
    request->node_prefix = prefix;
    request->actions = actions;

    auto result = action_registration_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Registered actions for %s", prefix.c_str());
  }

  void setEmergencyMode(bool emergency)
  {
    stopBlade();
    stopMoving();

    auto request = std::make_shared<mower_msgs::srv::EmergencyStopSrv::Request>();
    request->emergency = emergency;

    if (!emergency_client_->wait_for_service(1s))
    {
      RCLCPP_ERROR(this->get_logger(), "Emergency service not available");
      return;
    }

    auto result = emergency_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Set emergency mode to %s", emergency ? "true" : "false");
  }

  void setMowerEnabled(bool enabled)
  {
    auto request = std::make_shared<mower_msgs::srv::MowerControlSrv::Request>();
    request->mow_enabled = enabled;

    if (!mow_client_->wait_for_service(1s))
    {
      RCLCPP_ERROR(this->get_logger(), "Mower control service not available");
      return;
    }

    auto result = mow_client_->async_send_request(request);
  }

  void stopMoving()
  {
    geometry_msgs::msg::Twist stop;
    stop.angular.z = 0;
    stop.linear.x = 0;
    cmd_vel_pub_->publish(stop);
  }

  void stopBlade()
  {
    setMowerEnabled(false);
    mower_allowed_.store(false);
  }

  bool isGpsGood()
  {
    auto pose = pose_sub_->getMessage();
    return pose.orientation_valid && pose.position_accuracy < config_.max_position_accuracy &&
           (pose.flags & xbot_msgs::msg::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE);
  }

  void safetyTimerCallback()
  {
    auto emergency = emergency_sub_->getMessage();
    auto status = status_sub_->getMessage();
    auto power = power_sub_->getMessage();
    auto pose = pose_sub_->getMessage();
    auto pose_time = pose_sub_->getMessageTime();
    auto status_time = status_sub_->getMessageTime();
    auto power_time = power_sub_->getMessageTime();

    high_level_status_.emergency = emergency.latched_emergency;
    high_level_status_.is_charging = power.v_charge > 10.0;

    // Handle emergency state
    {
      std::lock_guard<std::mutex> lock(behavior_mutex_);
      if (current_behavior_ != nullptr && emergency.latched_emergency)
      {
        current_behavior_->requestPause(PAUSE_EMERGENCY);
      }
      else if (current_behavior_ != nullptr)
      {
        current_behavior_->requestContinue(PAUSE_EMERGENCY);
      }
    }

    // Check pose timeout
    if ((this->now() - pose_time).seconds() > 1.0)
    {
      stopBlade();
      stopMoving();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Pose values stopped");
      return;
    }

    // Check status timeout
    if ((this->now() - status_time).seconds() > 3.0 || (this->now() - power_time).seconds() > 3.0)
    {
      setEmergencyMode(true);
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Status/power values stopped");
      return;
    }

    // GPS quality
    if (isGpsGood() || config_.ignore_gps_errors)
    {
      last_good_gps_ = this->now();
      gps_initialized_ = true;
      high_level_status_.gps_quality_percent =
          1.0 - std::min(1.0, pose.position_accuracy / config_.max_position_accuracy);
    }
    else
    {
      high_level_status_.gps_quality_percent = pose.orientation_valid ? 0.0 : -1.0;
    }

    bool gps_timeout = gps_initialized_ && (this->now() - last_good_gps_).seconds() > config_.gps_timeout;

    {
      std::lock_guard<std::mutex> lock(behavior_mutex_);
      if (current_behavior_ != nullptr && current_behavior_->needs_gps())
      {
        current_behavior_->setGoodGPS(!gps_timeout);
        if (gps_timeout)
        {
          stopBlade();
          stopMoving();
          return;
        }
      }
    }

    // Battery percentage
    double battery_percent =
        (power.v_battery - battery_empty_voltage_) / (battery_full_voltage_ - battery_empty_voltage_);
    battery_percent = std::clamp(battery_percent, 0.0, 1.0);
    high_level_status_.battery_percent = battery_percent;

    // Enable mower if allowed — only set true when conditions are actually safe
    if (!emergency.latched_emergency && (!gps_timeout || !gps_initialized_))
    {
      mower_allowed_.store(true);
    }

    {
      std::lock_guard<std::mutex> lock(behavior_mutex_);
      setMowerEnabled(current_behavior_ != nullptr && mower_allowed_.load() && current_behavior_->mower_enabled());
    }
  }

  void updateUI()
  {
    std::lock_guard<std::mutex> lock(behavior_mutex_);
    if (current_behavior_)
    {
      high_level_status_.state_name = current_behavior_->state_name();
      high_level_status_.state = current_behavior_->get_state();
      high_level_status_.sub_state_name = current_behavior_->sub_state_name();
    }
    else
    {
      high_level_status_.state_name = "NULL";
      high_level_status_.sub_state_name = "";
      high_level_status_.state = mower_msgs::msg::HighLevelStatus::HIGH_LEVEL_STATE_NULL;
    }
    high_level_status_pub_->publish(high_level_status_);
  }

  void joyVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    joy_vel_time_ = this->now();
    std::lock_guard<std::mutex> lock(behavior_mutex_);
    if (current_behavior_ && current_behavior_->redirect_joystick())
    {
      cmd_vel_pub_->publish(*msg);
    }
  }

  void actionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "mower_logic/reset_emergency")
    {
      RCLCPP_WARN(this->get_logger(), "Got reset emergency action");
      setEmergencyMode(false);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(behavior_mutex_);
      if (current_behavior_)
      {
        current_behavior_->handle_action(msg->data);
      }
    }
  }

  void highLevelControlCallback(const std::shared_ptr<mower_msgs::srv::HighLevelControlSrv::Request> request,
                                std::shared_ptr<mower_msgs::srv::HighLevelControlSrv::Response> /*response*/)
  {
    using ControlRequest = mower_msgs::srv::HighLevelControlSrv::Request;

    {
      std::lock_guard<std::mutex> lock(behavior_mutex_);
      switch (request->command)
      {
        case ControlRequest::COMMAND_HOME:
          RCLCPP_INFO(this->get_logger(), "COMMAND_HOME");
          if (current_behavior_)
            current_behavior_->command_home();
          break;
        case ControlRequest::COMMAND_START:
          RCLCPP_INFO(this->get_logger(), "COMMAND_START");
          if (current_behavior_)
            current_behavior_->command_start();
          break;
        case ControlRequest::COMMAND_S1:
          RCLCPP_INFO(this->get_logger(), "COMMAND_S1");
          if (current_behavior_)
            current_behavior_->command_s1();
          break;
        case ControlRequest::COMMAND_S2:
          RCLCPP_INFO(this->get_logger(), "COMMAND_S2");
          if (current_behavior_)
            current_behavior_->command_s2();
          break;
        case ControlRequest::COMMAND_RESET_EMERGENCY:
        RCLCPP_WARN(this->get_logger(), "COMMAND_RESET_EMERGENCY");
          setEmergencyMode(false);
          break;
      }
    }
  }

  // State subscribers
  std::unique_ptr<StateSubscriber<mower_msgs::msg::Emergency>> emergency_sub_;
  std::unique_ptr<StateSubscriber<mower_msgs::msg::Status>> status_sub_;
  std::unique_ptr<StateSubscriber<mower_msgs::msg::Power>> power_sub_;
  std::unique_ptr<StateSubscriber<mower_msgs::msg::ESCStatus>> left_esc_sub_;
  std::unique_ptr<StateSubscriber<mower_msgs::msg::ESCStatus>> right_esc_sub_;
  std::unique_ptr<StateSubscriber<xbot_msgs::msg::AbsolutePose>> pose_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<mower_msgs::msg::HighLevelStatus>::SharedPtr high_level_status_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;

  // Service clients
  rclcpp::Client<xbot_positioning_msgs::srv::GPSControlSrv>::SharedPtr gps_client_;
  rclcpp::Client<mower_msgs::srv::MowerControlSrv>::SharedPtr mow_client_;
  rclcpp::Client<mower_msgs::srv::EmergencyStopSrv>::SharedPtr emergency_client_;
  rclcpp::Client<xbot_positioning_msgs::srv::SetPoseSrv>::SharedPtr positioning_client_;
  rclcpp::Client<xbot_msgs::srv::RegisterActionsSrv>::SharedPtr action_registration_client_;

  // Services
  rclcpp::Service<mower_msgs::srv::HighLevelControlSrv>::SharedPtr high_level_control_srv_;

  // Timers
  rclcpp::TimerBase::SharedPtr safety_timer_;
  rclcpp::TimerBase::SharedPtr ui_timer_;

  // State
  MowerLogicConfig config_;
  std::shared_ptr<SharedState> shared_state_;
  std::mutex behavior_mutex_;
  Behavior* current_behavior_ = nullptr;
  std::atomic<bool> mower_allowed_{ false };
  rclcpp::Time last_good_gps_;
  rclcpp::Time joy_vel_time_;
  bool gps_initialized_;
  mower_msgs::msg::HighLevelStatus high_level_status_;
  std::vector<xbot_msgs::msg::ActionInfo> root_actions_;

  // Battery config
  double battery_empty_voltage_ = 22.0;
  double battery_full_voltage_ = 28.0;
  double battery_critical_voltage_ = 21.0;
};

}  // namespace mower_logic

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mower_logic::MowerLogicNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
