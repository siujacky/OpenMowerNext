// Copyright (c) 2022 Clemens Elflein. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// ROS2 port of xbot_monitoring node
// Provides MQTT bridge for robot state, sensor data, and RPC

#include <filesystem>
#include <memory>
#include <mutex>
#include <regex>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

#include "xbot_msgs/msg/sensor_info.hpp"
#include "xbot_msgs/msg/sensor_data_string.hpp"
#include "xbot_msgs/msg/sensor_data_double.hpp"
#include "xbot_msgs/msg/robot_state.hpp"
#include "xbot_msgs/msg/action_info.hpp"
#include "xbot_msgs/msg/map_overlay.hpp"
#include "xbot_msgs/srv/register_actions_srv.hpp"

#include "xbot_rpc_msgs/msg/rpc_request.hpp"
#include "xbot_rpc_msgs/msg/rpc_response.hpp"
#include "xbot_rpc_msgs/msg/rpc_error.hpp"
#include "xbot_rpc_msgs/srv/register_methods_srv.hpp"

#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>

#include "xbot_monitoring/capabilities.hpp"

using json = nlohmann::ordered_json;
using namespace std::chrono_literals;

namespace xbot_monitoring
{

class XbotMonitoringNode : public rclcpp::Node
{
public:
  XbotMonitoringNode() : Node("xbot_monitoring")
  {
    // Declare parameters
    this->declare_parameter<std::string>("software_version", "UNKNOWN VERSION");
    this->declare_parameter<bool>("external_mqtt_enable", false);
    this->declare_parameter<std::string>("external_mqtt_topic_prefix", "");
    this->declare_parameter<std::string>("external_mqtt_hostname", "");
    this->declare_parameter<int>("external_mqtt_port", 1883);
    this->declare_parameter<std::string>("external_mqtt_username", "");
    this->declare_parameter<std::string>("external_mqtt_password", "");

    // Get parameters
    version_string_ = this->get_parameter("software_version").as_string();
    if (version_string_.empty())
    {
      version_string_ = "UNKNOWN VERSION";
    }

    external_mqtt_enable_ = this->get_parameter("external_mqtt_enable").as_bool();
    external_mqtt_topic_prefix_ = this->get_parameter("external_mqtt_topic_prefix").as_string();
    if (!external_mqtt_topic_prefix_.empty() && external_mqtt_topic_prefix_.back() != '/')
    {
      external_mqtt_topic_prefix_ += "/";
    }
    external_mqtt_hostname_ = this->get_parameter("external_mqtt_hostname").as_string();
    external_mqtt_port_ = std::to_string(this->get_parameter("external_mqtt_port").as_int());
    external_mqtt_username_ = this->get_parameter("external_mqtt_username").as_string();
    external_mqtt_password_ = this->get_parameter("external_mqtt_password").as_string();

    if (external_mqtt_enable_)
    {
      RCLCPP_INFO(this->get_logger(), "Using external MQTT broker: %s:%s with topic prefix: %s",
                  external_mqtt_hostname_.c_str(), external_mqtt_port_.c_str(), external_mqtt_topic_prefix_.c_str());
    }

    // Setup MQTT
    setup_mqtt_client();

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("xbot_monitoring/remote_cmd_vel", 1);
    action_pub_ = this->create_publisher<std_msgs::msg::String>("xbot/action", 1);
    rpc_request_pub_ = this->create_publisher<xbot_rpc_msgs::msg::RpcRequest>("xbot_rpc/request", 100);

    // Subscribers
    robot_state_sub_ = this->create_subscription<xbot_msgs::msg::RobotState>(
        "xbot_monitoring/robot_state", 10,
        std::bind(&XbotMonitoringNode::robot_state_callback, this, std::placeholders::_1));

    map_sub_ = this->create_subscription<std_msgs::msg::String>(
        "mower_map_service/json_map", 10, std::bind(&XbotMonitoringNode::map_callback, this, std::placeholders::_1));

    map_overlay_sub_ = this->create_subscription<xbot_msgs::msg::MapOverlay>(
        "xbot_monitoring/map_overlay", 10,
        std::bind(&XbotMonitoringNode::map_overlay_callback, this, std::placeholders::_1));

    rpc_response_sub_ = this->create_subscription<xbot_rpc_msgs::msg::RpcResponse>(
        "xbot_rpc/response", 100, std::bind(&XbotMonitoringNode::rpc_response_callback, this, std::placeholders::_1));

    rpc_error_sub_ = this->create_subscription<xbot_rpc_msgs::msg::RpcError>(
        "xbot_rpc/error", 100, std::bind(&XbotMonitoringNode::rpc_error_callback, this, std::placeholders::_1));

    // Services
    register_actions_service_ = this->create_service<xbot_msgs::srv::RegisterActionsSrv>(
        "xbot/register_actions",
        std::bind(&XbotMonitoringNode::register_actions_callback, this, std::placeholders::_1, std::placeholders::_2));

    register_methods_service_ = this->create_service<xbot_rpc_msgs::srv::RegisterMethodsSrv>(
        "xbot_rpc/register_methods",
        std::bind(&XbotMonitoringNode::register_methods_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Timer for sensor discovery
    sensor_check_timer_ = this->create_wall_timer(100ms, std::bind(&XbotMonitoringNode::sensor_check_callback, this));

    RCLCPP_INFO(this->get_logger(), "XBot Monitoring Node initialized");
  }

  ~XbotMonitoringNode()
  {
    if (client_)
    {
      try
      {
        client_->disconnect()->wait();
      }
      catch (...)
      {
      }
    }
    if (client_external_)
    {
      try
      {
        client_external_->disconnect()->wait();
      }
      catch (...)
      {
      }
    }
  }

private:
  // MQTT callback class
  class MqttCallback : public mqtt::callback
  {
  public:
    void set_node(XbotMonitoringNode* node, const std::string& topic_prefix)
    {
      node_ = node;
      topic_prefix_ = topic_prefix;
    }

    void connected(const mqtt::string& /*cause*/) override
    {
      RCLCPP_INFO(node_->get_logger(), "MQTT Connected");
      node_->publish_capabilities();
      node_->publish_sensor_metadata();
      node_->publish_map();
      node_->publish_map_overlay();
      node_->publish_actions();
      node_->publish_version();

      // Subscribe to control topics
      node_->client_->subscribe(topic_prefix_ + "teleop", 0);
      node_->client_->subscribe(topic_prefix_ + "command", 0);
      node_->client_->subscribe(topic_prefix_ + "action", 0);
      node_->client_->subscribe(topic_prefix_ + "rpc/request", 0);
    }

    void message_arrived(mqtt::const_message_ptr msg) override
    {
      std::string topic = msg->get_topic();

      if (topic == topic_prefix_ + "teleop")
      {
        try
        {
          json j = json::from_bson(msg->get_payload().begin(), msg->get_payload().end());
          geometry_msgs::msg::Twist t;
          t.linear.x = j["vx"];
          t.angular.z = j["vz"];
          node_->cmd_vel_pub_->publish(t);
        }
        catch (const json::exception& e)
        {
          RCLCPP_ERROR(node_->get_logger(), "Error decoding teleop bson: %s", e.what());
        }
      }
      else if (topic == topic_prefix_ + "action")
      {
        RCLCPP_INFO(node_->get_logger(), "Got action: %s", msg->get_payload_str().c_str());
        std_msgs::msg::String action_msg;
        action_msg.data = msg->get_payload_str();
        node_->action_pub_->publish(action_msg);
      }
      else if (topic == topic_prefix_ + "rpc/request")
      {
        node_->rpc_request_callback(msg->get_payload_str());
      }
    }

  private:
    XbotMonitoringNode* node_ = nullptr;
    std::string topic_prefix_;
  };

  void setup_mqtt_client()
  {
    // Setup internal MQTT client
    mqtt::connect_options connect_options;
    connect_options.set_automatic_reconnect(true);
    connect_options.set_clean_session(true);
    connect_options.set_keep_alive_interval(1000);

    std::string uri = "tcp://127.0.0.1:1883";
    try
    {
      client_ = std::make_shared<mqtt::async_client>(uri, "xbot_monitoring");
      mqtt_callback_.set_node(this, "");
      client_->set_callback(mqtt_callback_);
      client_->connect(connect_options);
    }
    catch (const mqtt::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "MQTT client could not be initialized: %s", e.what());
    }

    // Setup external MQTT client if enabled
    if (external_mqtt_enable_)
    {
      mqtt::connect_options ext_connect_options;
      ext_connect_options.set_automatic_reconnect(true);
      ext_connect_options.set_clean_session(true);
      ext_connect_options.set_keep_alive_interval(1000);

      if (!external_mqtt_username_.empty())
      {
        ext_connect_options.set_user_name(external_mqtt_username_);
        ext_connect_options.set_password(external_mqtt_password_);
      }

      std::string ext_uri = "tcp://" + external_mqtt_hostname_ + ":" + external_mqtt_port_;
      try
      {
        client_external_ = std::make_shared<mqtt::async_client>(ext_uri, "ext_xbot_monitoring");
        mqtt_callback_external_.set_node(this, external_mqtt_topic_prefix_);
        client_external_->set_callback(mqtt_callback_external_);
        client_external_->connect(ext_connect_options);
      }
      catch (const mqtt::exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "External MQTT client could not be initialized: %s", e.what());
      }
    }
  }

  void try_publish(const std::string& topic, const std::string& data, bool retain = false)
  {
    try
    {
      if (client_ && client_->is_connected())
      {
        client_->publish(topic, data, retain ? 1 : 0, retain);
      }
    }
    catch (const mqtt::exception& e)
    {
      // Client disconnected, drop message
    }

    if (external_mqtt_enable_ && client_external_)
    {
      try
      {
        if (client_external_->is_connected())
        {
          client_external_->publish(external_mqtt_topic_prefix_ + topic, data, retain ? 1 : 0, retain);
        }
      }
      catch (const mqtt::exception& e)
      {
        // Client disconnected, drop message
      }
    }
  }

  void try_publish_binary(const std::string& topic, const void* data, size_t size, bool retain = false)
  {
    try
    {
      if (client_ && client_->is_connected())
      {
        client_->publish(topic, data, size, retain ? 1 : 0, retain);
      }
    }
    catch (const mqtt::exception& e)
    {
      // Client disconnected, drop message
    }
  }

  void publish_version()
  {
    json version = { { "version", version_string_ } };
    try_publish("version/json", version.dump(), true);
    auto bson = json::to_bson(version);
    try_publish_binary("version", bson.data(), bson.size(), true);
  }

  void publish_capabilities()
  {
    try_publish("capabilities/json", CAPABILITIES.dump(2), true);
  }

  void publish_sensor_metadata()
  {
    std::lock_guard<std::mutex> lk(mqtt_callback_mutex_);
    if (found_sensors_.empty())
      return;

    json sensor_info;
    for (const auto& [topic, info] : found_sensors_)
    {
      json j;
      j["sensor_id"] = info.sensor_id;
      j["sensor_name"] = info.sensor_name;

      switch (info.value_type)
      {
        case xbot_msgs::msg::SensorInfo::TYPE_STRING:
          j["value_type"] = "STRING";
          break;
        case xbot_msgs::msg::SensorInfo::TYPE_DOUBLE:
          j["value_type"] = "DOUBLE";
          break;
        default:
          j["value_type"] = "UNKNOWN";
          break;
      }

      switch (info.value_description)
      {
        case xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE:
          j["value_description"] = "TEMPERATURE";
          break;
        case xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_VELOCITY:
          j["value_description"] = "VELOCITY";
          break;
        case xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_VOLTAGE:
          j["value_description"] = "VOLTAGE";
          break;
        case xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_CURRENT:
          j["value_description"] = "CURRENT";
          break;
        case xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_PERCENT:
          j["value_description"] = "PERCENT";
          break;
        default:
          j["value_description"] = "UNKNOWN";
          break;
      }

      j["unit"] = info.unit;
      j["has_min_max"] = info.has_min_max;
      j["min_value"] = info.min_value;
      j["max_value"] = info.max_value;
      j["has_critical_low"] = info.has_critical_low;
      j["lower_critical_value"] = info.lower_critical_value;
      j["has_critical_high"] = info.has_critical_high;
      j["upper_critical_value"] = info.upper_critical_value;

      sensor_info.push_back(j);
    }

    try_publish("sensor_infos/json", sensor_info.dump(), true);
    json data;
    data["d"] = sensor_info;
    auto bson = json::to_bson(data);
    try_publish_binary("sensor_infos/bson", bson.data(), bson.size(), true);
  }

  void publish_actions()
  {
    json actions = json::array();
    for (const auto& [prefix, action_list] : registered_actions_)
    {
      for (const auto& action : action_list)
      {
        json action_info;
        action_info["action_id"] = prefix + "/" + action.action_id;
        action_info["action_name"] = action.action_name;
        action_info["enabled"] = action.enabled;
        actions.push_back(action_info);
      }
    }

    try_publish("actions/json", actions.dump(), true);
    json data;
    data["d"] = actions;
    auto bson = json::to_bson(data);
    try_publish_binary("actions/bson", bson.data(), bson.size(), true);
  }

  void publish_map()
  {
    if (!has_map_)
      return;
    try_publish("map/json", map_.dump(2), true);
    json data;
    data["d"] = map_;
    auto bson = json::to_bson(data);
    try_publish_binary("map/bson", bson.data(), bson.size(), true);
  }

  void publish_map_overlay()
  {
    if (!has_map_overlay_)
      return;
    try_publish("map_overlay/json", map_overlay_.dump(), true);
    json data;
    data["d"] = map_overlay_;
    auto bson = json::to_bson(data);
    try_publish_binary("map_overlay/bson", bson.data(), bson.size(), true);
  }

  void robot_state_callback(const xbot_msgs::msg::RobotState::SharedPtr msg)
  {
    json j;
    j["battery_percentage"] = msg->battery_percentage;
    j["gps_percentage"] = msg->gps_percentage;
    j["current_action_progress"] = msg->current_action_progress;
    j["current_state"] = msg->current_state;
    j["current_sub_state"] = msg->current_sub_state;
    j["current_area"] = msg->current_area;
    j["current_path"] = msg->current_path;
    j["current_path_index"] = msg->current_path_index;
    j["emergency"] = msg->emergency;
    j["is_charging"] = msg->is_charging;
    j["rain_detected"] = msg->rain_detected;
    j["pose"]["x"] = msg->robot_pose.pose.pose.position.x;
    j["pose"]["y"] = msg->robot_pose.pose.pose.position.y;
    j["pose"]["heading"] = msg->robot_pose.vehicle_heading;
    j["pose"]["pos_accuracy"] = msg->robot_pose.position_accuracy;
    j["pose"]["heading_accuracy"] = msg->robot_pose.orientation_accuracy;
    j["pose"]["heading_valid"] = msg->robot_pose.orientation_valid;

    try_publish("robot_state/json", j.dump());
    json data;
    data["d"] = j;
    auto bson = json::to_bson(data);
    try_publish_binary("robot_state/bson", bson.data(), bson.size());
  }

  void map_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    try
    {
      map_ = json::parse(msg->data);
      has_map_ = true;
      publish_map();
    }
    catch (const json::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error processing map JSON: %s", e.what());
    }
  }

  void map_overlay_callback(const xbot_msgs::msg::MapOverlay::SharedPtr msg)
  {
    json polys;
    for (const auto& poly : msg->polygons)
    {
      if (poly.polygon.points.size() < 2)
        continue;

      json poly_j;
      json outline_poly_j;
      for (const auto& pt : poly.polygon.points)
      {
        json p_j;
        p_j["x"] = pt.x;
        p_j["y"] = pt.y;
        outline_poly_j.push_back(p_j);
      }
      poly_j["poly"] = outline_poly_j;
      poly_j["is_closed"] = poly.closed;
      poly_j["line_width"] = poly.line_width;
      poly_j["color"] = poly.color;
      polys.push_back(poly_j);
    }

    json j;
    j["polygons"] = polys;
    map_overlay_ = j;
    has_map_overlay_ = true;
    publish_map_overlay();
  }

  void register_actions_callback(const std::shared_ptr<xbot_msgs::srv::RegisterActionsSrv::Request> request,
                                 std::shared_ptr<xbot_msgs::srv::RegisterActionsSrv::Response> /*response*/)
  {
    RCLCPP_INFO(this->get_logger(), "New actions registered: %s registered %zu actions", request->node_prefix.c_str(),
                request->actions.size());
    registered_actions_[request->node_prefix] = request->actions;
    publish_actions();
  }

  void register_methods_callback(const std::shared_ptr<xbot_rpc_msgs::srv::RegisterMethodsSrv::Request> request,
                                 std::shared_ptr<xbot_rpc_msgs::srv::RegisterMethodsSrv::Response> /*response*/)
  {
    std::lock_guard<std::mutex> lk(registered_methods_mutex_);
    registered_methods_[request->node_id] = request->methods;
    RCLCPP_INFO(this->get_logger(), "New methods registered: %s registered %zu methods", request->node_id.c_str(),
                request->methods.size());
  }

  void rpc_publish_error(int16_t code, const std::string& message, const json& id = nullptr)
  {
    json err_resp = { { "jsonrpc", "2.0" }, { "error", { { "code", code }, { "message", message } } }, { "id", id } };
    try_publish("rpc/response", err_resp.dump(2));
  }

  void rpc_request_callback(const std::string& payload)
  {
    // RPC error codes
    constexpr int16_t ERROR_INVALID_JSON = -32700;
    constexpr int16_t ERROR_INVALID_REQUEST = -32600;
    constexpr int16_t ERROR_METHOD_NOT_FOUND = -32601;

    json req;
    try
    {
      req = json::parse(payload);
    }
    catch (const json::parse_error& e)
    {
      return rpc_publish_error(ERROR_INVALID_JSON, "Could not parse request JSON");
    }

    if (!req.is_object())
    {
      return rpc_publish_error(ERROR_INVALID_REQUEST, "Request is not a JSON object");
    }

    json id = req.contains("id") ? req["id"] : nullptr;
    if (id != nullptr && !id.is_string())
    {
      return rpc_publish_error(ERROR_INVALID_REQUEST, "ID is not a string", id);
    }
    else if (!req.contains("jsonrpc") || !req["jsonrpc"].is_string() || req["jsonrpc"] != "2.0")
    {
      return rpc_publish_error(ERROR_INVALID_REQUEST, "Invalid JSON-RPC version");
    }
    else if (!req.contains("method") || !req["method"].is_string())
    {
      return rpc_publish_error(ERROR_INVALID_REQUEST, "Method is not a string", req["id"]);
    }

    const std::string method = req["method"];

    // Check if method is registered
    bool is_registered = false;
    {
      std::lock_guard<std::mutex> lk(registered_methods_mutex_);
      for (const auto& [_, method_ids] : registered_methods_)
      {
        if (std::find(method_ids.begin(), method_ids.end(), method) != method_ids.end())
        {
          is_registered = true;
          break;
        }
      }
    }

    if (!is_registered)
    {
      return rpc_publish_error(ERROR_METHOD_NOT_FOUND, "Method \"" + method + "\" not found", req["id"]);
    }

    // Forward to providers
    xbot_rpc_msgs::msg::RpcRequest msg;
    msg.method = method;
    msg.params = req.contains("params") ? req["params"].dump() : "";
    msg.id = id != nullptr ? id.get<std::string>() : "";
    rpc_request_pub_->publish(msg);
  }

  void rpc_response_callback(const xbot_rpc_msgs::msg::RpcResponse::SharedPtr msg)
  {
    json result;
    try
    {
      result = json::parse(msg->result);
    }
    catch (const json::parse_error& e)
    {
      return rpc_publish_error(-32603, "Internal error while parsing result JSON: " + std::string(e.what()), msg->id);
    }

    json j = { { "jsonrpc", "2.0" }, { "result", result }, { "id", msg->id } };
    try_publish("rpc/response", j.dump(2));
  }

  void rpc_error_callback(const xbot_rpc_msgs::msg::RpcError::SharedPtr msg)
  {
    rpc_publish_error(msg->code, msg->message, msg->id);
  }

  void sensor_check_callback()
  {
    // In ROS2, we would use topic discovery differently
    // For now, this is a placeholder - actual implementation would use
    // get_topic_names_and_types() and subscribe dynamically
  }

  // Members
  std::string version_string_;
  bool external_mqtt_enable_ = false;
  std::string external_mqtt_topic_prefix_;
  std::string external_mqtt_hostname_;
  std::string external_mqtt_port_;
  std::string external_mqtt_username_;
  std::string external_mqtt_password_;

  std::shared_ptr<mqtt::async_client> client_;
  std::shared_ptr<mqtt::async_client> client_external_;
  MqttCallback mqtt_callback_;
  MqttCallback mqtt_callback_external_;

  std::mutex mqtt_callback_mutex_;
  std::map<std::string, xbot_msgs::msg::SensorInfo> found_sensors_;
  std::map<std::string, std::vector<xbot_msgs::msg::ActionInfo>> registered_actions_;
  std::map<std::string, std::vector<std::string>> registered_methods_;
  std::mutex registered_methods_mutex_;

  json map_;
  json map_overlay_;
  bool has_map_ = false;
  bool has_map_overlay_ = false;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_pub_;
  rclcpp::Publisher<xbot_rpc_msgs::msg::RpcRequest>::SharedPtr rpc_request_pub_;

  // Subscribers
  rclcpp::Subscription<xbot_msgs::msg::RobotState>::SharedPtr robot_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_sub_;
  rclcpp::Subscription<xbot_msgs::msg::MapOverlay>::SharedPtr map_overlay_sub_;
  rclcpp::Subscription<xbot_rpc_msgs::msg::RpcResponse>::SharedPtr rpc_response_sub_;
  rclcpp::Subscription<xbot_rpc_msgs::msg::RpcError>::SharedPtr rpc_error_sub_;

  // Services
  rclcpp::Service<xbot_msgs::srv::RegisterActionsSrv>::SharedPtr register_actions_service_;
  rclcpp::Service<xbot_rpc_msgs::srv::RegisterMethodsSrv>::SharedPtr register_methods_service_;

  // Timers
  rclcpp::TimerBase::SharedPtr sensor_check_timer_;
};

}  // namespace xbot_monitoring

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xbot_monitoring::XbotMonitoringNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
