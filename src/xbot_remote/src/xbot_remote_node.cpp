// Copyright (c) 2022 Clemens Elflein. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// ROS2 port of xbot_remote
// WebSocket server for remote control

#include <memory>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <nlohmann/json.hpp>

// WebSocket++ includes
#include <websocketpp/server.hpp>
#include <websocketpp/config/asio_no_tls.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;

typedef websocketpp::server<websocketpp::config::asio> WsServer;
typedef WsServer::message_ptr MessagePtr;

namespace xbot_remote
{

class XbotRemoteNode : public rclcpp::Node
{
public:
  XbotRemoteNode() : Node("xbot_remote"), running_(true)
  {
    // Declare parameters
    this->declare_parameter<int>("port", 9002);
    port_ = this->get_parameter("port").as_int();

    // Create publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("xbot_remote/cmd_vel", 1);

    // Start websocket server in separate thread
    server_thread_ = std::thread(&XbotRemoteNode::runServer, this);

    RCLCPP_INFO(this->get_logger(), "XBot Remote Node initialized on port %d", port_);
  }

  ~XbotRemoteNode()
  {
    running_ = false;

    try
    {
      server_.stop_listening();
      server_.stop();
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(this->get_logger(), "Error stopping server: %s", e.what());
    }

    if (server_thread_.joinable())
    {
      server_thread_.join();
    }
  }

private:
  void onMessage(websocketpp::connection_hdl /*hdl*/, MessagePtr msg)
  {
    try
    {
      // Try to decode as BSON
      const auto& payload = msg->get_payload();
      json j = json::from_bson(payload.begin(), payload.end());

      geometry_msgs::msg::Twist twist;
      twist.linear.x = j["vx"].get<double>();
      twist.angular.z = j["vz"].get<double>();

      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500, "vx: %f, vz: %f", twist.linear.x,
                            twist.angular.z);

      cmd_vel_pub_->publish(twist);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception during remote decoding: %s", e.what());
    }
  }

  void runServer()
  {
    try
    {
      // Set logging settings
      server_.set_access_channels(websocketpp::log::alevel::all);
      server_.clear_access_channels(websocketpp::log::alevel::frame_payload);

      // Initialize Asio
      server_.init_asio();

      // Register message handler
      server_.set_message_handler(
          std::bind(&XbotRemoteNode::onMessage, this, std::placeholders::_1, std::placeholders::_2));

      server_.set_reuse_addr(true);

      // Listen on port
      server_.listen(port_);

      // Start accept loop
      server_.start_accept();

      RCLCPP_INFO(this->get_logger(), "WebSocket server listening on port %d", port_);

      // Run the server
      while (running_)
      {
        try
        {
          server_.run_one();
        }
        catch (const std::exception& e)
        {
          if (running_)
          {
            RCLCPP_ERROR(this->get_logger(), "Server error: %s", e.what());
          }
        }
      }
    }
    catch (const websocketpp::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "WebSocket exception: %s", e.what());
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }

    RCLCPP_INFO(this->get_logger(), "WebSocket server stopped");
  }

  int port_;
  std::atomic<bool> running_;
  WsServer server_;
  std::thread server_thread_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

}  // namespace xbot_remote

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xbot_remote::XbotRemoteNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
