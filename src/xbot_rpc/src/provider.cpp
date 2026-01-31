/**
 * @file provider.cpp
 * @brief RPC Provider implementation for ROS2
 *
 * Ported from open_mower_ros (ROS1) to ROS2
 */

#include "xbot_rpc/provider.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace xbot_rpc
{

void RpcProvider::init(rclcpp::Node::SharedPtr node)
{
  node_ = node;

  // Use TCP no delay for low latency
  auto qos = rclcpp::QoS(100).reliable();

  request_sub_ = node_->create_subscription<xbot_rpc_msgs::msg::RpcRequest>(
      TOPIC_REQUEST, qos, std::bind(&RpcProvider::handleRequest, this, std::placeholders::_1));

  response_pub_ = node_->create_publisher<xbot_rpc_msgs::msg::RpcResponse>(TOPIC_RESPONSE, qos);

  error_pub_ = node_->create_publisher<xbot_rpc_msgs::msg::RpcError>(TOPIC_ERROR, qos);

  registration_client_ = node_->create_client<xbot_rpc_msgs::srv::RegisterMethodsSrv>(SERVICE_REGISTER_METHODS);

  // Wait for registration service
  if (!registration_client_->wait_for_service(10s))
  {
    RCLCPP_WARN(node_->get_logger(), "Registration service not available after waiting");
  }

  publishMethods();
}

void RpcProvider::publishMethods()
{
  auto request = std::make_shared<xbot_rpc_msgs::srv::RegisterMethodsSrv::Request>();
  request->node_id = node_id_;
  request->methods.reserve(methods_.size());

  for (const auto& [method_id, _] : methods_)
  {
    request->methods.push_back(method_id);
  }

  auto future = registration_client_->async_send_request(request);

  // Don't block, just log if it fails
  if (future.wait_for(5s) != std::future_status::ready)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error registering methods for %s", node_id_.c_str());
  }
}

void RpcProvider::handleRequest(const xbot_rpc_msgs::msg::RpcRequest::SharedPtr request)
{
  // Look up the method. Ignore if not found, as it might be handled by another node.
  auto it = methods_.find(request->method);
  if (it == methods_.end())
  {
    return;
  }

  // Parse the parameters.
  nlohmann::basic_json<> params;
  if (!request->params.empty())
  {
    try
    {
      params = nlohmann::ordered_json::parse(request->params);
    }
    catch (const nlohmann::json::parse_error& e)
    {
      publishError(request, xbot_rpc_msgs::msg::RpcError::ERROR_INVALID_JSON,
                   std::string("Invalid parameters JSON: ") + e.what());
      return;
    }
  }

  // Execute the method callback and publish the response.
  try
  {
    nlohmann::basic_json<> response = it->second(request->method, params);
    publishResponse(request, response);
  }
  catch (const RpcException& e)
  {
    publishError(request, e.code, e.message);
  }
  catch (const std::exception& e)
  {
    publishError(request, xbot_rpc_msgs::msg::RpcError::ERROR_INTERNAL, std::string("Internal error: ") + e.what());
  }
}

void RpcProvider::publishResponse(const xbot_rpc_msgs::msg::RpcRequest::SharedPtr request,
                                  const nlohmann::basic_json<>& response)
{
  if (request->id.empty())
  {
    return;
  }

  xbot_rpc_msgs::msg::RpcResponse response_msg;
  response_msg.result = response.dump();
  response_msg.id = request->id;
  response_pub_->publish(response_msg);
}

void RpcProvider::publishError(const xbot_rpc_msgs::msg::RpcRequest::SharedPtr request, int16_t code,
                               const std::string& message)
{
  if (request->id.empty())
  {
    return;
  }

  xbot_rpc_msgs::msg::RpcError err_msg;
  err_msg.id = request->id;
  err_msg.code = code;
  err_msg.message = message;
  error_pub_->publish(err_msg);
}

}  // namespace xbot_rpc
