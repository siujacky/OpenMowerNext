/**
 * @file provider.hpp
 * @brief RPC Provider class for ROS2
 *
 * Ported from open_mower_ros (ROS1) to ROS2
 */

#ifndef XBOT_RPC__PROVIDER_HPP_
#define XBOT_RPC__PROVIDER_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

#include <xbot_rpc_msgs/msg/rpc_error.hpp>
#include <xbot_rpc_msgs/msg/rpc_request.hpp>
#include <xbot_rpc_msgs/msg/rpc_response.hpp>
#include <xbot_rpc_msgs/srv/register_methods_srv.hpp>

#include "xbot_rpc/constants.hpp"

#define RPC_METHOD(id, body)                                                                                           \
  {                                                                                                                    \
    id, [](const std::string& method, const nlohmann::basic_json<>& params) body                                       \
  }

namespace xbot_rpc
{

using callback_t =
    std::function<nlohmann::basic_json<>(const std::string& method, const nlohmann::basic_json<>& params)>;

/**
 * @brief Exception thrown by RPC methods
 */
class RpcException : public std::exception
{
public:
  const int16_t code;
  const std::string message;

  RpcException(int16_t code, const std::string& message) : code(code), message(message)
  {
  }

  const char* what() const noexcept override
  {
    return message.c_str();
  }
};

/**
 * @brief RPC Provider for handling JSON-RPC requests
 */
class RpcProvider
{
public:
  /**
   * @brief Constructor
   * @param node_id Unique identifier for this node
   * @param methods Initial map of method handlers
   */
  explicit RpcProvider(const std::string& node_id, const std::map<std::string, callback_t>& methods = {})
    : node_id_(node_id), methods_(methods)
  {
  }

  /**
   * @brief Initialize the provider with a ROS2 node
   * @param node Shared pointer to the ROS2 node
   */
  void init(rclcpp::Node::SharedPtr node);

  /**
   * @brief Add a method handler
   * @param id Method identifier
   * @param callback Handler function
   */
  void addMethod(const std::string& id, callback_t callback)
  {
    methods_.emplace(id, callback);
  }

  /**
   * @brief Add a method handler
   * @param method Pair of method ID and callback
   */
  void addMethod(const std::pair<std::string, callback_t>& method)
  {
    methods_.insert(method);
  }

  /**
   * @brief Publish registered methods to the registration service
   */
  void publishMethods();

private:
  void handleRequest(const xbot_rpc_msgs::msg::RpcRequest::SharedPtr request);
  void publishResponse(const xbot_rpc_msgs::msg::RpcRequest::SharedPtr request, const nlohmann::basic_json<>& response);
  void publishError(const xbot_rpc_msgs::msg::RpcRequest::SharedPtr request, int16_t code, const std::string& message);

  std::string node_id_;
  std::map<std::string, callback_t> methods_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<xbot_rpc_msgs::msg::RpcRequest>::SharedPtr request_sub_;
  rclcpp::Publisher<xbot_rpc_msgs::msg::RpcResponse>::SharedPtr response_pub_;
  rclcpp::Publisher<xbot_rpc_msgs::msg::RpcError>::SharedPtr error_pub_;
  rclcpp::Client<xbot_rpc_msgs::srv::RegisterMethodsSrv>::SharedPtr registration_client_;
};

}  // namespace xbot_rpc

#endif  // XBOT_RPC__PROVIDER_HPP_
