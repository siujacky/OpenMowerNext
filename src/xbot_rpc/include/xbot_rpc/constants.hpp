/**
 * @file constants.hpp
 * @brief RPC topic and service constants
 *
 * Ported from open_mower_ros (ROS1) to ROS2
 */

#ifndef XBOT_RPC__CONSTANTS_HPP_
#define XBOT_RPC__CONSTANTS_HPP_

namespace xbot_rpc
{

inline constexpr const char* TOPIC_REQUEST = "/xbot/rpc/request";
inline constexpr const char* TOPIC_RESPONSE = "/xbot/rpc/response";
inline constexpr const char* TOPIC_ERROR = "/xbot/rpc/error";
inline constexpr const char* SERVICE_REGISTER_METHODS = "/xbot/rpc/register";

}  // namespace xbot_rpc

#endif  // XBOT_RPC__CONSTANTS_HPP_
