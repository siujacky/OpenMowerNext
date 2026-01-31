// Copyright (c) 2022-2024 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: GPL-3.0
//
// ROS2 port of StateSubscriber

#ifndef MOWER_LOGIC__STATE_SUBSCRIBER_HPP_
#define MOWER_LOGIC__STATE_SUBSCRIBER_HPP_

#include <mutex>
#include <string>
#include "rclcpp/rclcpp.hpp"

namespace mower_logic
{

template <typename MessageT>
class StateSubscriber
{
public:
  explicit StateSubscriber(rclcpp::Node* node, const std::string& topic, size_t qos_depth = 10)
    : node_(node), topic_(topic), qos_depth_(qos_depth)
  {
  }

  void start()
  {
    subscriber_ = node_->create_subscription<MessageT>(
        topic_, qos_depth_, [this](const typename MessageT::SharedPtr msg) { setMessage(*msg); });
  }

  MessageT getMessage() const
  {
    std::lock_guard<std::mutex> lk(message_mutex_);
    return message_;
  }

  bool hasMessage() const
  {
    std::lock_guard<std::mutex> lk(message_mutex_);
    return has_message_;
  }

  rclcpp::Time getMessageTime() const
  {
    std::lock_guard<std::mutex> lk(message_mutex_);
    return last_message_time_;
  }

  void setMessage(const MessageT& message)
  {
    std::lock_guard<std::mutex> lk(message_mutex_);
    last_message_time_ = node_->now();
    message_ = message;
    has_message_ = true;
  }

private:
  rclcpp::Node* node_;
  std::string topic_;
  size_t qos_depth_;
  mutable std::mutex message_mutex_;
  MessageT message_{};
  rclcpp::Time last_message_time_;
  bool has_message_ = false;
  typename rclcpp::Subscription<MessageT>::SharedPtr subscriber_;
};

}  // namespace mower_logic

#endif  // MOWER_LOGIC__STATE_SUBSCRIBER_HPP_
