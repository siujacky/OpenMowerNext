// Copyright (c) 2022 Clemens Elflein. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// ROS2 port of xbot_sensor_example node
// Example node showing how to register and publish sensor data

#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "xbot_msgs/msg/sensor_info.hpp"
#include "xbot_msgs/msg/sensor_data_double.hpp"

using namespace std::chrono_literals;

namespace xbot_monitoring {

class XbotSensorExampleNode : public rclcpp::Node {
public:
    XbotSensorExampleNode() : Node("xbot_sensor_example"), counter_(0) {
        // Setup sensor info
        sensor_info_.has_critical_high = false;
        sensor_info_.has_critical_low = false;
        sensor_info_.has_min_max = true;
        sensor_info_.min_value = -5.0;
        sensor_info_.max_value = 105.0;
        sensor_info_.value_type = xbot_msgs::msg::SensorInfo::TYPE_DOUBLE;
        sensor_info_.value_description = xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE;
        sensor_info_.unit = "deg C";
        sensor_info_.sensor_id = "some_temperature";
        sensor_info_.sensor_name = "Some Temperature";

        // Create publishers
        sensor_info_pub_ = this->create_publisher<xbot_msgs::msg::SensorInfo>(
            "xbot_monitoring/sensors/" + sensor_info_.sensor_id + "/info", 
            rclcpp::QoS(1).transient_local());
        
        sensor_data_pub_ = this->create_publisher<xbot_msgs::msg::SensorDataDouble>(
            "xbot_monitoring/sensors/" + sensor_info_.sensor_id + "/data", 1);

        // Publish sensor info once
        sensor_info_pub_->publish(sensor_info_);

        // Timer to publish sensor data
        timer_ = this->create_wall_timer(
            1s, std::bind(&XbotSensorExampleNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "XBot Sensor Example Node initialized");
    }

private:
    void timer_callback() {
        xbot_msgs::msg::SensorDataDouble data;
        data.stamp = this->now();
        data.data = (std::sin(counter_ / 10.0) + 0.5) * 50.0;
        counter_++;

        sensor_data_pub_->publish(data);
    }

    xbot_msgs::msg::SensorInfo sensor_info_;
    int counter_;

    rclcpp::Publisher<xbot_msgs::msg::SensorInfo>::SharedPtr sensor_info_pub_;
    rclcpp::Publisher<xbot_msgs::msg::SensorDataDouble>::SharedPtr sensor_data_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace xbot_monitoring

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<xbot_monitoring::XbotSensorExampleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
