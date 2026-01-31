// Copyright (c) 2021-2022 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// ROS2 port of slic3r_coverage_planner
// Coverage path planning using Slic3r polygon operations

#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "slic3r_coverage_planner_msgs/srv/plan_path.hpp"
#include "slic3r_coverage_planner_msgs/msg/path.hpp"

// Note: This file requires the Slic3r library to be available
// The following includes would be from Slic3r:
// #include "ExPolygon.hpp"
// #include "Polyline.hpp"
// #include "Fill/FillRectilinear.hpp"
// #include "Fill/FillConcentric.hpp"
// #include "Surface.hpp"
// #include "Fill/FillPlanePath.hpp"
// #include "PerimeterGenerator.hpp"
// #include "ClipperUtils.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace slic3r_coverage_planner {

class CoveragePlannerNode : public rclcpp::Node {
public:
    CoveragePlannerNode() : Node("slic3r_coverage_planner") {
        // Declare parameters
        this->declare_parameter<bool>("visualize_plan", true);
        visualize_plan_ = this->get_parameter("visualize_plan").as_bool();

        // Create publisher for visualization
        if (visualize_plan_) {
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "slic3r_coverage_planner/path_marker_array", 
                rclcpp::QoS(100).transient_local());
        }

        // Create service
        plan_path_srv_ = this->create_service<slic3r_coverage_planner_msgs::srv::PlanPath>(
            "slic3r_coverage_planner/plan_path",
            std::bind(&CoveragePlannerNode::planPathCallback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Slic3r Coverage Planner initialized");
    }

private:
    void planPathCallback(
        const std::shared_ptr<slic3r_coverage_planner_msgs::srv::PlanPath::Request> request,
        std::shared_ptr<slic3r_coverage_planner_msgs::srv::PlanPath::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "Received path planning request with %zu outline points",
                    request->outline.points.size());

        // Note: The actual implementation requires the Slic3r library
        // This is a placeholder that demonstrates the service interface
        // In a complete implementation, the Slic3r polygon operations would be used here

        // For now, create a simple path following the outline
        if (request->outline.points.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Outline too small, need at least 3 points");
            return;
        }

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "map";

        // Create outline path
        if (!request->skip_area_outline) {
            slic3r_coverage_planner_msgs::msg::Path outline_path;
            outline_path.is_outline = true;
            outline_path.path.header = header;

            geometry_msgs::msg::Point32 prev_point = request->outline.points[0];
            for (size_t i = 1; i < request->outline.points.size(); ++i) {
                const auto& pt = request->outline.points[i];
                
                // Calculate orientation
                double dx = pt.x - prev_point.x;
                double dy = pt.y - prev_point.y;
                double orientation = std::atan2(dy, dx);
                
                tf2::Quaternion q;
                q.setRPY(0, 0, orientation);
                
                geometry_msgs::msg::PoseStamped pose;
                pose.header = header;
                pose.pose.position.x = prev_point.x;
                pose.pose.position.y = prev_point.y;
                pose.pose.position.z = 0;
                pose.pose.orientation = tf2::toMsg(q);
                
                outline_path.path.poses.push_back(pose);
                prev_point = pt;
            }

            // Add final pose
            if (!outline_path.path.poses.empty()) {
                geometry_msgs::msg::PoseStamped final_pose;
                final_pose.header = header;
                final_pose.pose.position.x = prev_point.x;
                final_pose.pose.position.y = prev_point.y;
                final_pose.pose.position.z = 0;
                final_pose.pose.orientation = outline_path.path.poses.back().pose.orientation;
                outline_path.path.poses.push_back(final_pose);
            }

            if (!outline_path.path.poses.empty()) {
                response->paths.push_back(outline_path);
            }
        }

        // For a complete implementation, add:
        // 1. Multiple perimeter passes (outline_count parameter)
        // 2. Fill pattern generation (linear or concentric based on fill_type)
        // 3. Hole/obstacle handling
        // 4. Path optimization for efficient coverage

        // Publish visualization if enabled
        if (visualize_plan_ && marker_pub_) {
            visualization_msgs::msg::MarkerArray marker_array;
            
            // Delete all previous markers
            visualization_msgs::msg::Marker delete_marker;
            delete_marker.header.frame_id = "map";
            delete_marker.ns = "coverage_planner";
            delete_marker.id = -1;
            delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(delete_marker);
            
            // Create visualization markers for paths
            int marker_id = 0;
            std::vector<std_msgs::msg::ColorRGBA> colors = getColorPalette();
            
            for (size_t i = 0; i < response->paths.size(); ++i) {
                const auto& path = response->paths[i];
                visualization_msgs::msg::Marker line_marker;
                line_marker.header.frame_id = "map";
                line_marker.header.stamp = this->now();
                line_marker.ns = "coverage_planner";
                line_marker.id = marker_id++;
                line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                line_marker.action = visualization_msgs::msg::Marker::ADD;
                line_marker.scale.x = 0.02;
                line_marker.color = colors[i % colors.size()];
                line_marker.pose.orientation.w = 1.0;

                for (const auto& pose : path.path.poses) {
                    geometry_msgs::msg::Point p;
                    p.x = pose.pose.position.x;
                    p.y = pose.pose.position.y;
                    p.z = 0;
                    line_marker.points.push_back(p);
                }

                marker_array.markers.push_back(line_marker);
            }

            marker_pub_->publish(marker_array);
        }

        RCLCPP_INFO(this->get_logger(), "Generated %zu paths", response->paths.size());
    }

    std::vector<std_msgs::msg::ColorRGBA> getColorPalette() {
        std::vector<std_msgs::msg::ColorRGBA> colors;
        
        auto make_color = [](float r, float g, float b) {
            std_msgs::msg::ColorRGBA c;
            c.r = r; c.g = g; c.b = b; c.a = 1.0;
            return c;
        };

        colors.push_back(make_color(1.0, 0.0, 0.0));  // Red
        colors.push_back(make_color(0.0, 1.0, 0.0));  // Green
        colors.push_back(make_color(0.0, 0.0, 1.0));  // Blue
        colors.push_back(make_color(1.0, 1.0, 0.0));  // Yellow
        colors.push_back(make_color(1.0, 0.0, 1.0));  // Magenta
        colors.push_back(make_color(0.0, 1.0, 1.0));  // Cyan
        colors.push_back(make_color(1.0, 1.0, 1.0));  // White

        return colors;
    }

    bool visualize_plan_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Service<slic3r_coverage_planner_msgs::srv::PlanPath>::SharedPtr plan_path_srv_;
};

}  // namespace slic3r_coverage_planner

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<slic3r_coverage_planner::CoveragePlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
