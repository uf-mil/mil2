#pragma once
#include <mutex>
#include <optional>

#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>

#include "std_srvs/srv/set_bool.hpp"
#include "subjugator_msgs/msg/thruster_efforts.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

struct Context
{
    rclcpp::Node::SharedPtr node;

    // Publishers and Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goal_pub;
    rclcpp::Publisher<subjugator_msgs::msg::ThrusterEfforts>::SharedPtr raw_effort_pub;  // no mutex since im lazy and
                                                                                         // no-one else using this rn :P
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr targets_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

    // services and clients
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr controller_enable_client;

    // Latest state
    std::mutex odom_mx;
    std::optional<nav_msgs::msg::Odometry> latest_odom;

    std::mutex last_goal_mx;
    std::optional<geometry_msgs::msg::Pose> last_goal;

    std::mutex detections_mx;
    std::optional<yolo_msgs::msg::DetectionArray> latest_detections;

    std::mutex img_mx;
    uint32_t img_width{ 0 };
    uint32_t img_height{ 0 };

    inline rclcpp::Logger logger() const
    {
        return node->get_logger();
    }
};
