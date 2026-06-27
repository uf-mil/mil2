#pragma once
#include <sys/types.h>

#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "subjugator_msgs/msg/thruster_efforts.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <subjugator_msgs/srv/servo.hpp>
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
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr wall_direction_sub;

    // Service clients to actuate servos (driver.py services)
    rclcpp::Client<subjugator_msgs::srv::Servo>::SharedPtr dropper_client;
    rclcpp::Client<subjugator_msgs::srv::Servo>::SharedPtr gripper_client;
    rclcpp::Client<subjugator_msgs::srv::Servo>::SharedPtr torpedo_client;
    // services and clients
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr controller_enable_client;

    // Latest state
    std::mutex odom_mx;
    std::optional<nav_msgs::msg::Odometry> latest_odom;

    std::mutex last_goal_mx;
    std::optional<geometry_msgs::msg::Pose> last_goal;

    std::mutex detections_mx;
    std::optional<yolo_msgs::msg::DetectionArray> latest_detections;

    // Latest class from the coin_flip classifier node (/coin_flip/direction).
    std::mutex wall_direction_mx;
    std::optional<std::string> latest_wall_direction;

    // PID of a helper node the mission launched (e.g. coin_flip_node); its own
    // process group so it can be killed independently of the terminal. -1 = none.
    std::mutex child_mx;
    pid_t coin_flip_pid{ -1 };

    std::mutex img_mx;
    uint32_t img_width{ 0 };
    uint32_t img_height{ 0 };

    // Named absolute waypoints captured during a mission (or preloaded).
    // Mission BT nodes (RememberWaypoint / LookupWaypoint) read and write this.
    std::mutex waypoints_mx;
    std::unordered_map<std::string, geometry_msgs::msg::Pose> waypoints;

    inline rclcpp::Logger logger() const
    {
        return node->get_logger();
    }
};
