#pragma once
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

struct Context
{
    rclcpp::Node::SharedPtr node;

    // Publishers and Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goal_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr targets_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

    // Service Clients
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr gripper_client;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr marble_client;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr torpedo_client;

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

// Function: Actuate a Service Client to open or close a servo
// Output: Returns a boolean indicating whether the service call was successful (true) or not (false)
inline bool actuateServo(std::shared_ptr<rclcpp::Node> node, rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client,
                         bool isOpen)
{
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = isOpen;
    auto future = client->async_send_request(request);
    std::string clientName = client->get_service_name();

    // Spin Node until future completes, or timeout after 2 second
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(2)) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Servo actuation for %s: %s", clientName.c_str(),
                    future.get()->success ? "true" : "false");
        return true;
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call %s service", clientName.c_str());
        return false;
    }
}
