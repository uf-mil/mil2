#include <rclcpp/rclcpp.hpp>

#include "mil_tools/geometry/Rotation.hpp"
#include "mil_tools/geometry/Slerp.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

class PathPlannerNode : public rclcpp::Node
{
    int segment_count_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Odometry last_odom_;

  public:
    PathPlannerNode() : Node("path_planner_node"), segment_count_(10)
    {
        // Goal pose subscription
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "goal_pose", 10, std::bind(&PathPlannerNode::goal_pose_cb, this, std::placeholders::_1));
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&PathPlannerNode::odom_cb, this, std::placeholders::_1));
        // Path publisher
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
    }

    void goal_pose_cb(geometry_msgs::msg::Pose::SharedPtr const msg)
    {
        // Generate path
        auto path = generate_path(*msg);
        // Publish path
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();
        for (auto const &pose : path)
        {
            path_msg.poses.push_back(pose);
        }
        path_pub_->publish(path_msg);
    }

    void odom_cb(nav_msgs::msg::Odometry::SharedPtr const msg)
    {
        last_odom_ = *msg;
    }

    std::vector<geometry_msgs::msg::Pose> slerp(geometry_msgs::msg::Pose const &goal_pose)
    {
        std::vector<geometry_msgs::msg::Pose> path;
        Rotation start_rot{ last_odom_.pose.pose.orientation };
        Rotation goal_rot{ goal_pose.orientation };
        Slerp slerp(start_rot, goal_rot);
        for (int i = 0; i < segment_count_; ++i)
        {
            double t = static_cast<double>(i) / (segment_count_ - 1);
            geometry_msgs::msg::Pose pose;
            pose.position.x =
                last_odom_.pose.pose.position.x + t * (goal_pose.position.x - last_odom_.pose.pose.position.x);
            pose.position.y =
                last_odom_.pose.pose.position.y + t * (goal_pose.position.y - last_odom_.pose.pose.position.y);
            pose.position.z =
                last_odom_.pose.pose.position.z + t * (goal_pose.position.z - last_odom_.pose.pose.position.z);
            pose.orientation = slerp.get_rotation(t).quat_msg();
            path.push_back(pose);
        }
    }

    std::vector<geometry_msgs::msg::Pose> generate_path(geometry_msgs::msg::Pose const &goal_pose)
    {
        return slerp(goal_pose);
    }
};
