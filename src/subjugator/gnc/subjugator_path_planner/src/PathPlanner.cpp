#include "subjugator_path_planner/PathPlanner.hpp"

#include "mil_tools/geometry/Rotation.hpp"
#include "mil_tools/geometry/Slerp.hpp"

PathPlanner::PathPlanner() : Node("path_planner_node"), segment_count_(10)
{
    // Goal pose subscription
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "goal_pose", 10, [this](geometry_msgs::msg::Pose::SharedPtr msg) { goal_pose_cb(msg); });
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom_cb(msg); });
    // Path publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
}

void PathPlanner::goal_pose_cb(geometry_msgs::msg::Pose::SharedPtr const &msg)
{
    // Generate path
    auto path = generate_path(*msg);
    // Publish path
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();
    for (auto const &pose : path)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = this->now();
        pose_stamped.pose = pose;
        path_msg.poses.push_back(pose_stamped);
    }
    path_pub_->publish(path_msg);
}

void PathPlanner::odom_cb(nav_msgs::msg::Odometry::SharedPtr const &msg)
{
    last_odom_ = *msg;
}

std::vector<geometry_msgs::msg::Pose> PathPlanner::slerp(geometry_msgs::msg::Pose const &goal_pose)
{
    std::vector<geometry_msgs::msg::Pose> path;
    mil_tools::geometry::Rotation const start_rot{ last_odom_.pose.pose.orientation };
    mil_tools::geometry::Rotation const goal_rot{ goal_pose.orientation };
    mil_tools::geometry::Slerp const slerp(start_rot, goal_rot);
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
        pose.orientation = slerp.at(t).quat_msg();
        path.push_back(pose);
    }
    return path;
}

std::vector<geometry_msgs::msg::Pose> PathPlanner::generate_path(geometry_msgs::msg::Pose const &goal_pose)
{
    return slerp(goal_pose);
}
