#include "subjugator_trajectory_planner/trajectory_planner.h"

TrajectoryPlanner::TrajectoryPlanner() : Node("trajectory_planner")
{
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10, [this](nav_msgs::msg::Odometry::UniquePtr msg) { this->odom_cb(std::move(msg)); });
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("goal/trajectory", 1);
}

void TrajectoryPlanner::odom_cb(nav_msgs::msg::Odometry::UniquePtr const msg)
{
    this->odom_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w;
}
