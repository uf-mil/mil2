#include "subjugator_path_planner/PathPlanner.hpp"

#include <cmath>

#include "mil_tools/geometry/Rotation.hpp"
#include "mil_tools/geometry/Slerp.hpp"

PathPlanner::PathPlanner() : Node("path_planner_node"), segment_count_(10)
{
    // Goal pose subscription
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "goal_pose", 10, [this](geometry_msgs::msg::Pose::SharedPtr msg) { goal_pose_cb(msg); });
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom_cb(msg); });
    // Path publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
}

void PathPlanner::goal_pose_cb(geometry_msgs::msg::Pose::SharedPtr const &msg)
{
    // Generate path
    auto path = generate_path(*msg);
    // Publish path
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "odom";
    path_msg.header.stamp = this->now();
    for (auto const &pose : path)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "odom";
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

    // get vector from current to goal with magnitude 0.33, add this to A one too few times, then final path is B
    // TODO rotations??
    double const segment_legnth = 0.33;
    double const x_err = goal_pose.position.x - last_odom_.pose.pose.position.x;
    double const y_err = goal_pose.position.y - last_odom_.pose.pose.position.y;
    double const z_err = goal_pose.position.z - last_odom_.pose.pose.position.z;
    double const err_magnitude = std::sqrt(x_err * x_err + y_err * y_err + z_err * z_err);
    double const delta_x = x_err * (segment_legnth / err_magnitude);
    double const delta_y = y_err * (segment_legnth / err_magnitude);
    double const delta_z = z_err * (segment_legnth / err_magnitude);

    // casting to an int is basically the same as std::floor, plus we need it to be an int :P
    segment_count_ = (int)(err_magnitude / segment_legnth);
    double const t_inc_amount = std::min(0.1, 1.0 / ((double)segment_count_));
    for (int i = 1; i < segment_count_ + 1; ++i)
    {
        // double t = static_cast<double>(i) / (segment_count_);
        geometry_msgs::msg::Pose pose;
        pose.position.x = last_odom_.pose.pose.position.x + i * delta_x;
        pose.position.y = last_odom_.pose.pose.position.y + i * delta_y;
        pose.position.z = last_odom_.pose.pose.position.z + i * delta_z;
        pose.orientation = slerp.at(i * t_inc_amount).quat_msg();
        path.push_back(pose);
    }

    double t_so_far = t_inc_amount * segment_count_;
    while (t_so_far + t_inc_amount < 1)
    {
        t_so_far += t_inc_amount;  // the fact that we are in this while loop implies that this sum is <1

        geometry_msgs::msg::Pose pose;
        pose.position = goal_pose.position;
        pose.orientation = slerp.at(t_so_far).quat_msg();
        path.push_back(pose);
    }

    // one more path after the loop since we took the floor of segment_count
    // this path is the only one which doesn't have a magnitude of segment_legnth.
    // This path can be thought of as a fraction of a segment (usually we'd need like 2.3 segments, this is the 0.3)
    path.push_back(goal_pose);

    return path;
}

std::vector<geometry_msgs::msg::Pose> PathPlanner::generate_path(geometry_msgs::msg::Pose const &goal_pose)
{
    return slerp(goal_pose);
}
