#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

// Follows a path with lookahead line-of-sight guidance, then holds station on
// the final waypoint.
//
//   in   plan                      nav_msgs/Path, map frame
//        odometry/filtered/global  nav_msgs/Odometry
//   out  cmd_vel                   geometry_msgs/Twist
//
// The boat cannot translate sideways, so cross-track error is corrected by
// steering back onto the leg rather than crabbing across to it. Station keeping
// is a deadband for the same reason: it can only hold the point by driving at it.
class Guidance : public rclcpp::Node
{
  public:
    Guidance();

  private:
    using Point = std::pair<double, double>;

    void plan_callback(nav_msgs::msg::Path const& path);
    void step();

    // Each returns the speed and heading error to command. hold latches, so
    // unlike follow it is not const.
    std::pair<double, double> follow() const;
    std::pair<double, double> hold();

    double speed_;
    double lookahead_;      // m, larger converges more gently
    double accept_radius_;  // m
    double kp_heading_;
    double max_yaw_rate_;
    double hold_radius_;    // station keeping deadband
    double yaw_tolerance_;  // heading deadband once it is on the point
    double approach_gain_;  // m/s per m remaining

    std::vector<Point> waypoints_;
    Point leg_start_{ 0.0, 0.0 };
    Point position_{ 0.0, 0.0 };
    double heading_{ 0.0 };
    bool located_{ false };
    std::size_t target_{ 0 };  // Keeps track of whether or not we arrived at the end of goal

    double goal_heading_{ 0.0 };  // only meaningful with has_goal_heading_
    bool has_goal_heading_{ false };
    bool holding_heading_{ false };

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
