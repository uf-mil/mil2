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

    // Each returns the speed and heading error to command.
    std::pair<double, double> follow() const;
    std::pair<double, double> hold() const;

    double speed_;          // m/s along a leg
    double lookahead_;      // m, larger converges more gently
    double accept_radius_;  // m
    double kp_heading_;
    double max_yaw_rate_;   // rad/s
    double hold_radius_;    // m, station keeping deadband
    double approach_gain_;  // m/s per m remaining

    std::vector<Point> waypoints_;
    Point leg_start_{ 0.0, 0.0 };
    Point position_{ 0.0, 0.0 };
    double heading_{ 0.0 };
    bool located_{ false };
    std::size_t target_{ 0 };  // Keeps track of whether or not we arrived at the end of goal

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
