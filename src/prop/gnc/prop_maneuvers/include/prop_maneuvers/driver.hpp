/**
 * @file driver.hpp
 * @brief Delegates straight-line driving to prop_controller's guidance node.
 *
 *   out  plan  nav_msgs/Path, map frame, latched
 *
 * guidance drives to each point in turn and then parks on the last one. It
 * publishes cmd_vel continuously while it holds any points at all, so a
 * maneuver that wants to command the motors itself MUST call release() first.
 * release() publishes an empty Path, which makes guidance return early and go
 * completely quiet.
 */

#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "prop_maneuvers/geometry.hpp"

#include <nav_msgs/msg/path.hpp>

namespace prop_maneuvers
{

class Driver
{
  public:
    Driver(rclcpp::Node *node, double arrive_tolerance);

    /// Hand guidance a list of points to drive, in order.
    void go_to(std::vector<Point> const &points);

    /// Hand guidance a single point.
    void go_to(Point const &point);

    /// Switch guidance off. Safe to call when it is already off.
    void release();

    /// True once `boat` is within tolerance of the last point handed over.
    /// False when nothing has been handed over.
    ///
    /// Only the last point is checked. Callers must ensure it is the real
    /// destination; any earlier points in a multi-point go_to() are routing
    /// aids only, and arrived() does not check whether they were visited.
    bool arrived(Point const &boat) const;

  private:
    rclcpp::Node *node_;
    double arrive_tolerance_;
    std::vector<Point> points_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
};

}  // namespace prop_maneuvers
