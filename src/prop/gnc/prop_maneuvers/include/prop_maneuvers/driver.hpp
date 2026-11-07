/**
 * @file driver.hpp
 * @brief Delegates straight-line driving to prop_controller's guidance node.
 *
 *   out  plan  nav_msgs/Path, map frame, latched
 *
 * guidance drives to each point in turn, parks on the last one and, if that
 * pose carries a heading, turns to it. It publishes cmd_vel continuously while
 * it holds any points at all. Nothing in this package commands the motors
 * directly. release() publishes an empty Path, which makes guidance go quiet
 * and lets thruster_manager cut thrust; prefer a plan at the boat to hold it.
 */

#pragma once

#include <optional>
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

    /// Hand guidance a list of points to drive, in order. `final_heading` (radians, map frame) is where to point
    /// once parked on the last one; without it guidance finishes on whatever heading it arrives with.
    void go_to(std::vector<Point> const &points, std::optional<double> final_heading = std::nullopt);

    /// Hand guidance a single point.
    void go_to(Point const &point, std::optional<double> final_heading = std::nullopt);

    /// Keep the boat where it is, so guidance keeps publishing zero speed and thruster_manager does not cut
    /// thrust. With a heading it also turns on the spot to it and holds it; without one the heading is left
    /// alone. Falls back to release() when there is no position estimate yet.
    void hold(Boat const &boat, std::optional<double> heading = std::nullopt);

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
