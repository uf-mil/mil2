/**
 * @file maneuvers.hpp
 * @brief The three reusable maneuvers, as small state machines.
 *
 * Each maneuver is stepped on a timer and reports Running, Succeeded or
 * Failed. They share a Context holding the boat's position, the driver, the
 * spinner and the target lock.
 *
 * The rule every maneuver follows: only one thing commands the motors at a
 * time. Before the spinner is used, the driver is released; before the driver
 * is used, the spinner is stopped.
 */

#pragma once

#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "prop_maneuvers/constants.hpp"
#include "prop_maneuvers/driver.hpp"
#include "prop_maneuvers/geometry.hpp"
#include "prop_maneuvers/spinner.hpp"
#include "prop_maneuvers/target_lock.hpp"

#include <nav_msgs/msg/odometry.hpp>

namespace prop_maneuvers
{

enum class Status
{
    Running,
    Succeeded,
    Failed,
};

/// Where the boat is and which way it points, from the position estimate.
struct Boat
{
    Point position;
    double direction{ 0.0 };
    bool valid{ false };
};

/// Everything a maneuver needs, built once per program.
class Context
{
  public:
    Context(rclcpp::Node *node, Constants const &settings);

    Boat boat() const
    {
        return boat_;
    }

    rclcpp::Node *node;
    Constants const &settings;
    Driver driver;
    Spinner spinner;
    TargetLock lock;

  private:
    Boat boat_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
};

/// Times a maneuver out using the node clock, which follows simulated time
/// when use_sim_time is set. Never use wall-clock here: simulation speed on
/// this machine has been measured varying more than tenfold between runs.
class Deadline
{
  public:
    Deadline(rclcpp::Node *node, double seconds);
    bool expired() const;

  private:
    rclcpp::Node *node_;
    rclcpp::Time start_;
    double seconds_;
};

/// Hold position and turn until the front of the boat points at the lock.
class FaceObject
{
  public:
    explicit FaceObject(Context &context);
    Status step();

  private:
    Context &context_;
    Deadline deadline_;
    bool released_{ false };
};

}  // namespace prop_maneuvers
