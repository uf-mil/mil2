/**
 * @file spinner.hpp
 * @brief Turns the boat in place by commanding the motors directly.
 *
 *   out  cmd_vel  geometry_msgs/Twist
 *
 * guidance parks on a point but stops caring which way the boat points, so
 * "hold position and point that way" has to come from here. thruster_manager
 * closes its own loop on measured turn rate, so this only has to ask for a
 * turn rate, not solve for thrust.
 *
 * The caller MUST have released the driver first. If guidance still holds
 * points it keeps publishing cmd_vel ten times a second and the two streams
 * interleave, making the boat stutter.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

namespace prop_maneuvers
{

class Spinner
{
  public:
    Spinner(rclcpp::Node *node, double turn_gain, double max_turn_rate, double point_tolerance);

    /// Command one step of a turn from `current_direction` towards
    /// `target_direction`, both absolute map directions in radians.
    /// Returns true once the boat is pointing close enough.
    bool step(double current_direction, double target_direction);

    /// Publish a single zero command. Call once when the turn ends, then stop
    /// calling step(); thruster_manager drops thrust after a second of silence.
    void stop();

    /// The error from the most recent step(), for logging.
    double last_error() const
    {
        return last_error_;
    }

  private:
    rclcpp::Node *node_;
    double turn_gain_;
    double max_turn_rate_;
    double point_tolerance_;
    double last_error_{ 0.0 };
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
};

}  // namespace prop_maneuvers
