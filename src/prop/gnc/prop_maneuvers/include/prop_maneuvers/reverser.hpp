/**
 * @file reverser.hpp
 * @brief Backs the boat straight up by commanding the motors directly.
 *
 *   out  cmd_vel  geometry_msgs/Twist
 *
 * guidance will never do this: its speed is `speed * max(0, cos(error))`, so
 * it cannot command a negative forward speed at all. Handed a point behind the
 * boat it pivots 180 degrees and drives forward instead, which sweeps the hull
 * through the very water being avoided and costs about four times as long.
 *
 * Holding the heading is the point. The boat ends up pointing where it
 * started, so whatever comes next is a continuation rather than a recovery
 * from a 180 degree detour, and the target stays in front of the lidar
 * throughout.
 *
 * The caller MUST have released the driver first, exactly as with Spinner. If
 * guidance still holds points it keeps publishing cmd_vel ten times a second
 * and the two streams interleave, making the boat stutter.
 *
 * The caller MUST also check clear_behind() first, and again on every step.
 * This class has no sensors and will happily reverse into a buoy.
 *
 * LIMITATION: holding a heading is not holding a line. Wind or current will
 * crab the boat sideways a little, and nothing here corrects for it. Over the
 * couple of metres this is meant for that is small; correcting it is
 * guidance's job, not this one's.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "prop_maneuvers/geometry.hpp"

#include <geometry_msgs/msg/twist.hpp>

namespace prop_maneuvers
{

class Reverser
{
  public:
    Reverser(rclcpp::Node *node, double speed, double turn_gain, double max_turn_rate);

    /// Command one step of backing away. `start` and `held_direction` are
    /// whatever the caller stashed when the reverse began; this class keeps no
    /// state of its own, the same way Spinner does not.
    ///
    /// Returns true once the boat has travelled `distance` from `start`.
    bool step(Point const &current, double current_direction, Point const &start, double held_direction,
              double distance_to_cover);

    /// Publish a single zero command. Call once when the reverse ends, then
    /// stop calling step(); thruster_manager drops thrust after a second of
    /// silence.
    void stop();

  private:
    rclcpp::Node *node_;
    double speed_;
    double turn_gain_;
    double max_turn_rate_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
};

}  // namespace prop_maneuvers
