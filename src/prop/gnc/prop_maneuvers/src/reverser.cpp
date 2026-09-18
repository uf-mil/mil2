#include "prop_maneuvers/reverser.hpp"

#include <algorithm>
#include <cmath>

namespace prop_maneuvers
{

Reverser::Reverser(rclcpp::Node *node, double speed, double turn_gain, double max_turn_rate)
  : node_(node), speed_(speed), turn_gain_(turn_gain), max_turn_rate_(max_turn_rate)
{
    command_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

bool Reverser::step(Point const &current, double current_direction, Point const &start, double held_direction,
                    double distance_to_cover)
{
    double const travelled = distance(start, current);
    if (travelled >= distance_to_cover)
    {
        stop();
        return true;
    }

    geometry_msgs::msg::Twist command;
    // Backwards, and only ever backwards, on every path through this class.
    command.linear.x = -speed_;
    // Hold the heading we set off with. Same control law as Spinner: a
    // clamped proportional turn, with thruster_manager closing its own loop
    // on the measured rate.
    command.angular.z =
        std::clamp(turn_gain_ * wrap_angle(held_direction - current_direction), -max_turn_rate_, max_turn_rate_);
    command_publisher_->publish(command);

    RCLCPP_DEBUG(node_->get_logger(), "backing off, %.2f of %.2f m", travelled, distance_to_cover);
    return false;
}

void Reverser::stop()
{
    command_publisher_->publish(geometry_msgs::msg::Twist());
}

}  // namespace prop_maneuvers
