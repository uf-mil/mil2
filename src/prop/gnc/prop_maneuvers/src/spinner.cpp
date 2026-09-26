#include "prop_maneuvers/spinner.hpp"

#include <algorithm>
#include <cmath>

#include "prop_maneuvers/geometry.hpp"

namespace prop_maneuvers
{

Spinner::Spinner(rclcpp::Node *node, double turn_gain, double max_turn_rate, double point_tolerance)
  : node_(node), turn_gain_(turn_gain), max_turn_rate_(max_turn_rate), point_tolerance_(point_tolerance)
{
    command_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

bool Spinner::step(double current_direction, double target_direction)
{
    last_error_ = wrap_angle(target_direction - current_direction);

    if (std::abs(last_error_) <= point_tolerance_)
    {
        stop();
        return true;
    }

    geometry_msgs::msg::Twist command;
    // Forward speed stays at zero on every path through this class: the whole
    // point is to turn without moving.
    command.linear.x = 0.0;
    command.angular.z = std::clamp(turn_gain_ * last_error_, -max_turn_rate_, max_turn_rate_);
    command_publisher_->publish(command);

    RCLCPP_DEBUG(node_->get_logger(), "turning, %.1f degrees to go", last_error_ * 180.0 / M_PI);
    return false;
}

void Spinner::stop()
{
    command_publisher_->publish(geometry_msgs::msg::Twist());
}

}  // namespace prop_maneuvers
