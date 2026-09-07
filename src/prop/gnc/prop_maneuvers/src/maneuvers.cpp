#include "prop_maneuvers/maneuvers.hpp"

#include <cmath>

#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace prop_maneuvers
{

Context::Context(rclcpp::Node *node_in, Constants const &settings_in)
  : node(node_in)
  , settings(settings_in)
  , driver(node_in, settings_in.arrive_tolerance_)
  , spinner(node_in, settings_in.turn_gain_, settings_in.max_turn_rate_, settings_in.point_tolerance_)
  , lock(node_in, settings_in)
{
    odometry_subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered/global", 10,
        [this](nav_msgs::msg::Odometry::SharedPtr const msg)
        {
            boat_.position = Point{ msg->pose.pose.position.x, msg->pose.pose.position.y };
            boat_.direction = tf2::getYaw(msg->pose.pose.orientation);
            boat_.valid = true;
        });
}

Deadline::Deadline(rclcpp::Node *node, double seconds) : node_(node), start_(node->now()), seconds_(seconds)
{
}

bool Deadline::expired() const
{
    return (node_->now() - start_).seconds() > seconds_;
}

FaceObject::FaceObject(Context &context)
  : context_(context), deadline_(context.node, context.settings.maneuver_timeout_)
{
}

Status FaceObject::step()
{
    if (deadline_.expired())
    {
        context_.spinner.stop();
        RCLCPP_ERROR(context_.node->get_logger(), "face: gave up after %.0f s", context_.settings.maneuver_timeout_);
        return Status::Failed;
    }

    Boat const boat = context_.boat();
    if (!boat.valid)
    {
        RCLCPP_WARN_THROTTLE(context_.node->get_logger(), *context_.node->get_clock(), 2000,
                             "face: waiting for the position estimate");
        return Status::Running;
    }

    if (!context_.lock.locked())
    {
        RCLCPP_ERROR(context_.node->get_logger(), "face: nothing locked on");
        return Status::Failed;
    }

    // Release the driver before commanding a single turn. If guidance still
    // holds points it publishes cmd_vel ten times a second and the two streams
    // interleave, making the boat stutter.
    if (!released_)
    {
        context_.driver.release();
        released_ = true;
    }

    double const target = bearing(boat.position, context_.lock.point());
    if (context_.spinner.step(boat.direction, target))
    {
        RCLCPP_INFO(context_.node->get_logger(), "face: pointing at (%.1f, %.1f)", context_.lock.point().x,
                    context_.lock.point().y);
        return Status::Succeeded;
    }

    return Status::Running;
}

}  // namespace prop_maneuvers
