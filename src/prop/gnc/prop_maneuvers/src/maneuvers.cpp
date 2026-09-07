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

CircleObject::CircleObject(Context &context, double radius, int legs, bool counter_clockwise)
  : context_(context)
  , deadline_(context.node, context.settings.maneuver_timeout_)
  , radius_(radius)
  , legs_(legs)
  , counter_clockwise_(counter_clockwise)
{
}

bool CircleObject::turn_towards_buoy()
{
    // Release the driver before commanding a turn, once per turn. guidance
    // publishes cmd_vel continuously while it holds any points, and the two
    // streams interleaving makes the boat stutter.
    if (!released_for_turn_)
    {
        context_.driver.release();
        released_for_turn_ = true;
    }

    Boat const boat = context_.boat();
    return context_.spinner.step(boat.direction, bearing(boat.position, context_.lock.point()));
}

Status CircleObject::step()
{
    if (deadline_.expired())
    {
        context_.spinner.stop();
        context_.driver.release();
        RCLCPP_ERROR(context_.node->get_logger(), "circle: gave up after %.0f s on leg %d of %d",
                     context_.settings.maneuver_timeout_, legs_driven_, legs_);
        return Status::Failed;
    }

    Boat const boat = context_.boat();
    if (!boat.valid)
    {
        RCLCPP_WARN_THROTTLE(context_.node->get_logger(), *context_.node->get_clock(), 2000,
                             "circle: waiting for the position estimate");
        return Status::Running;
    }

    if (!context_.lock.locked())
    {
        RCLCPP_ERROR(context_.node->get_logger(), "circle: nothing locked on");
        return Status::Failed;
    }

    switch (phase_)
    {
        case Phase::FaceForEntry:
        {
            if (!turn_towards_buoy())
            {
                return Status::Running;
            }

            // Pointed at the buoy: best possible look, so re-read before
            // committing to a shape.
            context_.lock.refresh();

            auto const corners = ring_corners(context_.lock.point(), boat.position, radius_, legs_, counter_clockwise_);
            if (corners.empty())
            {
                RCLCPP_ERROR(context_.node->get_logger(), "circle: %d legs is not a shape; need at least 3", legs_);
                return Status::Failed;
            }

            context_.spinner.stop();
            context_.driver.go_to(corners.front());
            phase_ = Phase::DriveToEntry;
            RCLCPP_INFO(context_.node->get_logger(), "circle: running out to the ring at %.1f m", radius_);
            return Status::Running;
        }

        case Phase::DriveToEntry:
        {
            if (!context_.driver.arrived(boat.position))
            {
                return Status::Running;
            }
            context_.driver.release();
            context_.spinner.stop();  // release only silences guidance; this stops the boat
            released_for_turn_ = true;
            legs_driven_ = 0;
            phase_ = Phase::FaceBuoy;
            RCLCPP_INFO(context_.node->get_logger(), "circle: on the ring, starting %d legs", legs_);
            return Status::Running;
        }

        case Phase::FaceBuoy:
        {
            if (!turn_towards_buoy())
            {
                return Status::Running;
            }

            if (!context_.lock.refresh())
            {
                // One failed refresh is survivable: the remembered point is
                // still good enough for the next leg, and the following corner
                // tries again. A lock that has gone STALE is not survivable --
                // it means the buoy has not actually been seen for
                // reading_max_age, so every remaining corner would steer at an
                // increasingly wrong memory and quietly circle nothing.
                if (context_.lock.stale())
                {
                    context_.driver.release();
                    context_.spinner.stop();
                    RCLCPP_ERROR(context_.node->get_logger(),
                                 "circle: lost the buoy on leg %d of %d (%s); stopping rather than "
                                 "circling a memory",
                                 legs_driven_ + 1, legs_, context_.lock.why().c_str());
                    return Status::Failed;
                }
                RCLCPP_WARN(context_.node->get_logger(), "circle: keeping the remembered point (%s)",
                            context_.lock.why().c_str());
            }

            // Redraw around wherever the buoy now is. ring_corners puts the
            // first corner on the line out through the boat, which is where
            // the boat already is, so the next corner is always index 1.
            auto const corners = ring_corners(context_.lock.point(), boat.position, radius_, legs_, counter_clockwise_);
            if (corners.size() < 2)
            {
                RCLCPP_ERROR(context_.node->get_logger(), "circle: %d legs is not a shape; need at least 3", legs_);
                return Status::Failed;
            }

            context_.spinner.stop();
            context_.driver.go_to(corners[1]);
            phase_ = Phase::DriveToCorner;
            RCLCPP_INFO(context_.node->get_logger(), "circle: leg %d of %d, to (%.1f, %.1f)", legs_driven_ + 1, legs_,
                        corners[1].x, corners[1].y);
            return Status::Running;
        }

        case Phase::DriveToCorner:
        {
            if (!context_.driver.arrived(boat.position))
            {
                return Status::Running;
            }

            context_.driver.release();
            context_.spinner.stop();  // release only silences guidance; this stops the boat
            released_for_turn_ = true;
            ++legs_driven_;

            if (legs_driven_ >= legs_)
            {
                RCLCPP_INFO(context_.node->get_logger(), "circle: finished %d of %d legs", legs_driven_, legs_);
                return Status::Succeeded;
            }

            phase_ = Phase::FaceBuoy;
            return Status::Running;
        }
    }

    return Status::Running;
}

}  // namespace prop_maneuvers
