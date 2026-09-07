#include "prop_maneuvers/maneuvers.hpp"

#include <cmath>
#include <limits>
#include <optional>

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
        context_.driver.release();
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

ApproachObject::ApproachObject(Context &context, double standoff, double clearance)
  : context_(context)
  , deadline_(context.node, context.settings.maneuver_timeout_)
  , standoff_(standoff)
  , clearance_(clearance)
{
}

std::vector<Point> ApproachObject::plan_route(Boat const &boat) const
{
    Point const target = context_.lock.point();

    // Stop clear of the object's SURFACE, so the standoff means the same thing
    // whatever size the object is.
    Point const goal = standoff_point(boat.position, target, context_.lock.radius() + standoff_);

    // Step around whichever blocking blob is nearest the boat.
    std::optional<Point> detour;
    double nearest_block = std::numeric_limits<double>::max();

    for (auto const &blob : context_.lock.blobs())
    {
        // Never treat the target as its own obstacle.
        if (distance(blob.centre, target) <= context_.lock.radius() + 1.0)
        {
            continue;
        }

        // CAVEAT (open design question, see the plan's Task 9 section): the
        // waypoint detour_point() returns really is `keep_out` from the blob,
        // but the boat drives start -> waypoint -> goal, and BOTH legs cut the
        // corner. Measured clearance can fall well short of what was asked
        // for -- as low as 0.93 m when 2.0 m was requested, if the obstacle is
        // nearer than keep_out along the direction of travel. This is a flaw
        // in detour_point's design, not something to work around here; fixing
        // it is David's call (four options are laid out in the plan) and
        // belongs entirely inside detour_point and its tests. ApproachObject
        // uses it as-is.
        auto const step_around = detour_point(boat.position, goal, blob.centre, blob.radius + clearance_);
        if (!step_around)
        {
            continue;
        }

        double const range = distance(boat.position, blob.centre);
        if (range < nearest_block)
        {
            nearest_block = range;
            detour = step_around;
        }
    }

    if (detour)
    {
        return { *detour, goal };
    }
    return { goal };
}

bool ApproachObject::worth_replanning(std::vector<Point> const &fresh) const
{
    if (fresh.size() != route_.size())
    {
        return true;
    }
    for (std::size_t i = 0; i < fresh.size(); ++i)
    {
        if (distance(fresh[i], route_[i]) > 0.5)
        {
            return true;
        }
    }
    return false;
}

Status ApproachObject::step()
{
    if (deadline_.expired())
    {
        context_.spinner.stop();
        context_.driver.release();
        RCLCPP_ERROR(context_.node->get_logger(), "approach: gave up after %.0f s",
                     context_.settings.maneuver_timeout_);
        return Status::Failed;
    }

    Boat const boat = context_.boat();
    if (!boat.valid)
    {
        RCLCPP_WARN_THROTTLE(context_.node->get_logger(), *context_.node->get_clock(), 2000,
                             "approach: waiting for the position estimate");
        return Status::Running;
    }

    if (!context_.lock.locked())
    {
        RCLCPP_ERROR(context_.node->get_logger(), "approach: nothing locked on");
        return Status::Failed;
    }

    switch (phase_)
    {
        case Phase::FaceTarget:
        {
            if (!released_for_turn_)
            {
                context_.driver.release();
                released_for_turn_ = true;
            }

            if (!context_.spinner.step(boat.direction, bearing(boat.position, context_.lock.point())))
            {
                return Status::Running;
            }

            // Pointed at it: best possible look, so re-read before planning.
            context_.lock.refresh();
            last_refresh_ = context_.node->now();

            route_ = plan_route(boat);
            context_.spinner.stop();
            context_.driver.go_to(route_);
            phase_ = Phase::Driving;
            RCLCPP_INFO(context_.node->get_logger(), "approach: %zu point route, stopping %.1f m clear", route_.size(),
                        standoff_);
            return Status::Running;
        }

        case Phase::Driving:
        {
            if (context_.driver.arrived(boat.position))
            {
                context_.driver.release();
                context_.spinner.stop();  // release only silences guidance; this stops the boat
                RCLCPP_INFO(context_.node->get_logger(), "approach: arrived, %.1f m from the object",
                            distance(boat.position, context_.lock.point()));
                return Status::Succeeded;
            }

            // Keep confirming the object is actually still there while
            // driving. Without this, the lock's timestamp freezes the moment
            // the drive starts (refresh() is only otherwise called once, back
            // in FaceTarget), and stale() below would fire on the clock alone
            // -- aborting a perfectly good approach just because it takes
            // longer than reading_max_age to arrive. A single failed refresh
            // is survivable, same as CircleObject: it does not clear the
            // lock, so the remembered point is still used until a refresh
            // actually succeeds or the lock goes genuinely stale.
            if ((context_.node->now() - last_refresh_).seconds() >= kRefreshInterval)
            {
                if (!context_.lock.refresh() && !context_.lock.stale())
                {
                    RCLCPP_WARN(context_.node->get_logger(), "approach: keeping the remembered point (%s)",
                                context_.lock.why().c_str());
                }
                last_refresh_ = context_.node->now();
            }

            // A stale lock means we have not seen the object for
            // reading_max_age and are driving at a memory. Stop rather than
            // close in on a position nothing has confirmed.
            if (context_.lock.stale())
            {
                context_.driver.release();
                context_.spinner.stop();
                RCLCPP_ERROR(context_.node->get_logger(), "approach: lost the object (%s); stopping",
                             context_.lock.why().c_str());
                return Status::Failed;
            }

            // The picture can change while driving: the object's estimate can
            // shift, or a blob can move into the way. Re-hand the route only
            // when it has actually changed, since a new plan restarts
            // guidance's leg from the boat's current position.
            auto const fresh = plan_route(boat);
            if (worth_replanning(fresh))
            {
                route_ = fresh;
                context_.driver.go_to(route_);
                RCLCPP_INFO(context_.node->get_logger(), "approach: route changed, now %zu point(s)", route_.size());
            }

            return Status::Running;
        }
    }

    return Status::Running;
}

}  // namespace prop_maneuvers
