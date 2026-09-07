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
  , reverser(node_in, settings_in.reverse_speed_, settings_in.turn_gain_, settings_in.max_turn_rate_)
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

ApproachObject::ApproachObject(Context &context, double standoff)
  : context_(context), deadline_(context.node, context.settings.maneuver_timeout_), standoff_(standoff)
{
}

namespace
{
/// The worst thing blocking the leg from `boat` to `goal`, ignoring `target`
/// itself. Used to decide whether to back off before planning a route.
///
/// Deliberately separate from plan_route: the decision to reverse is taken
/// before there is any route to drive, and plan_route's answer to
/// TooCloseToSwing is to give up and hand back best effort. Asking it here
/// would lose the distinction between "nothing in the way" and "in the way and
/// unroutable".
DetourNeed worst_need(Context const &context, Point const &boat, Point const &goal, Point const &target)
{
    DetourNeed worst = DetourNeed::None;
    for (auto const &blob : context.lock.blobs())
    {
        if (distance(blob.centre, target) <= context.lock.radius() + context.settings.target_blob_margin_)
        {
            continue;
        }
        Detour const d = plan_detour(boat, goal, blob, context.settings.hull_half_width_, context.settings.min_gap_,
                                     context.settings.detour_clearance_);
        if (d.need == DetourNeed::TooCloseToSwing)
        {
            return DetourNeed::TooCloseToSwing;  // the worst there is
        }
        if (d.need == DetourNeed::Straddle)
        {
            worst = DetourNeed::Straddle;
        }
    }
    return worst;
}
}  // namespace

std::vector<Point> ApproachObject::plan_route(Boat const &boat) const
{
    Point const target = context_.lock.point();

    // Stop clear of the object's SURFACE, so the standoff means the same thing
    // whatever size the object is.
    Point const goal = standoff_point(boat.position, target, context_.lock.radius() + standoff_);

    // Step around whichever blocking blob is nearest the boat.
    Detour chosen;
    double nearest_block = std::numeric_limits<double>::max();

    for (auto const &blob : context_.lock.blobs())
    {
        // Never treat the target as its own obstacle. The margin is a setting
        // rather than a hardcoded metre because it depends on how far the
        // clustering's idea of the buoy's edge can sit from the lock's.
        if (distance(blob.centre, target) <= context_.lock.radius() + context_.settings.target_blob_margin_)
        {
            continue;
        }

        Detour const step_around = plan_detour(boat.position, goal, blob, context_.settings.hull_half_width_,
                                               context_.settings.min_gap_, context_.settings.detour_clearance_);
        if (step_around.need == DetourNeed::None)
        {
            continue;
        }

        double const range = distance(boat.position, blob.centre);
        if (range < nearest_block)
        {
            nearest_block = range;
            chosen = step_around;
        }
    }

    // Anything that is not a straddle -- including TooCloseToSwing -- gets the
    // straight line. Backing off is step()'s business and happens at most once
    // per approach; by the time a second TooCloseToSwing turns up there is
    // nothing left to try, so best effort beats refusing to move.
    if (chosen.need != DetourNeed::Straddle)
    {
        return { goal };
    }

    if (chosen.achieved < context_.settings.detour_clearance_ - 1e-6)
    {
        // Say so rather than quietly under-delivering. This happens when the
        // goal itself lies inside the clearance, which no routing can fix.
        RCLCPP_WARN(context_.node->get_logger(), "approach: stepping around at %.2f m, %.2f m was asked for",
                    chosen.achieved, context_.settings.detour_clearance_);
    }

    return { chosen.before, chosen.after, goal };
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
        context_.reverser.stop();
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
            //
            // The stamp goes with the refresh, and both happen before EVERY
            // exit from this phase, backing off included. Phase::Driving
            // subtracts last_refresh_ from the node clock, and a
            // default-constructed rclcpp::Time carries a different clock
            // source, so leaving it unstamped throws rather than simply
            // reading as "long ago".
            context_.lock.refresh();
            last_refresh_ = context_.node->now();

            Point const goal = standoff_point(boat.position, context_.lock.point(), context_.lock.radius() + standoff_);

            // Asked once, here, on the reading taken a moment ago. This is the
            // only point in the approach where the boat is stopped and pointed
            // at the target, so it is the only point where a reverse is cheap;
            // has_reversed_ then keeps it to one for the rest of the maneuver.
            if (!has_reversed_ &&
                worst_need(context_, boat.position, goal, context_.lock.point()) == DetourNeed::TooCloseToSwing)
            {
                // Only one thing commands the motors at a time. The spinner
                // has just had them and guidance must stay quiet, so both are
                // shut down before the reverser is given a single tick.
                context_.driver.release();
                context_.spinner.stop();
                reverse_start_ = boat.position;
                reverse_held_direction_ = boat.direction;
                reverse_distance_ = context_.settings.reverse_max_distance_;
                has_reversed_ = true;
                phase_ = Phase::BackingOff;
                RCLCPP_INFO(context_.node->get_logger(), "approach: too close to step around it, backing off %.1f m",
                            reverse_distance_);
                return Status::Running;
            }

            route_ = plan_route(boat);
            context_.spinner.stop();
            context_.driver.go_to(route_);
            phase_ = Phase::Driving;
            RCLCPP_INFO(context_.node->get_logger(), "approach: %zu point route, stopping %.1f m clear", route_.size(),
                        standoff_);
            return Status::Running;
        }

        case Phase::BackingOff:
        {
            // Re-checked on EVERY step, not once on entry. The boat can see
            // straight out the back -- the blind spots are on the sides -- so
            // this is live data rather than a snapshot, and a reverse takes
            // several seconds, which is long enough for something to drift in
            // behind us. Stopping short and taking a worse route is always
            // better than backing into a buoy: Reverser has no sensors of its
            // own and will do exactly that if nobody is watching for it.
            if (!clear_behind(context_.lock.blobs(), boat.position, boat.direction, reverse_distance_,
                              context_.settings.hull_half_width_, context_.settings.hull_behind_))
            {
                context_.reverser.stop();
                RCLCPP_WARN(context_.node->get_logger(), "approach: something behind us, taking the best route "
                                                         "available instead");
                route_ = plan_route(boat);
                context_.driver.go_to(route_);
                phase_ = Phase::Driving;
                return Status::Running;
            }

            // Keep confirming the object while backing off, on the same
            // cadence Phase::Driving uses. The reverser holds the heading, so
            // the target stays in front of the lidar the whole way and this is
            // a real look, not a formality. Without it the lock's timestamp
            // would freeze for the length of the reverse -- 2.0 m at 0.4 m/s
            // is five seconds, which is exactly reading_max_age -- and the
            // approach would reach Phase::Driving one tick from stale on the
            // strength of the clock alone. A failed refresh is survivable and
            // stays quiet here; Phase::Driving reports a lock that has
            // genuinely gone.
            if ((context_.node->now() - last_refresh_).seconds() >= kRefreshInterval)
            {
                context_.lock.refresh();
                last_refresh_ = context_.node->now();
            }

            if (context_.reverser.step(boat.position, boat.direction, reverse_start_, reverse_held_direction_,
                                       reverse_distance_))
            {
                // step() publishes its own zero command on the tick it
                // finishes. Saying so again costs one duplicate zero twist and
                // means leaving the motors uncommanded does not depend on a
                // primitive's internals, which is the same reason the timeout
                // branch above stops it too.
                context_.reverser.stop();
                RCLCPP_INFO(context_.node->get_logger(), "approach: backed off, re-planning");
                route_ = plan_route(boat);
                context_.driver.go_to(route_);
                phase_ = Phase::Driving;
            }
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
