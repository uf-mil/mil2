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
    // A node built with use_sim_time before its first /clock message reads
    // now() as 0. Starting the clock here would then measure "seconds since
    // the simulation began" instead of "seconds from now", so the deadline is
    // born already expired against any sim that has been up longer than it.
    // Observed on 2026-09-07: a maneuver launched against a sim at t=604 s
    // gave up "after 300 s" in the same millisecond it started, on the first
    // momentary gap in perception.
    started_ = start_.nanoseconds() > 0;
}

bool Deadline::expired() const
{
    rclcpp::Time const now = node_->now();

    // Start the count from the first real reading rather than from a zero
    // that only meant "the clock had not arrived yet".
    if (!started_)
    {
        if (now.nanoseconds() == 0)
        {
            return false;  // still no clock; nothing has begun, so nothing can be late
        }
        start_ = now;
        started_ = true;
        return false;
    }

    return (now - start_).seconds() > seconds_;
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

ApproachObject::Plan ApproachObject::plan_route(Boat const &boat) const
{
    Point const target = context_.lock.point();

    // Stop clear of the object's SURFACE, so the standoff means the same thing
    // whatever size the object is -- and measure it from the BOW, because that
    // is the part of the boat that gets close to things. base_link is the
    // lidar, 0.760 m aft of the bow, so leaving that out would put the front
    // of the boat three quarters of a metre nearer than asked.
    // Aim past the intended stopping point by guidance's hold radius: it stops
    // commanding once it is that close to the final waypoint, so a goal placed
    // exactly on the mark leaves the boat parked short with nothing left to
    // close the gap. Clamped so the aim point never crosses the object itself.
    double const want_from_centre = context_.lock.radius() + standoff_ + context_.settings.hull_front_;
    double const aim_from_centre =
        std::max(context_.lock.radius(), want_from_centre - context_.settings.guidance_hold_radius_);
    Point const goal = standoff_point(boat.position, target, aim_from_centre);

    Plan plan;
    plan.goal = goal;
    plan.route = { goal };

    // Step around whichever blocking blob is nearest the boat.
    Detour chosen;
    Blob chosen_blob;
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
            chosen_blob = blob;
        }
    }

    // Anything that is not a straddle -- including TooCloseToSwing -- gets the
    // straight line. Backing off is step()'s business and happens at most once
    // per approach; by the time a second TooCloseToSwing turns up there is
    // nothing left to try, so best effort beats refusing to move.
    if (chosen.need != DetourNeed::Straddle)
    {
        return plan;
    }

    if (chosen.achieved < context_.settings.detour_clearance_ - 1e-6)
    {
        // Say so rather than quietly under-delivering. This happens when the
        // goal itself lies inside the clearance, which no routing can fix.
        RCLCPP_WARN(context_.node->get_logger(), "approach: stepping around at %.2f m, %.2f m was asked for",
                    chosen.achieved, context_.settings.detour_clearance_);
    }

    plan.route = { chosen.before, chosen.after, goal };
    // The straddle was planned for the leg boat -> goal, so that is the
    // direction "past it" is measured along later.
    plan.commitment = Commitment{ chosen_blob, chosen.before, chosen.after, bearing(boat.position, goal) };
    return plan;
}

bool ApproachObject::supersedes(std::optional<Commitment> const &fresh, Boat const &boat) const
{
    if (!commitment_ || !fresh)
    {
        return false;
    }

    // Blobs carry no identity from one frame to the next, so "the same
    // obstacle" has to be "close enough to the one remembered". match_radius
    // is the tolerance the lock already uses to re-find an object it is
    // tracking, and the job here is the same one.
    if (distance(fresh->obstacle.centre, commitment_->obstacle.centre) <= context_.settings.match_radius_)
    {
        return false;
    }

    // Something else, but further off than the obstacle being stepped around.
    // Deal with it after this one rather than instead of it: swapping now
    // would drop the committed pair half way through and cut straight back
    // across the near obstacle, which is exactly what the commitment is for.
    return distance(boat.position, fresh->obstacle.centre) < distance(boat.position, commitment_->obstacle.centre);
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

            Plan const plan = plan_route(boat);
            route_ = plan.route;
            commitment_ = plan.commitment;
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
            // An EMPTY blob list is not evidence the water is clear. blobs()
            // returns {} both when the clustering genuinely sees nothing and
            // when the transform lookup throws, and those mean opposite
            // things here. Tell them apart by the situation rather than the
            // list: the approach is locked onto a blob, so during a reverse
            // the clustering should always be reporting at least that one.
            // Nothing at all means we have stopped seeing, not that there is
            // nothing to see -- and reversing on a blind reading is the one
            // thing reverser.hpp tells callers never to do.
            std::vector<Blob> const behind_us = context_.lock.blobs();
            if (behind_us.empty())
            {
                context_.reverser.stop();
                RCLCPP_WARN(context_.node->get_logger(),
                            "approach: cannot see behind us (%s); stopping the reverse rather than guessing",
                            context_.lock.why().c_str());
                Plan const plan = plan_route(boat);
                route_ = plan.route;
                commitment_ = plan.commitment;
                context_.driver.go_to(route_);
                phase_ = Phase::Driving;
                return Status::Running;
            }

            if (!clear_behind(behind_us, boat.position, boat.direction, reverse_distance_,
                              context_.settings.hull_half_width_, context_.settings.hull_behind_))
            {
                context_.reverser.stop();
                RCLCPP_WARN(context_.node->get_logger(), "approach: something behind us, taking the best route "
                                                         "available instead");
                Plan const plan = plan_route(boat);
                route_ = plan.route;
                commitment_ = plan.commitment;
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
                Plan const plan = plan_route(boat);
                route_ = plan.route;
                commitment_ = plan.commitment;
                context_.driver.go_to(route_);
                phase_ = Phase::Driving;
            }
            return Status::Running;
        }

        case Phase::Driving:
        {
            // Judge arrival on the STANDOFF, which is what this maneuver
            // promises, not on nearness to the goal waypoint.
            //
            // driver.arrived() asks whether the boat is within
            // arrive_tolerance of the last waypoint, and that tolerance is
            // 1.5 m against a 3.0 m standoff -- so the approach was allowed to
            // stop half a standoff short and call it success. Measured in
            // simulation on 2026-09-07: asked to stop 3.22 m from a buoy, it
            // reported "arrived" at 4.70 m, short by exactly arrive_tolerance,
            // having driven barely half a metre. The gap also moves whenever
            // the lock does, so the error is not even consistent.
            //
            // guidance parks on the final waypoint, so it keeps closing on its
            // own; the maneuver simply has to stop declaring victory early.
            double const wanted = context_.lock.radius() + standoff_ + context_.settings.hull_front_;
            double const actual = distance(boat.position, context_.lock.point());
            if (actual <= wanted + context_.settings.standoff_tolerance_)
            {
                context_.driver.release();
                context_.spinner.stop();  // release only silences guidance; this stops the boat
                RCLCPP_INFO(context_.node->get_logger(),
                            "approach: arrived, bow %.2f m from the object's surface (asked for %.2f)",
                            actual - context_.lock.radius() - context_.settings.hull_front_, standoff_);
                return Status::Succeeded;
            }

            // Still short, but guidance thinks it has parked. Nothing more is
            // coming, so say what was actually achieved rather than waiting
            // out the timeout in silence.
            if (context_.driver.arrived(boat.position))
            {
                RCLCPP_WARN_THROTTLE(context_.node->get_logger(), *context_.node->get_clock(), 2000,
                                     "approach: guidance has parked %.2f m from the object, %.2f m short of the "
                                     "standoff; still closing",
                                     actual, actual - wanted);
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

            // Let a committed straddle go once the obstacle it steps around is
            // genuinely behind the hull. From there the direct line to the
            // goal cannot cross it, so there is nothing left to hold.
            if (commitment_ && !obstacle_ahead(boat.position, commitment_->travel, commitment_->obstacle,
                                               context_.settings.hull_behind_))
            {
                commitment_.reset();
                RCLCPP_INFO(context_.node->get_logger(), "approach: past the obstacle, back on the direct line");
            }

            // The picture can change while driving: the object's estimate can
            // shift, or a blob can move into the way. Re-hand the route only
            // when it has actually changed, since a new plan restarts
            // guidance's leg from the boat's current position.
            Plan plan = plan_route(boat);

            // KEEP THE COMMITTED STRADDLE. This is not a missing optimisation
            // and must not be tidied away: the plan above is made from where
            // the boat is NOW, and driving to the first straddle waypoint
            // moves the boat sideways, which pushes the obstacle further off
            // the line from here to the goal. Part way along that first leg
            // the obstacle stops counting as blocking at all, plan_route
            // collapses to the goal alone, and guidance is handed a straight
            // line back across the very obstacle it was stepping around --
            // delivering min_gap instead of detour_clearance, whatever
            // detour_clearance is set to. The pair only works as a pair, so it
            // is held until obstacle_ahead() says the obstacle is behind us.
            //
            // The route is still live: the goal is re-taken from the plan
            // every tick, so the target moving is followed, and a different,
            // nearer obstacle turning up replaces the commitment outright.
            if (commitment_ && !supersedes(plan.commitment, boat))
            {
                // Waypoints already behind the boat are left out. guidance
                // restarts at waypoint 0 from wherever the boat is on every
                // hand-over, so a passed waypoint left in the list would turn
                // the boat round to go back and collect it.
                plan.route.clear();
                for (Point const &waypoint : { commitment_->before, commitment_->after })
                {
                    if (along_track(boat.position, commitment_->travel, waypoint) > 0.0)
                    {
                        plan.route.push_back(waypoint);
                    }
                }
                plan.route.push_back(plan.goal);
                plan.commitment = commitment_;
            }
            commitment_ = plan.commitment;

            if (worth_replanning(plan.route))
            {
                route_ = plan.route;
                context_.driver.go_to(route_);
                RCLCPP_INFO(context_.node->get_logger(), "approach: route changed, now %zu point(s)", route_.size());
            }

            return Status::Running;
        }
    }

    return Status::Running;
}

}  // namespace prop_maneuvers
