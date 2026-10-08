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
  , reverser(node_in, settings_in.reverse_speed_, settings_in.turn_gain_, settings_in.max_turn_rate_)
  , lock(node_in, settings_in)
{
    odometry_subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered/global", 10,
        [this](nav_msgs::msg::Odometry::SharedPtr const msg)
        {
            boat_.position = Point{ msg->pose.pose.position.x, msg->pose.pose.position.y };
            boat_.direction = tf2::getYaw(msg->pose.pose.orientation);
            boat_.surge = msg->twist.twist.linear.x;
            boat_.yaw_rate = msg->twist.twist.angular.z;
            boat_.valid = true;
        });
}

Deadline::Deadline(rclcpp::Node *node, double seconds) : node_(node), start_(node->now()), seconds_(seconds)
{
    // With use_sim_time, now() reads 0 until the first /clock message; starting
    // the count from that would make the deadline expire immediately.
    started_ = start_.nanoseconds() > 0;
}

bool Deadline::expired() const
{
    rclcpp::Time const now = node_->now();

    if (!started_)
    {
        if (now.nanoseconds() == 0)
        {
            return false;
        }
        start_ = now;
        started_ = true;
        return false;
    }

    return (now - start_).seconds() > seconds_;
}

namespace
{
/// How often a maneuver re-confirms the object while driving, in node-clock seconds. 0.3 s is well under
/// reading_max_age (5.0 s) and short enough to fire on a close-in approach at low RTF.
constexpr double kRefreshInterval{ 0.3 };

/// A route point further than this from the one in flight is worth handing to guidance again.
constexpr double kReplanDistance{ 0.5 };

/// A heading change this big is worth handing to guidance again; smaller drift of the bearing is not.
constexpr double kRepublishHeading{ 3.0 * M_PI / 180.0 };

/// Keep the boat where it is. Unlike release(), guidance keeps publishing zero speed, so thruster_manager does not
/// cut thrust and let the boat drift. Falls back to release() when there is no position yet.
void hold_station(Context &context)
{
    Boat const boat = context.boat();
    if (boat.valid)
    {
        context.driver.go_to(boat.position);
    }
    else
    {
        context.driver.release();
    }
}

/// Why a step cannot go on yet, if it cannot: no position estimate (keep waiting) or no lock (fail).
std::optional<Status> not_ready(Context &context, Boat const &boat, char const *name)
{
    if (!boat.valid)
    {
        RCLCPP_WARN_THROTTLE(context.node->get_logger(), *context.node->get_clock(), 2000,
                             "%s: waiting for the position estimate", name);
        return Status::Running;
    }

    if (!context.lock.locked())
    {
        RCLCPP_ERROR(context.node->get_logger(), "%s: nothing locked on", name);
        return Status::Failed;
    }

    return std::nullopt;
}

/// Re-confirm the lock if `kRefreshInterval` has passed since `last_refresh`. A failed refresh is normal
/// (blind wedges, dropped frames); only stale() means lost. `warn_on_failure` says so while still holding a
/// remembered point.
void refresh_if_due(Context &context, rclcpp::Time &last_refresh, bool warn_on_failure = false)
{
    if ((context.node->now() - last_refresh).seconds() < kRefreshInterval)
    {
        return;
    }
    if (!context.lock.refresh() && warn_on_failure && !context.lock.stale())
    {
        RCLCPP_WARN_THROTTLE(context.node->get_logger(), *context.node->get_clock(), 2000,
                             "approach: keeping the remembered point (%s)", context.lock.why().c_str());
    }
    last_refresh = context.node->now();
}

/// Settling ends when the boat has stopped or, if there is one, the deadline runs out.
struct Settled
{
    bool stopped;
    bool gave_up;

    explicit operator bool() const
    {
        return stopped || gave_up;
    }
};

Settled settle(bool stopped, std::optional<Deadline> const &deadline)
{
    return Settled{ stopped, deadline && deadline->expired() };
}
}  // namespace

FaceObject::FaceObject(Context &context)
  : context_(context), deadline_(context.node, context.settings.maneuver_timeout_)
{
}

Status FaceObject::step()
{
    if (deadline_.expired())
    {
        hold_station(context_);
        RCLCPP_ERROR(context_.node->get_logger(), "face: gave up after %.0f s", context_.settings.maneuver_timeout_);
        return Status::Failed;
    }

    Boat const boat = context_.boat();
    if (auto const early = not_ready(context_, boat, "face"))
    {
        return *early;
    }

    double const target = bearing(boat.position, context_.lock.point());

    if (phase_ == Phase::Turning)
    {
        // Guidance does the turning; hand it the heading on entry and again only if the bearing has moved.
        if (!published_heading_ || std::abs(wrap_angle(target - *published_heading_)) > kRepublishHeading)
        {
            context_.driver.turn_to(boat.position, target);
            published_heading_ = target;
        }

        if (std::abs(wrap_angle(target - boat.direction)) > context_.settings.point_tolerance_)
        {
            return Status::Running;
        }

        // Inside the tolerance is not finished: let the remaining turn play out.
        settle_deadline_.emplace(context_.node, context_.settings.stop_timeout_);
        phase_ = Phase::Settling;
        RCLCPP_INFO(context_.node->get_logger(), "face: inside the tolerance at %.1f deg/s, letting it settle",
                    degrees(boat.yaw_rate));
        return Status::Running;
    }

    Settled const settled = settle(std::abs(boat.yaw_rate) <= context_.settings.stop_turn_rate_, settle_deadline_);
    if (!settled)
    {
        RCLCPP_INFO_THROTTLE(context_.node->get_logger(), *context_.node->get_clock(), 1000,
                             "face: settling, %.1f deg/s", degrees(boat.yaw_rate));
        return Status::Running;
    }

    // Re-measure at rest; the error at the moment of crossing was mid-sweep.
    double const resting = wrap_angle(target - boat.direction);

    if (std::abs(resting) <= context_.settings.point_tolerance_)
    {
        hold_station(context_);
        RCLCPP_INFO(context_.node->get_logger(), "face: pointing at (%.1f, %.1f), %.1f deg off",
                    context_.lock.point().x, context_.lock.point().y, degrees(resting));
        return Status::Succeeded;
    }

    if (settled.gave_up)
    {
        hold_station(context_);
        RCLCPP_WARN(context_.node->get_logger(),
                    "face: still turning at %.1f deg/s after %.0f s; reporting %.1f deg off", degrees(boat.yaw_rate),
                    context_.settings.stop_timeout_, degrees(resting));
        return Status::Succeeded;
    }

    if (++corrections_ > kMaxCorrections)
    {
        hold_station(context_);
        RCLCPP_WARN(context_.node->get_logger(), "face: settled %.1f deg off after %d corrections; reporting that",
                    degrees(resting), kMaxCorrections);
        return Status::Succeeded;
    }

    // Overshot. Turn again, this time from a standstill.
    RCLCPP_INFO(context_.node->get_logger(), "face: overshot to %.1f deg, correction %d of %d", degrees(resting),
                corrections_, kMaxCorrections);
    settle_deadline_.reset();
    published_heading_.reset();
    phase_ = Phase::Turning;
    return Status::Running;
}

CircleObject::CircleObject(Context &context, double radius, int legs, bool counter_clockwise)
  : context_(context)
  , deadline_(context.node, context.settings.maneuver_timeout_)
  , radius_(radius)
  , legs_(legs)
  , counter_clockwise_(counter_clockwise)
  , last_refresh_(context.node->now())
{
}

Point CircleObject::aim_past(Point const &from, Point const &corner) const
{
    // Guidance stops hold_radius short of its last waypoint, which would park
    // the boat inside the ring; aim that far past the corner (see aim_goal).
    double const span = distance(from, corner);
    if (span < 1e-6)
    {
        return corner;
    }
    double const scale = (span + context_.settings.guidance_hold_radius_) / span;
    return Point{ from.x + (corner.x - from.x) * scale, from.y + (corner.y - from.y) * scale };
}

bool CircleObject::run_to_corner(Boat const &boat)
{
    // Judged on the real corner, not driver.arrived(), whose tolerance is
    // looser than guidance's hold radius.
    if (distance(boat.position, corner_) <= context_.settings.corner_tolerance_)
    {
        return true;
    }

    // Backstop: once guidance has parked at the aim point nothing closes the
    // gap, so take what was achieved instead of hanging until the timeout.
    if (distance(boat.position, aim_) <= context_.settings.guidance_hold_radius_)
    {
        RCLCPP_WARN(context_.node->get_logger(),
                    "circle: guidance parked %.2f m from the corner, outside the %.2f m wanted; taking it",
                    distance(boat.position, corner_), context_.settings.corner_tolerance_);
        return true;
    }

    return false;
}

Status CircleObject::step()
{
    if (deadline_.expired())
    {
        hold_station(context_);
        RCLCPP_ERROR(context_.node->get_logger(), "circle: gave up after %.0f s on leg %d of %d",
                     context_.settings.maneuver_timeout_, legs_driven_, legs_);
        return Status::Failed;
    }

    Boat const boat = context_.boat();
    if (auto const early = not_ready(context_, boat, "circle"))
    {
        return *early;
    }

    // Keep the lock's timestamp fresh; the buoy sits in the blind wedges for part of each leg.
    refresh_if_due(context_, last_refresh_);

    if (context_.lock.stale())
    {
        hold_station(context_);
        RCLCPP_ERROR(context_.node->get_logger(),
                     "circle: lost the buoy on leg %d of %d (%s); stopping rather than circling a memory",
                     legs_driven_ + 1, legs_, context_.lock.why().c_str());
        return Status::Failed;
    }

    switch (phase_)
    {
        case Phase::Enter:
        {
            if (legs_ < 3)
            {
                RCLCPP_ERROR(context_.node->get_logger(), "circle: %d legs is not a shape; need at least 3", legs_);
                return Status::Failed;
            }

            // Pin the lap to the line out through the boat so it need not double back.
            entry_bearing_ = bearing(context_.lock.point(), boat.position);
            corner_ = ring_corner(context_.lock.point(), radius_, entry_bearing_, 0, legs_, counter_clockwise_);
            aim_ = aim_past(boat.position, corner_);

            context_.driver.go_to(aim_);
            phase_ = Phase::DriveToEntry;
            RCLCPP_INFO(context_.node->get_logger(), "circle: running out to the ring at %.1f m", radius_);
            return Status::Running;
        }

        case Phase::DriveToEntry:
        {
            if (!run_to_corner(boat))
            {
                return Status::Running;
            }

            legs_driven_ = 0;
            start_leg(boat);
            phase_ = Phase::DriveToCorner;
            RCLCPP_INFO(context_.node->get_logger(), "circle: on the ring, starting %d legs", legs_);
            return Status::Running;
        }

        case Phase::DriveToCorner:
        {
            if (!run_to_corner(boat))
            {
                return Status::Running;
            }

            ++legs_driven_;

            if (legs_driven_ >= legs_)
            {
                hold_station(context_);
                RCLCPP_INFO(context_.node->get_logger(), "circle: finished %d of %d legs", legs_driven_, legs_);
                return Status::Succeeded;
            }

            // Roll straight on: a new plan replaces the old one without stopping.
            start_leg(boat);
            return Status::Running;
        }
    }

    return Status::Running;
}

void CircleObject::start_leg(Boat const &boat)
{
    // Redraw around the buoy's current position, at the angle this leg always ended on (see ring_corner).
    corner_ = ring_corner(context_.lock.point(), radius_, entry_bearing_, legs_driven_ + 1, legs_, counter_clockwise_);
    aim_ = aim_past(boat.position, corner_);
    context_.driver.go_to(aim_);
    RCLCPP_INFO(context_.node->get_logger(), "circle: leg %d of %d, to (%.1f, %.1f)", legs_driven_ + 1, legs_,
                corner_.x, corner_.y);
}

ApproachObject::ApproachObject(Context &context, double standoff)
  : context_(context), deadline_(context.node, context.settings.maneuver_timeout_), standoff_(standoff)
{
    // Phase::FaceTarget turns the boat, so guidance must not be publishing cmd_vel.
    context_.driver.release();
}

namespace
{
/// Whether `blob` is the locked target itself. It is never its own obstacle; the margin covers clustering and
/// the lock disagreeing on the edge.
bool is_target_blob(Context const &context, Blob const &blob)
{
    return distance(blob.centre, context.lock.point()) <= context.lock.radius() + context.settings.target_blob_margin_;
}

/// The worst thing blocking the leg from `boat` to `goal`, ignoring the target.
/// Separate from plan_route so "nothing in the way" stays distinct from "in the way and unroutable".
DetourNeed worst_need(Context const &context, Point const &boat, Point const &goal)
{
    DetourNeed worst = DetourNeed::None;
    for (auto const &blob : context.lock.blobs())
    {
        if (is_target_blob(context, blob))
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

Point ApproachObject::aim_goal(Point const &boat) const
{
    // Stop clear of the object's surface, measured from the bow (base_link is 0.760 m aft of it).
    // Aim guidance_hold_radius past that, since guidance parks short of its last waypoint.
    double const want_from_centre = context_.lock.radius() + standoff_ + context_.settings.hull_front_;

    // Floor at where the bow touches the object, not base_link. guidance_hold_radius is not tied to
    // prop_controller's hold_radius, so this is the only guard against driving into the object.
    double const floor = context_.lock.radius() + context_.settings.hull_front_;
    double const aim_from_centre = std::max(floor, want_from_centre - context_.settings.guidance_hold_radius_);
    return standoff_point(boat, context_.lock.point(), aim_from_centre);
}

ApproachObject::Plan ApproachObject::plan_route(Boat const &boat) const
{
    Point const goal = aim_goal(boat.position);

    Plan plan;
    plan.goal = goal;
    plan.route = { goal };

    // Step around whichever blocking blob is nearest the boat.
    Detour chosen;
    Blob chosen_blob;
    double nearest_block = std::numeric_limits<double>::max();

    for (auto const &blob : context_.lock.blobs())
    {
        if (is_target_blob(context_, blob))
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

    // Anything but a straddle gets the straight line; backing off is step()'s job and happens once.
    if (chosen.need != DetourNeed::Straddle)
    {
        return plan;
    }

    if (chosen.achieved < context_.settings.detour_clearance_ - 1e-6)
    {
        // Say so: the goal itself lies inside the clearance, which no routing can fix.
        RCLCPP_WARN(context_.node->get_logger(), "approach: stepping around at %.2f m, %.2f m was asked for",
                    chosen.achieved, context_.settings.detour_clearance_);
    }

    plan.route = { chosen.before, chosen.after, goal };
    // The straddle was planned for boat -> goal, so "past it" is measured along that.
    plan.commitment = Commitment{ chosen_blob, chosen.before, chosen.after, bearing(boat.position, goal) };
    return plan;
}

bool ApproachObject::supersedes(std::optional<Commitment> const &fresh, Boat const &boat) const
{
    if (!commitment_ || !fresh)
    {
        return false;
    }

    // Blobs have no identity across frames; match within match_radius, as the lock does.
    if (distance(fresh->obstacle.centre, commitment_->obstacle.centre) <= context_.settings.match_radius_)
    {
        return false;
    }

    // Another, farther obstacle waits its turn; swapping now would drop the committed pair half way.
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
        if (distance(fresh[i], route_[i]) > kReplanDistance)
        {
            return true;
        }
    }
    return false;
}

void ApproachObject::begin_driving(Boat const &boat)
{
    Plan const plan = plan_route(boat);
    route_ = plan.route;
    commitment_ = plan.commitment;
    context_.driver.go_to(route_);
    phase_ = Phase::Driving;
}

Status ApproachObject::step()
{
    if (deadline_.expired())
    {
        context_.reverser.stop();
        context_.driver.release();
        RCLCPP_ERROR(context_.node->get_logger(), "approach: gave up after %.0f s",
                     context_.settings.maneuver_timeout_);
        return Status::Failed;
    }

    Boat const boat = context_.boat();
    if (auto const early = not_ready(context_, boat, "approach"))
    {
        return *early;
    }

    switch (phase_)
    {
        case Phase::FaceTarget:
        {
            double const target = bearing(boat.position, context_.lock.point());
            if (!published_heading_ || std::abs(wrap_angle(target - *published_heading_)) > kRepublishHeading)
            {
                context_.driver.turn_to(boat.position, target);
                published_heading_ = target;
            }

            if (std::abs(wrap_angle(target - boat.direction)) > context_.settings.point_tolerance_)
            {
                return Status::Running;
            }

            // Pointed at it: re-read before planning. Stamp last_refresh_ before every exit from this phase;
            // a default-constructed Time has a different clock source, so subtracting it throws.
            context_.lock.refresh();
            last_refresh_ = context_.node->now();

            // The same goal plan_route uses, so the back-off decision is made on the leg that will be driven.
            Point const goal = aim_goal(boat.position);

            // Asked once, while stopped and pointed at the target (the only cheap moment to reverse).
            // has_reversed_ limits it to one.
            if (!has_reversed_ && worst_need(context_, boat.position, goal) == DetourNeed::TooCloseToSwing)
            {
                // One thing commands the motors at a time: stop guidance first.
                context_.driver.release();
                reverse_start_ = boat.position;
                reverse_held_direction_ = boat.direction;
                has_reversed_ = true;
                phase_ = Phase::BackingOff;
                RCLCPP_INFO(context_.node->get_logger(), "approach: too close to step around it, backing off %.1f m",
                            context_.settings.reverse_max_distance_);
                return Status::Running;
            }

            begin_driving(boat);
            RCLCPP_INFO(context_.node->get_logger(), "approach: %zu point route, stopping %.1f m clear", route_.size(),
                        standoff_);
            return Status::Running;
        }

        case Phase::BackingOff:
        {
            // Re-checked every step: a reverse takes seconds and Reverser has no sensors. An empty blob list is
            // not proof the water is clear (blobs() also returns {} when the transform throws), and the locked
            // blob should always be reported, so treat empty as blind.
            std::vector<Blob> const behind_us = context_.lock.blobs();
            if (behind_us.empty())
            {
                context_.reverser.stop();
                RCLCPP_WARN(context_.node->get_logger(),
                            "approach: cannot see behind us (%s); stopping the reverse rather than guessing",
                            context_.lock.why().c_str());
                begin_driving(boat);
                return Status::Running;
            }

            // Check only the water left to cover; the strip is measured from the boat's current position,
            // so passing the full distance would sweep about twice the reverse.
            double const still_to_cover =
                std::max(0.0, context_.settings.reverse_max_distance_ - distance(reverse_start_, boat.position));

            if (!clear_behind(behind_us, boat.position, boat.direction, still_to_cover,
                              context_.settings.hull_half_width_, context_.settings.hull_behind_))
            {
                context_.reverser.stop();
                RCLCPP_WARN(context_.node->get_logger(), "approach: something behind us, taking the best route "
                                                         "available instead");
                begin_driving(boat);
                return Status::Running;
            }

            // Keep the lock fresh during the reverse (about reading_max_age long). A failed refresh is fine;
            // Phase::Driving reports a lock that is truly lost.
            refresh_if_due(context_, last_refresh_);

            if (context_.reverser.step(boat.position, boat.direction, reverse_start_, reverse_held_direction_,
                                       context_.settings.reverse_max_distance_))
            {
                // step() already published its own zero; repeating it keeps the motors from depending on its internals.
                context_.reverser.stop();
                RCLCPP_INFO(context_.node->get_logger(), "approach: backed off, re-planning");
                begin_driving(boat);
            }
            return Status::Running;
        }

        case Phase::Driving:
        {
            // Judge arrival on the standoff, not driver.arrived(): its 1.5 m tolerance let the approach stop half a
            // standoff short.
            double const wanted = context_.lock.radius() + standoff_ + context_.settings.hull_front_;
            double const actual = distance(boat.position, context_.lock.point());

            // Aim at the standoff and use standoff_tolerance only as the pass mark. aim_goal() aims
            // guidance_hold_radius inside it, so guidance parks near `wanted`.
            bool const on_the_standoff = actual <= wanted;

            // Guidance and boat have stopped: accept anything inside the tolerance rather than timing out.
            bool const parked_close_enough = context_.driver.arrived(boat.position) &&
                                             std::abs(boat.surge) <= context_.settings.stop_speed_ &&
                                             actual <= wanted + context_.settings.standoff_tolerance_;

            if (on_the_standoff || parked_close_enough)
            {
                if (!on_the_standoff)
                {
                    RCLCPP_WARN(context_.node->get_logger(),
                                "approach: guidance stopped %.2f m short of the standoff and will not close it; "
                                "taking it, inside the %.2f m tolerance",
                                actual - wanted, context_.settings.standoff_tolerance_);
                }
                // Crossing the standoff is not arriving on it. Hold station with a one-waypoint-at-the-boat plan rather
                // than releasing: thruster_manager cuts thrust after command_timeout of silence.
                context_.driver.go_to(boat.position);
                settle_deadline_.emplace(context_.node, context_.settings.stop_timeout_);
                phase_ = Phase::Settling;
                RCLCPP_INFO(context_.node->get_logger(),
                            "approach: reached the standoff at %.2f m/s, holding here to settle", boat.surge);
                return Status::Running;
            }

            // Still short but guidance has parked: report what was achieved.
            if (context_.driver.arrived(boat.position))
            {
                RCLCPP_WARN_THROTTLE(context_.node->get_logger(), *context_.node->get_clock(), 2000,
                                     "approach: guidance has parked %.2f m from the object, %.2f m short of the "
                                     "standoff; still closing",
                                     actual, actual - wanted);
            }

            // Keep the lock fresh while driving, else stale() fires on the clock alone. One failed refresh is
            // survivable.
            refresh_if_due(context_, last_refresh_, true);

            // Stale: we would be driving at a memory. Stop.
            if (context_.lock.stale())
            {
                hold_station(context_);
                RCLCPP_ERROR(context_.node->get_logger(), "approach: lost the object (%s); stopping",
                             context_.lock.why().c_str());
                return Status::Failed;
            }

            // Release a committed straddle once the obstacle is behind the hull.
            if (commitment_ && !obstacle_ahead(boat.position, commitment_->travel, commitment_->obstacle,
                                               context_.settings.hull_behind_))
            {
                commitment_.reset();
                RCLCPP_INFO(context_.node->get_logger(), "approach: past the obstacle, back on the direct line");
            }

            // Re-hand the route only when it changed; a new plan restarts guidance's leg.
            Plan plan = plan_route(boat);

            // Keep the committed straddle; do not tidy away. Replanning from the current position collapses to the
            // goal alone once the first leg moves the obstacle off the line, sending guidance back across it.
            // The goal is still re-taken each tick, and a nearer obstacle replaces the commitment.
            if (commitment_ && !supersedes(plan.commitment, boat))
            {
                // Drop waypoints already behind the boat; guidance restarts at waypoint 0 on each hand-over.
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

        case Phase::Settling:
        {
            Settled const settled = settle(std::abs(boat.surge) <= context_.settings.stop_speed_, settle_deadline_);
            if (!settled)
            {
                RCLCPP_INFO_THROTTLE(context_.node->get_logger(), *context_.node->get_clock(), 1000,
                                     "approach: settling, %.2f m/s", boat.surge);
                return Status::Running;
            }

            // Report where the boat actually is now, not where it crossed.
            double const resting = distance(boat.position, context_.lock.point());
            double const bow = resting - context_.lock.radius() - context_.settings.hull_front_;

            hold_station(context_);

            if (settled.gave_up)
            {
                // Still moving after stop_timeout: not a failure, but the number is a snapshot.
                RCLCPP_WARN(context_.node->get_logger(),
                            "approach: still making %.2f m/s after %.0f s; reporting anyway", boat.surge,
                            context_.settings.stop_timeout_);
            }
            // Reads low by about 0.2 m: the lidar paints only a buoy's near face, so the cluster centre and radius
            // both read short. That belongs in the lock or clustering. standoff_tolerance only judges the result.
            if (std::abs(bow - standoff_) > context_.settings.standoff_tolerance_)
            {
                RCLCPP_WARN(context_.node->get_logger(),
                            "approach: stopped, bow %.2f m from the object's surface (asked for %.2f) -- outside "
                            "the %.2f m tolerance",
                            bow, standoff_, context_.settings.standoff_tolerance_);
                return Status::Succeeded;
            }

            RCLCPP_INFO(context_.node->get_logger(),
                        "approach: stopped, bow %.2f m from the object's surface (asked for %.2f)", bow, standoff_);
            return Status::Succeeded;
        }
    }

    return Status::Running;
}

}  // namespace prop_maneuvers
