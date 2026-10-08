/**
 * @file maneuvers.hpp
 * @brief The three reusable maneuvers, as small state machines.
 *
 * Each maneuver is stepped on a timer and reports Running, Succeeded or
 * Failed. They share a Context holding the boat's position, the driver, the
 * spinner, the reverser and the target lock.
 *
 * The rule every maneuver follows: only one thing commands the motors at a
 * time. Before the spinner is used, the driver is released; before the driver
 * is used, the spinner is stopped. The reverser is a third claimant on the
 * same cmd_vel topic and obeys the same rule at both ends: the driver is
 * released before it starts, and it is stopped before anything else takes
 * over -- including when a maneuver times out part-way through a reverse.
 */

#pragma once

#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "prop_maneuvers/constants.hpp"
#include "prop_maneuvers/driver.hpp"
#include "prop_maneuvers/geometry.hpp"
#include "prop_maneuvers/reverser.hpp"
#include "prop_maneuvers/spinner.hpp"
#include "prop_maneuvers/target_lock.hpp"

#include <nav_msgs/msg/odometry.hpp>

namespace prop_maneuvers
{

enum class Status
{
    Running,
    Succeeded,
    Failed,
};

/// Where the boat is and which way it points, from the position estimate.
struct Boat
{
    Point position;
    double direction{ 0.0 };
    /// Forward speed in m/s, body frame; the quantity thruster_manager closes its loop on.
    double surge{ 0.0 };
    /// Turn rate in rad/s, left positive. A turn is over when the boat stops, not when the command does.
    double yaw_rate{ 0.0 };
    bool valid{ false };
};

/// Everything a maneuver needs, built once per program.
class Context
{
  public:
    Context(rclcpp::Node *node, Constants const &settings);

    Boat boat() const
    {
        return boat_;
    }

    rclcpp::Node *node;
    Constants const &settings;
    Driver driver;
    Spinner spinner;
    Reverser reverser;
    TargetLock lock;

  private:
    Boat boat_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
};

/// Times a maneuver out using the node clock, which follows simulated time. Never use wall-clock: sim speed varies
/// tenfold.
class Deadline
{
  public:
    Deadline(rclcpp::Node *node, double seconds);
    bool expired() const;

  private:
    rclcpp::Node *node_;
    mutable rclcpp::Time start_;
    mutable bool started_{ false };
    double seconds_;
};

/// Something that can be stepped and says how it went, so one runner can drive any maneuver.
class Maneuver
{
  public:
    virtual ~Maneuver() = default;
    virtual Status step() = 0;
};

/// Hold position and turn until the front of the boat points at the lock.
/// Spinner::step() stops commanding once the error is inside point_tolerance, but the boat coasts on past
/// it (measured: 6.84 deg off against a 5 deg tolerance). So the turn ends, the boat is allowed to stop,
/// and the error is re-measured at rest; a miss turns again from a standstill, which overshoots less.
class FaceObject : public Maneuver
{
  public:
    explicit FaceObject(Context &context);
    Status step() override;

  private:
    enum class Phase
    {
        Turning,
        Settling,
    };

    /// Re-aims allowed after settling before reporting what was achieved; a backstop, not a budget.
    static constexpr int kMaxCorrections{ 3 };

    Context &context_;
    Deadline deadline_;
    Phase phase_{ Phase::Turning };
    std::optional<Deadline> settle_deadline_;
    int corrections_{ 0 };
};

/// Go all the way round the locked object along straight legs.
/// The boat cannot slide sideways, so the circle is a ring of corners. It does not stop at a corner: it
/// hands guidance the next one and keeps moving (stopping to face the buoy cost a turn per corner and
/// bulged the path inward, and refresh() ignores heading anyway).
/// While circling the buoy is abeam, which always overlaps an antenna blind wedge (they reach out to
/// 90 deg), so a failed refresh mid-leg is expected; only a stale lock stops the maneuver.
class CircleObject : public Maneuver
{
  public:
    CircleObject(Context &context, double radius, int legs, bool counter_clockwise);
    Status step() override;

  private:
    enum class Phase
    {
        Enter,          ///< pin the lap to the entry bearing and set off
        DriveToEntry,   ///< run out to the ring
        DriveToCorner,  ///< run the leg, rolling straight on at each corner
    };

    /// `corner` pushed along the line of travel by guidance's hold radius, so guidance parking short lands on it.
    Point aim_past(Point const &from, Point const &corner) const;

    /// Drive to `corner`, aiming past it. Returns true once the boat is there.
    bool run_to_corner(Boat const &boat);

    /// Point guidance at the next corner, redrawn around the current lock. Does not stop the boat.
    void start_leg(Boat const &boat);

    Context &context_;
    Deadline deadline_;
    double radius_;
    int legs_;
    bool counter_clockwise_;

    Phase phase_{ Phase::Enter };
    int legs_driven_{ 0 };
    rclcpp::Time last_refresh_;

    /// Bearing from the object out through the boat when the ring was entered; every corner is measured
    /// from it so the lap closes at exactly 360 degrees.
    double entry_bearing_{ 0.0 };

    /// The aim point handed to guidance. Remembered because "has guidance parked?" is about its waypoint.
    Point aim_{};

    /// The real corner on the ring. Arrival is judged against this.
    Point corner_{};
};

/// Face the object, drive to it, and stop short of it.
/// LIMITATION: obstacle handling is a sidestep around one blocking blob at a time, not path planning.
/// If the boat starts too close to a blocking blob for any swing to clear it, it backs up once and re-plans.
class ApproachObject : public Maneuver
{
  public:
    ApproachObject(Context &context, double standoff);
    Status step() override;

  private:
    enum class Phase
    {
        FaceTarget,
        BackingOff,  ///< too close to swing around something; make room first
        Driving,
        Settling,  ///< standoff reached; hold here until the boat is actually stopped
    };

    /// A straddle the approach is driving and holding on to. The pair only works as a pair; see
    /// obstacle_ahead() and the replan site in Phase::Driving.
    struct Commitment
    {
        Blob obstacle;         ///< the blob the pair steps around
        Point before;          ///< the waypoint short of it
        Point after;           ///< the waypoint past it
        double travel{ 0.0 };  ///< direction the straddled leg runs in, radians
    };

    /// A route to hand guidance, and what it commits the boat to.
    struct Plan
    {
        std::vector<Point> route;              ///< the points, ending at the goal
        Point goal;                            ///< where to stop; always route.back()
        std::optional<Commitment> commitment;  ///< set only when the route straddles something
    };

    /// Where to aim so the boat ends at the requested standoff. The one definition of the goal, shared by
    /// plan_route and the back-off decision; accounts for the bow offset and guidance parking short.
    Point aim_goal(Point const &boat) const;

    /// Where to stop, plus straddle waypoints if something blocks the line.
    Plan plan_route(Boat const &boat) const;

    /// Plan from where the boat is, hand it to guidance and enter Phase::Driving.
    void begin_driving(Boat const &boat);

    /// True when a fresh straddle should replace the one in flight; only a different, nearer obstacle does.
    bool supersedes(std::optional<Commitment> const &fresh, Boat const &boat) const;

    /// True when `fresh` differs enough from the route in flight to hand over again (a hand-over restarts guidance's
    /// leg).
    bool worth_replanning(std::vector<Point> const &fresh) const;

    Context &context_;
    Deadline deadline_;
    double standoff_;

    Phase phase_{ Phase::FaceTarget };
    std::vector<Point> route_;

    // The straddle being driven, held for the whole detour rather than replanned (see Phase::Driving).
    std::optional<Commitment> commitment_;

    // Backing off. One reverse per approach; if still too close afterwards, take the best route available.
    // Reverser keeps no state, so the start point and heading to hold are stashed here and handed back each
    // step. Whether it is safe to keep reversing is re-read live every step.
    bool has_reversed_{ false };
    Point reverse_start_;
    double reverse_held_direction_{ 0.0 };

    rclcpp::Time last_refresh_;

    /// Started when the standoff is crossed so Settling cannot wait forever (the maneuver's own Deadline is too long).
    std::optional<Deadline> settle_deadline_;
};

}  // namespace prop_maneuvers
