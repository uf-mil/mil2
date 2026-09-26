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
    /// Forward speed in m/s, body frame, straight off the position estimate.
    /// This is the same quantity thruster_manager closes its loop on, so a
    /// maneuver asking "have I stopped?" and the controller trying to stop
    /// are talking about the same number.
    double surge{ 0.0 };
    /// Turn rate in rad/s, left positive, straight off the position estimate.
    /// The rotational twin of surge, and used for the same question: a turn is
    /// not over when the command stops, it is over when the boat stops.
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

/// Times a maneuver out using the node clock, which follows simulated time
/// when use_sim_time is set. Never use wall-clock here: simulation speed on
/// this machine has been measured varying more than tenfold between runs.
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

/// How often a maneuver re-confirms the object is actually still seen while
/// driving, in simulated seconds (this is compared against node clock time,
/// which follows use_sim_time -- see Deadline). Refreshing on literally every
/// 100 ms tick would work but logs at 10 Hz. 0.3 s is small next to the
/// default reading_max_age (5.0 s), so several refresh attempts happen before
/// a genuine loss would go stale, and it is also small next to a short final
/// approach: on this machine RTF has been measured as low as ~0.4x, so even a
/// two-second real-world drive can be under one second of simulated time -- a
/// coarser interval risked never firing at all on a close-in approach, not
/// just logging less often.
///
/// Shared by ApproachObject and CircleObject, which confirm the lock the same
/// way for the same reason.
constexpr double kRefreshInterval{ 0.3 };

/// What all three maneuvers are: something that can be stepped, which says how
/// it went. Exists so one runner can drive any of them -- see ManeuverRunner.
class Maneuver
{
  public:
    virtual ~Maneuver() = default;
    virtual Status step() = 0;
};

/// Hold position and turn until the front of the boat points at the lock.
///
/// Turning and then WAITING, in two phases, for the same reason the approach
/// settles before reporting: the boat carries its turn past the point where
/// the command stops. Spinner::step() stops commanding as soon as the error is
/// inside point_tolerance, but measured 2026-09-13 against ground truth, that
/// moment arrives with the boat still turning at 17 deg/s. It then coasted
/// 11.7 degrees further and came to rest 6.84 degrees off -- OUTSIDE the 5
/// degree tolerance it had just declared it was inside, and on the far side of
/// the target. The estimate was not at fault; the EKF's yaw matched ground
/// truth to 0.00 degrees.
///
/// So the turn now ends, the boat is allowed to stop, and the error is
/// RE-MEASURED where it actually came to rest. If that still misses, it turns
/// again -- from a standstill, so the second correction is much gentler than
/// the first and the overshoot shrinks with it.
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

    /// How many times to re-aim after settling before reporting whatever was
    /// achieved. Each correction starts from rest and is smaller than the last,
    /// so this is a backstop against a boat that will not sit still, not a
    /// budget anything is expected to spend.
    static constexpr int kMaxCorrections{ 3 };

    Context &context_;
    Deadline deadline_;
    bool released_{ false };
    Phase phase_{ Phase::Turning };
    std::optional<Deadline> settle_deadline_;
    int corrections_{ 0 };
};

/// Go all the way round the locked object along straight legs.
///
/// The boat cannot slide sideways, so a circle becomes a ring of corners with
/// a straight run between each pair. The boat does NOT stop at a corner: it
/// simply hands guidance the next one and keeps moving, re-reading the buoy
/// on the way round.
///
/// An earlier version stopped at every corner, turned to face the buoy, took
/// a reading and turned back. That was never needed. refresh() matches blobs
/// near the remembered point and does not look at the boat's heading at all,
/// so pointing the bow at the buoy bought a tidier picture and nothing else.
/// It cost two things: a turn of 360/legs at every corner, and an inward
/// bulge of about 1.4 m mid-leg as the boat carried the turn's momentum into
/// the start of each run.
///
/// Where the buoy sits while circling: dead abeam, by definition, sweeping
/// 90 -/+ 180/legs off the front across a leg -- 45 to 135 degrees for four
/// legs, 60 to 120 for six, 67.5 to 112.5 for eight. The antenna blind wedges
/// reach out to exactly 90 (see blind_spots_deg), so that arc always overlaps
/// one of them however many legs are used, and the buoy goes unseen for part
/// of every leg. A failed refresh mid-leg is therefore expected rather than
/// alarming; only a lock gone genuinely stale stops the maneuver.
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

    /// `corner` pushed further along the boat's line of travel by guidance's
    /// hold radius, so guidance parking short lands the boat ON the corner.
    Point aim_past(Point const &from, Point const &corner) const;

    /// Drive to `corner`, aiming past it. Returns true once the boat is there.
    bool run_to_corner(Boat const &boat);

    /// Point guidance at the next corner, re-drawn around the current lock.
    /// Does not release guidance or stop the boat -- a new plan replaces the
    /// old one, so the boat rolls through the corner still carrying way.
    void start_leg(Boat const &boat);

    Context &context_;
    Deadline deadline_;
    double radius_;
    int legs_;
    bool counter_clockwise_;

    Phase phase_{ Phase::Enter };
    int legs_driven_{ 0 };
    rclcpp::Time last_refresh_;

    /// Bearing from the object out through the boat when the ring was entered.
    /// Every corner is measured from this, NOT from wherever the boat happens
    /// to be, so the lap closes at exactly 360 degrees. See ring_corner.
    double entry_bearing_{ 0.0 };

    /// The aim point actually handed to guidance -- the corner pushed a hold
    /// radius further along. Remembered because "has guidance parked?" is a
    /// question about ITS waypoint and ITS hold radius, and asking it any
    /// other way silently changes meaning when either of those moves.
    Point aim_{};

    /// The corner being driven to -- the real one on the ring, not the aim
    /// point handed to guidance. Arrival is judged against this.
    Point corner_{};
};

/// Face the object, drive to it, and stop short of it.
///
/// LIMITATION: obstacle handling here is a sidestep, not path planning. It
/// steps around one blocking blob at a time and makes no promise in a crowded
/// field. Real planning is a separate effort.
///
/// When the boat starts so close to a blocking blob that no swing wide enough
/// to clear it exists, the approach backs straight up once to make room and
/// then re-plans. Once, deliberately -- see has_reversed_ below.
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

    /// A straddle the approach has started driving and is holding on to.
    ///
    /// The pair only works as a pair: the first waypoint carries the boat
    /// sideways, and the second holds that offset until the obstacle is
    /// behind. See obstacle_ahead() and the replan site in Phase::Driving for
    /// why it is remembered rather than re-derived every tick.
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

    /// Where to aim so the boat ends up at the requested standoff.
    ///
    /// The one definition of the goal, used both by plan_route and by the
    /// back-off decision, so the two can never judge different legs. It
    /// accounts for the bow being ahead of base_link and for guidance parking
    /// short of its final waypoint.
    Point aim_goal(Point const &boat) const;

    /// Where to stop, plus straddle waypoints if something blocks the line.
    Plan plan_route(Boat const &boat) const;

    /// True when a freshly planned straddle is reason enough to abandon the
    /// one already being driven. Only a DIFFERENT and NEARER obstacle is.
    bool supersedes(std::optional<Commitment> const &fresh, Boat const &boat) const;

    /// True when `fresh` is different enough from the route in flight to be
    /// worth handing over again. Re-publishing every tick would restart
    /// guidance's leg on every cycle.
    bool worth_replanning(std::vector<Point> const &fresh) const;

    Context &context_;
    Deadline deadline_;
    double standoff_;

    Phase phase_{ Phase::FaceTarget };
    bool released_for_turn_{ false };
    std::vector<Point> route_;

    // The straddle currently being driven, if any. Held for the whole of the
    // detour rather than re-planned from wherever the boat has got to, which
    // would throw it away half way through -- see Phase::Driving.
    std::optional<Commitment> commitment_;

    // Backing off. One reverse per approach: if the boat is still too close
    // afterwards, take the best route available rather than shuffling
    // backwards forever. Reversing again would only help if the picture had
    // changed, and the picture that put us here is the obstacle's, not ours.
    //
    // Reverser keeps no state of its own -- same as Spinner -- so the start
    // point and the heading to hold are stashed here on entering the phase
    // and handed back on every step. These three are the only things that are
    // deliberately snapshots; whether it is SAFE to keep reversing is re-read
    // live on every step instead, in Phase::BackingOff.
    bool has_reversed_{ false };
    Point reverse_start_;
    double reverse_held_direction_{ 0.0 };
    double reverse_distance_{ 0.0 };

    rclcpp::Time last_refresh_;

    /// Started when the standoff is crossed, so Settling cannot wait forever.
    /// Separate from the maneuver's own Deadline, which is 300 s and would let
    /// a boat that never settles hang for five minutes having already arrived.
    std::optional<Deadline> settle_deadline_;
};

}  // namespace prop_maneuvers
