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
    rclcpp::Time start_;
    double seconds_;
};

/// Hold position and turn until the front of the boat points at the lock.
class FaceObject
{
  public:
    explicit FaceObject(Context &context);
    Status step();

  private:
    Context &context_;
    Deadline deadline_;
    bool released_{ false };
};

/// Go all the way round the locked object along straight legs.
///
/// The boat cannot slide sideways, so a circle becomes a ring of corners with
/// a straight run between each pair. At every corner the boat stops, turns to
/// face the buoy -- which puts it straight ahead and clear of the blind spots
/// -- takes a fresh reading, and redraws the remaining corners around it.
///
/// Four legs gives the diamond. More legs keeps the buoy nearer the front of
/// the boat during each run: four spans 45 to 135 degrees off the front, six
/// spans 60 to 120, eight spans 67.5 to 112.5. Raise it if the blind spots
/// turn out worse than the guess in the settings file.
class CircleObject
{
  public:
    CircleObject(Context &context, double radius, int legs, bool counter_clockwise);
    Status step();

  private:
    enum class Phase
    {
        FaceForEntry,   ///< turn towards the buoy before working out the entry point
        DriveToEntry,   ///< run out to the ring
        FaceBuoy,       ///< at a corner: turn to the buoy and re-read it
        DriveToCorner,  ///< run the leg
    };

    /// Turn towards the remembered point. Returns true once pointed.
    bool turn_towards_buoy();

    Context &context_;
    Deadline deadline_;
    double radius_;
    int legs_;
    bool counter_clockwise_;

    Phase phase_{ Phase::FaceForEntry };
    int legs_driven_{ 0 };
    bool released_for_turn_{ false };
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
class ApproachObject
{
  public:
    ApproachObject(Context &context, double standoff);
    Status step();

  private:
    enum class Phase
    {
        FaceTarget,
        BackingOff,  ///< too close to swing around something; make room first
        Driving,
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

    // How often to re-confirm the object is actually still seen while
    // driving, in simulated seconds (this is compared against node clock
    // time, which follows use_sim_time -- see Deadline). Refreshing on
    // literally every 100 ms tick would work but logs at 10 Hz. 0.3 s is
    // small next to the default reading_max_age (5.0 s), so several refresh
    // attempts happen before a genuine loss would go stale, and it is also
    // small next to a short final approach: on this machine RTF has been
    // measured as low as ~0.4x, so even a two-second real-world drive can be
    // under one second of simulated time -- a coarser interval risked never
    // firing at all on a close-in approach, not just logging less often.
    static constexpr double kRefreshInterval{ 0.3 };
    rclcpp::Time last_refresh_;
};

}  // namespace prop_maneuvers
