/**
 * @file maneuvers.hpp
 * @brief The three reusable maneuvers, as small state machines.
 *
 * Each maneuver is stepped on a timer and reports Running, Succeeded or
 * Failed. They share a Context holding the boat's position, the driver, the
 * spinner and the target lock.
 *
 * The rule every maneuver follows: only one thing commands the motors at a
 * time. Before the spinner is used, the driver is released; before the driver
 * is used, the spinner is stopped.
 */

#pragma once

#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "prop_maneuvers/constants.hpp"
#include "prop_maneuvers/driver.hpp"
#include "prop_maneuvers/geometry.hpp"
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

}  // namespace prop_maneuvers
