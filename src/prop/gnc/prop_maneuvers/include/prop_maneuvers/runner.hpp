/**
 * @file runner.hpp
 * @brief Runs one maneuver as a standalone program.
 */

#pragma once

#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "prop_maneuvers/constants.hpp"
#include "prop_maneuvers/maneuvers.hpp"

namespace prop_maneuvers
{

/// Waits for a position estimate, acquires the lock the caller asked for, then
/// steps a maneuver until it finishes and exits.
///
/// One class rather than three programs. face_object, circle_object and
/// approach_object were the same 112 lines each -- the same parameters, the
/// same acquisition, the same give-up deadline, the same timer and the same
/// exit -- differing only in which maneuver they constructed.
class ManeuverRunner : public rclcpp::Node, public Constants
{
  public:
    /// Builds the maneuver, once a lock exists. Settings reach it through
    /// context.settings, so the factory needs nothing else handed to it.
    using Factory = std::function<std::unique_ptr<Maneuver>(Context &)>;

    ManeuverRunner(std::string const &name, Factory make);

  private:
    /// A maneuver that can never see its target must say so and exit, not spin
    /// forever. Separate from the Deadline inside the maneuver itself, which
    /// only starts once a lock exists and times out the drive.
    bool give_up_if_stuck(char const *why);

    void tick();

    Factory make_;
    bool use_front_{ false };
    double target_x_{ 0.0 };
    double target_y_{ 0.0 };
    std::unique_ptr<Context> context_;
    std::unique_ptr<Maneuver> maneuver_;
    rclcpp::TimerBase::SharedPtr timer_;
    Deadline acquire_deadline_;
};

}  // namespace prop_maneuvers
