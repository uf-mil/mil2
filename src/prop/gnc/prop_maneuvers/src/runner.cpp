#include "prop_maneuvers/runner.hpp"

#include <utility>

namespace prop_maneuvers
{

ManeuverRunner::ManeuverRunner(std::string const &name, Factory make)
  : rclcpp::Node(name), Constants(this), make_(std::move(make)), acquire_deadline_(this, maneuver_timeout_)
{
    use_front_ = declare_parameter("use_front", false);
    target_x_ = declare_parameter("target_x", 0.0);
    target_y_ = declare_parameter("target_y", 0.0);

    context_ = std::make_unique<Context>(this, *this);

    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { tick(); });
}

bool ManeuverRunner::give_up_if_stuck(char const *why)
{
    if (!acquire_deadline_.expired())
    {
        return false;
    }
    RCLCPP_ERROR(get_logger(), "could not lock on within %.0f s: %s", maneuver_timeout_, why);
    timer_->cancel();
    rclcpp::shutdown();
    return true;
}

void ManeuverRunner::tick()
{
    Boat const boat = context_->boat();
    if (!boat.valid)
    {
        if (give_up_if_stuck("no position estimate"))
        {
            return;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for the position estimate");
        return;
    }

    // Acquire once, on the first tick that has both a position and blobs.
    if (!context_->lock.locked())
    {
        bool const got = use_front_ ? context_->lock.acquire_in_front(boat.position, boat.direction) :
                                      context_->lock.acquire_near(Point{ target_x_, target_y_ });
        if (!got)
        {
            if (give_up_if_stuck(context_->lock.why().c_str()))
            {
                return;
            }
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "waiting to lock on: %s",
                                 context_->lock.why().c_str());
            return;
        }
        maneuver_ = make_(*context_);
    }

    switch (maneuver_->step())
    {
        case Status::Running:
            return;
        case Status::Succeeded:
            RCLCPP_INFO(get_logger(), "succeeded");
            break;
        case Status::Failed:
            RCLCPP_ERROR(get_logger(), "failed");
            break;
    }

    timer_->cancel();
    rclcpp::shutdown();
}

}  // namespace prop_maneuvers
