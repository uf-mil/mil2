/**
 * @file approach_object_node.cpp
 * @brief Standalone program for the "approach an object" maneuver.
 *
 *   ros2 run prop_maneuvers approach_object --ros-args \
 *       -p target_x:=20.0 -p target_y:=-3.0
 *
 * ROS 2 needs --ros-args -p name:=value. A bare -p is silently treated as a
 * remap and the parameter never arrives.
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "prop_maneuvers/constants.hpp"
#include "prop_maneuvers/maneuvers.hpp"

using namespace prop_maneuvers;

class ApproachObjectNode : public rclcpp::Node, public Constants
{
  public:
    ApproachObjectNode() : rclcpp::Node("approach_object"), Constants(this), acquire_deadline_(this, maneuver_timeout_)
    {
        use_front_ = declare_parameter("use_front", false);
        target_x_ = declare_parameter("target_x", 0.0);
        target_y_ = declare_parameter("target_y", 0.0);

        context_ = std::make_unique<Context>(this, *this);

        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { tick(); });
    }

  private:
    // A maneuver that can never see its target must say so and exit, not
    // spin forever. This is separate from the Deadline inside ApproachObject
    // itself, which only starts once a lock exists and times out the drive.
    bool give_up_if_stuck(char const *why)
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

    void tick()
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
            maneuver_ = std::make_unique<ApproachObject>(*context_, approach_standoff_, detour_clearance_);
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

    bool use_front_{ false };
    double target_x_{ 0.0 };
    double target_y_{ 0.0 };
    std::unique_ptr<Context> context_;
    std::unique_ptr<ApproachObject> maneuver_;
    rclcpp::TimerBase::SharedPtr timer_;
    Deadline acquire_deadline_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ApproachObjectNode>());
    rclcpp::shutdown();
    return 0;
}
