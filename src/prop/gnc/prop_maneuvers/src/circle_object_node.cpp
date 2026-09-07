/**
 * @file circle_object_node.cpp
 * @brief Standalone program for the "circle an object" maneuver.
 *
 *   ros2 run prop_maneuvers circle_object --ros-args -p target_x:=-4.0 -p target_y:=-4.0
 *   ros2 run prop_maneuvers circle_object --ros-args \
 *       -p target_x:=20.0 -p target_y:=-3.0 -p circle_legs:=6
 *
 * ROS 2 needs --ros-args -p name:=value. A bare -p is silently treated as a
 * remap and the parameter never arrives.
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "prop_maneuvers/constants.hpp"
#include "prop_maneuvers/maneuvers.hpp"

using namespace prop_maneuvers;

class CircleObjectNode : public rclcpp::Node, public Constants
{
  public:
    CircleObjectNode() : rclcpp::Node("circle_object"), Constants(this)
    {
        use_front_ = declare_parameter("use_front", false);
        target_x_ = declare_parameter("target_x", 0.0);
        target_y_ = declare_parameter("target_y", 0.0);

        context_ = std::make_unique<Context>(this, *this);

        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { tick(); });
    }

  private:
    void tick()
    {
        Boat const boat = context_->boat();
        if (!boat.valid)
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for the position estimate");
            return;
        }

        if (!context_->lock.locked())
        {
            bool const got = use_front_ ? context_->lock.acquire_in_front(boat.position, boat.direction) :
                                          context_->lock.acquire_near(Point{ target_x_, target_y_ });
            if (!got)
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "waiting to lock on: %s",
                                     context_->lock.why().c_str());
                return;
            }
            maneuver_ =
                std::make_unique<CircleObject>(*context_, circle_radius_, circle_legs_, circle_counter_clockwise_);
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
    std::unique_ptr<CircleObject> maneuver_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircleObjectNode>());
    rclcpp::shutdown();
    return 0;
}
