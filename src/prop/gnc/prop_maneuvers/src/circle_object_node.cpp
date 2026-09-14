/**
 * @file circle_object_node.cpp
 * @brief Standalone program for the "circle an object" maneuver.
 *
 *   ros2 run prop_maneuvers circle_object --ros-args -p target_x:=-4.0 -p target_y:=-4.0
 *   ros2 run prop_maneuvers circle_object --ros-args \\
 *       -p target_x:=20.0 -p target_y:=-3.0 -p circle_legs:=6 *
 * ROS 2 needs --ros-args -p name:=value. A bare -p is silently treated as a
 * remap and the parameter never arrives.
 *
 * Everything except the line that builds the maneuver lives in ManeuverRunner.
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "prop_maneuvers/runner.hpp"

using namespace prop_maneuvers;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManeuverRunner>("circle_object",
                                                  [](Context &context)
                                                  {
                                                      return std::make_unique<CircleObject>(
                                                          context, context.settings.circle_radius_,
                                                          context.settings.circle_legs_,
                                                          context.settings.circle_counter_clockwise_);
                                                  }));
    rclcpp::shutdown();
    return 0;
}
