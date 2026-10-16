/**
 * @file approach_object_node.cpp
 * @brief Standalone program for the "approach an object" maneuver.
 *
 *   ros2 run prop_maneuvers approach_object --ros-args -p target_x:=-4.0 -p target_y:=-4.0
 *
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
    rclcpp::spin(std::make_shared<ManeuverRunner>(
        "approach_object", [](Context &context)
        { return std::make_unique<ApproachObject>(context, context.settings.approach_standoff_); }));
    rclcpp::shutdown();
    return 0;
}
