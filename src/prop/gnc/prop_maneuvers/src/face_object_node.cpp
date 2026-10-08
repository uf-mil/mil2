/**
 * @file face_object_node.cpp
 * @brief Standalone program for the "face an object" maneuver.
 *
 *   ros2 run prop_maneuvers face_object --ros-args -p target_x:=-4.0 -p target_y:=-4.0
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
    rclcpp::spin(std::make_shared<ManeuverRunner>("face_object", [](Context &context)
                                                  { return std::make_unique<FaceObject>(context); }));
    rclcpp::shutdown();
    return 0;
}
