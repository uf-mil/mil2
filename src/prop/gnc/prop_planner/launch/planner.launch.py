"""
Route planning: remember what the lidar saw, then steer around it.

    ros2 launch prop_planner planner.launch.py

Needs tracked_markers from the pcd stack and odometry/filtered/global from
prop_localization, and publishes plan for prop_controller's guidance node to
follow. Send it somewhere to go with RViz's "2D Goal Pose" button, or by hand:

    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: map}, pose: {position: {x: 40.0, y: 0.0}, \
          orientation: {w: 1.0}}}"

What it remembers is published on obstacle_map, drawn at the radius it actually
keeps clear rather than the one the lidar measured.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="True under Gazebo. Detections are transformed at "
                "the stamp they carry, so the clock has to match the one they "
                "were stamped on.",
            ),
            SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
            Node(
                package="prop_planner",
                executable="planner",
                name="planner",
                parameters=[pkg_share("prop_planner", "config", "planner.yaml")],
                output="screen",
            ),
        ],
    )
