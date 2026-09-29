"""
Route planning: remember what the lidar saw, then steer around it.

    ros2 launch prop_planner planner.launch.py

    ros2 launch prop_planner planner.launch.py \
        odometry:=/odometry/filtered/global frame:=map

Needs tracked_markers from the pcd stack and an odometry estimate from
prop_localization, and publishes plan for prop_controller's guidance node to
follow.

The odometry argument and the frame argument travel together: the planner
compares the boat's position against the obstacles directly, so both have to
name the same frame. odometry/filtered/local is published in odom and
odometry/filtered/global in map. Picking one and leaving the other is the one
way to get a route that looks reasonable and goes to the wrong place, so the
node refuses to plan when they disagree. Send it somewhere to go with RViz's "2D Goal Pose" button, or by hand:

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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "odometry",
                default_value="/odometry/filtered/local",
                description="Odometry to follow. The local estimate is "
                "continuous; the global one is absolute but steps whenever GPS "
                "corrects it. Must match the frame argument.",
            ),
            DeclareLaunchArgument(
                "frame",
                default_value="odom",
                description="Frame to remember obstacles in and publish the "
                "plan in. Must be the frame the odometry argument is "
                "published in.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="True under Gazebo. Detections are transformed at "
                "the stamp they carry, so the clock has to match the one they "
                "were stamped on.",
            ),
            # Only set when asked. Left alone, the clock is whatever the bringup
            # that included this file chose, and an unconditional false here
            # would override it for every node launched after this one.
            SetParameter(
                "use_sim_time",
                True,
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="prop_planner",
                executable="planner",
                name="planner",
                parameters=[
                    pkg_share("prop_planner", "config", "planner.yaml"),
                    {"map_frame": LaunchConfiguration("frame")},
                ],
                remappings=[("odometry", LaunchConfiguration("odometry"))],
                output="screen",
            ),
        ],
    )
