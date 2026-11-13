"""
Route planning: build a map from the lidar, then steer around what is on it.

    ros2 launch prop_planner planner.launch.py

    ros2 launch prop_planner planner.launch.py \
        odometry:=/odometry/filtered/global frame:=map

Two planners, picked with planner:=

    grid        default. occupancy_grid turns the raw cloud into an evidence
                grid, and the planner runs A* over it. Needs the cloud topic
                and TF from its frame to the map frame.
    visibility  the capsule obstacle map fed by pcd's tracked_markers, with a
                visibility graph over it. Kept for comparison.

Either way the node is called planner, publishes plan for prop_controller's
guidance to follow, and starts idle until start-planning enables it.

The odometry argument and the frame argument travel together: the planner
compares the boat's position against the map directly, so both have to name
the same frame. odometry/filtered/local is published in odom and
odometry/filtered/global in map. Picking one and leaving the other is the one
way to get a route that looks reasonable and goes to the wrong place, so the
node refuses to plan when they disagree.

Send it somewhere to go with RViz's "2D Goal Pose" button, or by hand:

    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: map}, pose: {position: {x: 40.0, y: 0.0}, \
          orientation: {w: 1.0}}}"

What the grid planner sees is on occupancy_grid, and what it steers by, with
the hull's margin and the clearance it prefers, on costmap. Both draw with
RViz's Map display.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def generate_launch_description():
    frame = {"map_frame": LaunchConfiguration("frame")}
    odometry = [("odometry", LaunchConfiguration("odometry"))]
    grid = IfCondition(EqualsSubstitution(LaunchConfiguration("planner"), "grid"))
    visibility = IfCondition(
        EqualsSubstitution(LaunchConfiguration("planner"), "visibility"),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "planner",
                default_value="grid",
                choices=["grid", "visibility"],
                description="grid: an occupancy grid from the raw cloud and A* "
                "over it. visibility: the capsule obstacle map from "
                "tracked_markers and a visibility graph.",
            ),
            DeclareLaunchArgument(
                "cloud",
                default_value="/velodyne_points",
                description="Raw PointCloud2 the grid is built from.",
            ),
            DeclareLaunchArgument(
                "odometry",
                default_value="/odometry/filtered/global",
                description="Odometry to follow. The global estimate steps "
                "whenever GPS corrects it; the local one is continuous, but "
                "only once something feeds ekf_local a translation source - "
                "on the IMU alone it never moves at all. Must match frame.",
            ),
            DeclareLaunchArgument(
                "frame",
                default_value="map",
                description="Frame to keep the map in and publish the plan "
                "in. Must be the frame the odometry argument is published "
                "in. odom only works where ekf_local has a translation "
                "source, or buoys drift through it at hull speed.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="True under Gazebo. Clouds are transformed at the "
                "stamp they carry, so the clock has to match the one they "
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
                executable="occupancy_grid",
                name="occupancy_grid",
                parameters=[
                    pkg_share("prop_planner", "config", "grid_planner.yaml"),
                    frame,
                ],
                remappings=[("cloud", LaunchConfiguration("cloud"))],
                condition=grid,
                output="screen",
            ),
            Node(
                package="prop_planner",
                executable="grid_planner",
                name="planner",
                parameters=[
                    pkg_share("prop_planner", "config", "grid_planner.yaml"),
                    frame,
                ],
                remappings=odometry,
                condition=grid,
                output="screen",
            ),
            Node(
                package="prop_planner",
                executable="planner",
                name="planner",
                parameters=[pkg_share("prop_planner", "config", "planner.yaml"), frame],
                remappings=odometry,
                condition=visibility,
                output="screen",
            ),
        ],
    )
