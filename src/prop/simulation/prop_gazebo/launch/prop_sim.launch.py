"""
The simulated boat: sensors, localization, perception, planning and control.

    ros2 launch prop_gazebo prop_sim.launch.py
    ros2 launch prop_gazebo prop_sim.launch.py headless:=true
    ros2 launch prop_gazebo prop_sim.launch.py autostart:=false
    ros2 launch prop_gazebo prop_sim.launch.py software_rendering:=true
    ros2 launch prop_gazebo prop_sim.launch.py \
        world_pkg:=navigator_gazebo world:=robotx_2024.world

Gazebo supplies the hull and the sensors and nothing else: everything above
them is the same set of nodes the boat runs. With autostart, the default,
localization, the controller and the planner are enabled a few seconds in and
the boat holds station until it is given somewhere to go, with RViz's
"2D Goal Pose" or by hand:

    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: map}, pose: {position: {x: 10.0}, \
          orientation: {w: 1.0}}}"

From there the route goes planner -> trajectory_planner -> pid_controller ->
thruster_mapper -> sim_thrusters.py -> Gazebo. autostart:=false leaves the
services to you, the same ones as on the boat:

    prop-start-localization
    start-controller / stop-controller / reset-controller
    start-planning / stop-planning

By default the whole stack runs on ground truth: TF, the planner, the
trajectory follower and the controller all take the hull's pose from Gazebo,
and the EKFs still run but only publish their estimates for comparison. The
boat's own local estimate is rf2o lidar velocities integrated by an EKF, and
on an open lake with four buoys rf2o has nothing to track, so that estimate
never moves and the controller drives off without knowing it. Mixing is not
an option either: the GPS datum puts the estimate's map frame metres from
Gazebo's, so a route planned in one and driven in the other grazes buoys.
truth:=false puts the real estimator back in charge, for worlds with a
shoreline for rf2o to see.

The controller's gains are subjugator_controller_again/config/sim_pid_controller.yaml.
To tune them,  ros2 run prop_gazebo step_response.py  steps one axis and
measures the answer against ground truth.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

# Mesa's llvmpipe in place of the GPU, for a machine whose driver cannot give
# Gazebo an OpenGL 3.3 context. The lidar renders either way; slower here.
SOFTWARE_GL = {
    "__EGL_VENDOR_LIBRARY_FILENAMES": "/usr/share/glvnd/egl_vendor.d/50_mesa.json",
    "__GLX_VENDOR_LIBRARY_NAME": "mesa",
    "LIBGL_ALWAYS_SOFTWARE": "1",
    "GALLIUM_DRIVER": "llvmpipe",
    "MESA_GL_VERSION_OVERRIDE": "3.3",
    "EGL_PLATFORM": "surfaceless",
}


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def include(pkg, launch_file, **launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_share(pkg, "launch", launch_file)),
        launch_arguments=launch_arguments.items(),
    )


def generate_launch_description():
    # Any package that installs worlds/ will do, so the boat can be dropped
    # into the RobotX course as easily as into its own lake.
    world = PathJoinSubstitution(
        [
            FindPackageShare(LaunchConfiguration("world_pkg")),
            "worlds",
            LaunchConfiguration("world"),
        ],
    )

    def odometry_or_truth(estimate):
        return PythonExpression(
            [
                "'/sim/ground_truth' if '",
                LaunchConfiguration("truth"),
                f"' == 'true' else '{estimate}'",
            ],
        )

    control_odometry = odometry_or_truth("/odometry/filtered/local")
    planner_odometry = odometry_or_truth("/odometry/filtered/global")
    truth = IfCondition(LaunchConfiguration("truth"))
    estimate_tf = PythonExpression(
        ["'false' if '", LaunchConfiguration("truth"), "' == 'true' else 'true'"],
    )
    headless = PythonExpression(
        [
            "' -s --headless-rendering' if '",
            LaunchConfiguration("headless"),
            "' == 'true' else ''",
        ],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_share("ros_gz_sim", "launch", "gz_sim.launch.py"),
        ),
        launch_arguments={
            "gz_args": [world, " -r ", LaunchConfiguration("gz_args"), headless],
        }.items(),
    )

    spawn_prop = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "prop",
            "-file",
            pkg_share("prop_gazebo", "models", "prop", "model.sdf"),
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            # About where the hull settles, so it does not drop in.
            "-z",
            "0.06",
            "-Y",
            LaunchConfiguration("yaw"),
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {"config_file": pkg_share("prop_gazebo", "config", "prop_bridge.yaml")},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="prop_lake.world"),
            DeclareLaunchArgument(
                "world_pkg",
                default_value="prop_gazebo",
                description="Package the world lives in, under its worlds/ "
                "directory. navigator_gazebo for robotx_2024.world.",
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument(
                "yaw",
                default_value="0.0",
                description="Spawn heading in radians, ENU.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo without its window.",
            ),
            DeclareLaunchArgument(
                "software_rendering",
                default_value="false",
                description="Render on the CPU with Mesa, for when the GPU "
                "driver cannot give Gazebo OpenGL 3.3 and it crashes at "
                "the lidar.",
            ),
            *[
                SetEnvironmentVariable(
                    name,
                    value,
                    condition=IfCondition(LaunchConfiguration("software_rendering")),
                )
                for name, value in SOFTWARE_GL.items()
            ],
            DeclareLaunchArgument(
                "gz_args",
                default_value="--render-engine ogre2",
                description="Extra gz sim flags.",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Enable localization, the controller and the "
                "planner once the sim is up, instead of waiting for the "
                "service calls.",
            ),
            DeclareLaunchArgument(
                "planner",
                default_value="grid",
                choices=["grid", "visibility"],
                description="Which planner, see prop_planner's planner.launch.py.",
            ),
            DeclareLaunchArgument(
                "truth",
                default_value="true",
                description="Control against sim/ground_truth rather than the "
                "local estimate, which rf2o cannot hold on open water.",
            ),
            SetParameter("use_sim_time", True),
            gz_sim,
            spawn_prop,
            bridge,
            # Stand in for the thruster driver, and for the covariances the
            # bridge cannot carry.
            Node(
                package="prop_gazebo",
                executable="sim_thrusters.py",
                name="sim_thrusters",
                output="screen",
            ),
            Node(
                package="prop_gazebo",
                executable="sim_sensors.py",
                name="sim_sensors",
                output="screen",
            ),
            include(
                "prop_localization",
                "localization.launch.py",
                publish_tf=estimate_tf,
            ),
            Node(
                package="prop_gazebo",
                executable="sim_truth_tf.py",
                name="sim_truth_tf",
                condition=truth,
            ),
            include("pcd", "filter.launch.py"),
            include(
                "prop_planner",
                "planner.launch.py",
                planner=LaunchConfiguration("planner"),
                odometry=planner_odometry,
            ),
            Node(
                package="trajectory_planner",
                executable="trajectory_planner",
                remappings=[("/odometry/filtered/local", control_odometry)],
                output="screen",
            ),
            include(
                "subjugator_controller_again",
                "pid_controller.launch.py",
                param_file=pkg_share(
                    "subjugator_controller_again",
                    "config",
                    "sim_pid_controller.yaml",
                ),
                odometry=control_odometry,
            ),
            include("prop_controller", "thruster_mapper.launch.py"),
            Node(
                package="prop_gazebo",
                executable="sim_autostart.py",
                name="sim_autostart",
                output="screen",
                condition=IfCondition(LaunchConfiguration("autostart")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", pkg_share("prop_gazebo", "config", "prop_sim.rviz")],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ],
    )
