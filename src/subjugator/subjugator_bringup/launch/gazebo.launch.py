import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def generate_launch_description():
    # Setup to launch the simulator and Gazebo world
    gz_sim_world = DeclareLaunchArgument("world", default_value="robosub_2025.world")
    gz_sim = IncludeLaunchDescription(
        pkg_share("ros_gz_sim", "launch", "gz_sim.launch.py"),
        launch_arguments={
            "gz_args": [
                PathJoinSubstitution(
                    [
                        pkg_share("subjugator_gazebo"),
                        "worlds",
                        LaunchConfiguration("world"),
                    ],
                ),
                " --render-engine",
                " ogre2",
            ],
        }.items(),
    )

    # Get controller to use sim values
    set_sim_params = SetLaunchConfiguration(
        "param_file",
        pkg_share("subjugator_controller", "config", "sim_pid_controller.yaml"),
    )
    set_sim_time = SetParameter("use_sim_time", True)

    # Include the Subjugator_Setup Launch file
    subjugator_setup = IncludeLaunchDescription(
        pkg_share("subjugator_bringup", "launch", "common.launch.py"),
        launch_arguments={
            "xacro_file": pkg_share(
                "subjugator_description",
                "urdf",
                "sub9_sim.urdf.xacro",
            ),
            "gui": "true",
        }.items(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": pkg_share(
                    "subjugator_bringup",
                    "config",
                    "subjugator_bridge.yaml",
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            },
        ],
        output="screen",
    )

    # Pinger heading transformer: converts world-frame pinger direction
    # from the Gazebo Hydrophone plugin (/ping) into body-frame direction
    # and publishes on hydrophones/solved for the mission planner.
    # This replaces the real-world ping_publisher.py which reads from
    # actual hydrophone hardware.
    pinger_heading = Node(
        package="subjugator_gazebo",
        executable="pinger_heading_node.py",
        name="pinger_heading_node",
        output="screen",
    )

    return LaunchDescription(
        [
            gz_sim_world,
            gz_sim,
            set_sim_params,
            set_sim_time,
            subjugator_setup,
            bridge,
            pinger_heading,
        ],
    )
