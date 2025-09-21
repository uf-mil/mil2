import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch
    # Setup project paths
    pkg_project_bringup = get_package_share_directory("subjugator_bringup")
    pkg_project_gazebo = get_package_share_directory("subjugator_gazebo")
    # pkg_project_description = get_package_share_directory("subjugator_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_controller = get_package_share_directory("subjugator_controller")

    # Setup to launch the simulator and Gazebo world
    gz_sim_world = DeclareLaunchArgument("world", default_value="robosub_2025.world")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py"),
        ),
        launch_arguments={
            "gz_args": [
                PathJoinSubstitution(
                    [pkg_project_gazebo, "worlds", LaunchConfiguration("world")],
                ),
                " --render-engine",
                " ogre",
            ],
        }.items(),
    )

    # Get controller to use sim values
    sim_pid_yaml = os.path.join(pkg_controller, "config", "sim_pid_controller.yaml")
    set_sim_params = SetLaunchConfiguration("param_file", sim_pid_yaml)

    # Include the Subjugator_Setup Launch file
    subjugator_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, "launch", "subjugator_setup.launch.py"),
        ),
        launch_arguments={
            "use_sim_time": "true",
            "xacro_file": os.path.join(
                get_package_share_directory("subjugator_description"),
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
                "config_file": os.path.join(
                    pkg_project_bringup,
                    "config",
                    "subjugator_bridge.yaml",
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            },
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gz_sim_world,
            subjugator_setup,
            gz_sim,
            set_sim_params,
            bridge,
        ],
    )
