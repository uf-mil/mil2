import os
from pathlib import Path

import sdformat14 as sdf
from ament_index_python.packages import (
    get_package_share_directory,
    get_package_share_path,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from vrx_gz import bridges, payload_bridges


# Bridge ROS topics and Gazebo messages for establishing communication
def make_bridge(context, *args, **kwargs):
    world_name = Path(LaunchConfiguration("world").perform(context)).stem
    model_name = LaunchConfiguration("model_name").perform(context)

    link = "wamv/base_link"

    bridge_objs = [
        bridges.clock(),
        # bridges.pose(model_name),
        # bridges.pose_static(model_name),
        *payload_bridges.payload_bridges(
            world_name,
            model_name,
            link,
            "front_left_cam_sensor",
            sdf.Sensortype.CAMERA,
        ),
        *payload_bridges.payload_bridges(
            world_name,
            model_name,
            link,
            "front_right_cam_sensor",
            sdf.Sensortype.CAMERA,
        ),
        *payload_bridges.payload_bridges(
            world_name,
            model_name,
            link,
            "velodyne_sensor",
            sdf.Sensortype.GPU_LIDAR,
        ),
        *payload_bridges.payload_bridges(
            world_name,
            model_name,
            link,
            "imu_sensor",
            sdf.Sensortype.IMU,
        ),
    ]

    bridge_args = [obj.argument() for obj in bridge_objs]

    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            output="screen",
            arguments=bridge_args,
            # remappings=[
            #     ("/pose", "/tf"),
            #     ("/pose_static", "/tf_static")
            # ],
        ),
    ]


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory("navigator_bringup")
    pkg_project_gazebo = get_package_share_directory("navigator_gazebo")
    pkg_project_description = get_package_share_directory("navigator_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Navigator xacro model file argument
    xacro_file_arg = DeclareLaunchArgument(
        "xacro_file",
        default_value=os.path.join(
            pkg_project_description,
            "urdf",
            "navigator.urdf.xacro",
        ),
        description="Path to the robot xacro file",
    )
    world_arg = DeclareLaunchArgument("world", default_value="robotx_2024.world")
    model_name_arg = DeclareLaunchArgument("model_name", default_value="navigator")

    model_name = LaunchConfiguration("model_name")
    world = LaunchConfiguration("world")

    # Include model file path
    set_env = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH"),
            TextSubstitution(text=":"),
            get_package_share_path("wamv_description").parent,
            TextSubstitution(text=":"),
            get_package_share_path("wamv_gazebo").parent,
        ],
    )

    # !!! Uncomment once navigator_controller is created !!!
    # pkg_controller = get_package_share_directory("navigator_controller")

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py"),
        ),
        launch_arguments={
            "gz_args": [
                PathJoinSubstitution(
                    [pkg_project_gazebo, "worlds", world],
                ),
                " --render-engine",
                " ogre2",
            ],
        }.items(),
    )

    # !!! Uncomment once navigator_controller is created !!!
    # Get controller to use sim values
    # sim_pid_yaml = os.path.join(pkg_controller, "config", "sim_pid_controller.yaml")
    # set_sim_params = SetLaunchConfiguration("param_file", sim_pid_yaml)

    # Include the Subjugator_Setup Launch file
    navigator_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, "launch", "navigator_setup.launch.py"),
        ),
        launch_arguments={
            "use_sim_time": "true",
            "xacro_file": LaunchConfiguration("xacro_file"),
            "gui": "true",
        }.items(),
    )

    spawn_navigator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_project_gazebo, "launch", "spawn_navigator.launch.py"],
            ),
        ),
        launch_arguments={
            "xacro_file": LaunchConfiguration("xacro_file"),
            "model_name": model_name,
        }.items(),
    )

    return LaunchDescription(
        [
            xacro_file_arg,
            world_arg,
            model_name_arg,
            set_env,
            gz_sim,
            # !!! Uncomment once navigator_controller is created !!!
            # set_sim_params,
            navigator_setup,
            spawn_navigator,
            OpaqueFunction(function=make_bridge),
        ],
    )
