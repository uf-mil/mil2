import os

from ament_index_python.packages import (
    get_package_share_directory,
    get_package_share_path,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
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
    gz_sim_world = DeclareLaunchArgument("world", default_value="robotx_2024.world")
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
                " ogre2",
            ],
        }.items(),
    )

    # !!! Uncomment once navigator_controller is created !!!
    # Get controller to use sim values
    # sim_pid_yaml = os.path.join(pkg_controller, "config", "sim_pid_controller.yaml")
    # set_sim_params = SetLaunchConfiguration("param_file", sim_pid_yaml)

    # Write an on-disk URDF
    urdf_out = os.path.join(pkg_project_description, "urdf", "navigator.urdf")
    generate_urdf = ExecuteProcess(
        cmd=["xacro", LaunchConfiguration("xacro_file"), "-o", urdf_out],
        output="screen",
    )

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

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_project_bringup,
                    "config",
                    "navigator_bridge.yaml",
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            },
        ],
        output="screen",
    )

    # Spawn the navigator
    navigator_x = "0.0"
    navigator_y = "0.0"
    navigator_z = "0.0"
    navigator_yaw = "0.0"
    spawn_navigator = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "navigator",
            "-file",
            urdf_out,
            "-x",
            navigator_x,
            "-y",
            navigator_y,
            "-z",
            navigator_z,
            "-Y",
            navigator_yaw,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            xacro_file_arg,
            set_env,
            gz_sim_world,
            gz_sim,
            # !!! Uncomment once navigator_controller is created !!!
            # set_sim_params,
            navigator_setup,
            bridge,
            generate_urdf,
            spawn_navigator,
        ],
    )
