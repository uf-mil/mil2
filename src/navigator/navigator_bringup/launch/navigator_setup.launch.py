import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Setup project paths
    pkg_project_bringup = get_package_share_directory("navigator_bringup")
    pkg_project_description = get_package_share_directory("navigator_description")

    # !!! Uncomment once navigator_controller is created !!!
    # pkg_thruster_manager = get_package_share_directory("navigator_thruster_manager")
    # pkg_localization = get_package_share_directory("navigator_localization")
    # pkg_controller = get_package_share_directory("navigator_controller")

    # Args
    gui_cmd = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="whether to launch the gui",
    )

    xacro_file_arg = DeclareLaunchArgument(
        "xacro_file",
        default_value=PathJoinSubstitution(
            [pkg_project_description, "urdf", "navigator.urdf.xacro"],
        ),
        description="Path to the robot xacro file",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use sim clock (true for sim, false for real)",
    )

    # Expand xacro at runtime
    robot_desc = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", LaunchConfiguration("xacro_file")]),
        value_type=str,
    )

    # # Convert URDF to SDF using Gazebo's gz tool
    # sdf_file = os.path.join(pkg_project_description, 'urdf', 'navigator.sdf')
    # try:
    #     subprocess.run(['gz', 'sdf', '-p', urdf_file], check=True, stdout=open(sdf_file, 'w'))
    #     print(f"Successfully converted {urdf_file} to {sdf_file}")
    # except subprocess.CalledProcessError as e:
    #     print(f"Error converting URDF to SDF: {e}")

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"robot_description": robot_desc},
        ],
    )

    # Visualize in RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_project_bringup, "config", "navigator.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    # !!! Uncomment once navigator_controller is created !!!
    # thruster_manager = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_thruster_manager, "launch", "thruster_manager.launch.py"),
    #     ),
    # )
    # !!! Uncomment once navigator_localization is created !!!
    # localization = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             pkg_localization,
    #             "launch",
    #             "navigator_localization.launch.py",
    #         ),
    #     ),
    #     launch_arguments={
    #         "params_file": os.path.join(
    #             pkg_localization,
    #             "config",
    #             "localization_parameters.yaml",
    #         ),
    #     }.items(),
    # )

    # depth_to_pose = Node(
    # package="navigator_localization",
    # executable="depth_to_pose_node.py",
    # name="depth_to_pose",
    # output="screen",
    # )

    # !!! Uncomment once navigator_localization is created !!!
    # controller = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_controller, "launch", "pid_controller.launch.py"),
    #     ),
    # )

    # !!! Uncomment once navigator_path_planner is created !!!
    # path_planner = Node(
    #     package="navigator_path_planner",
    #     executable="navigator_path_planner",
    #     name="navigator_path_planner",
    #     output="both",
    # )

    # !!! Uncomment once navigator_trajectory_planner is created !!!
    # trajectory_planner = Node(
    #     package="navigator_trajectory_planner",
    #     executable="trajectory_planner",
    #     name="navigator_trajectory_planner",
    #     output="both",
    # )

    # wrench_tuner = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource(
    # os.path.join(
    # get_package_share_directory("navigator_wrench_tuner"),
    # "launch",
    # "wrench_tuner_launch.py",
    # ),
    # ),
    # )
    return LaunchDescription(
        [
            gui_cmd,
            xacro_file_arg,
            use_sim_time_arg,
            robot_state_publisher_node,
            # joint_state_publisher_node,
            rviz,
            # !!! Uncomment once navigator_localization is created !!!
            # thruster_manager,
            # localization,
            # controller,
            # !!! Uncomment once navigator_path_planner is created !!!
            # path_planner,
            # trajectory_planner,
        ],
    )
