import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Args
    gui_cmd = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="whether to launch the gui",
    )

    # Setup project paths
    pkg_project_bringup = get_package_share_directory("subjugator_bringup")
    pkg_project_description = get_package_share_directory("subjugator_description")
    pkg_thruster_manager = get_package_share_directory("subjugator_thruster_manager")
    pkg_localization = get_package_share_directory("subjugator_localization")
    pkg_controller = get_package_share_directory("subjugator_controller")
    pkg_mission_planner = get_package_share_directory("subjugator_mission_planner")

    # Load the URDF file from "description" package
    xacro_file = os.path.join(pkg_project_description, "urdf", "sub9.urdf.xacro")
    assert os.path.exists(xacro_file), "The sub9.urdf.xacro doesn't exist in " + str(
        xacro_file,
    )

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    urdf_file = os.path.join(pkg_project_description, "urdf", "sub9.urdf")
    with open(urdf_file, "w") as urdf:
        urdf.write(robot_desc)

    print(f"Successfully converted {xacro_file} to {urdf_file}")

    # # Convert URDF to SDF using Gazebo's gz tool
    # sdf_file = os.path.join(pkg_project_description, 'urdf', 'sub9.sdf')
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
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
    )
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': True},
    #         {'robot_description': robot_desc},
    #     ]
    # )

    # Visualize in RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_project_bringup, "config", "subjugator.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    thruster_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_thruster_manager, "launch", "thruster_manager.launch.py"),
        ),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_localization,
                "launch",
                "subjugator_localization.launch.py",
            ),
        ),
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_controller, "launch", "pid_controller.launch.py"),
        ),
    )

    path_planner = Node(
        package="subjugator_path_planner",
        executable="subjugator_path_planner",
        name="subjugator_path_planner",
        output="both",
    )

    trajectory_planner = Node(
        package="subjugator_trajectory_planner",
        executable="trajectory_planner",
        name="subjugator_trajectory_planner",
        output="both",
    )

    mission_servers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mission_planner, "launch", "task_server_launch.py"),
        ),
    )

    mission_planner = Node(
        package="subjugator_mission_planner",
        executable="mission_planner",
        name="subjugator_mission_planner",
        # output="both", # I am NOT getting spammed
    )

    return LaunchDescription(
        [
            gui_cmd,
            robot_state_publisher_node,
            # joint_state_publisher_node,
            rviz,
            thruster_manager,
            localization,
            controller,
            path_planner,
            trajectory_planner,
            mission_servers,
            mission_planner,
        ],
    )
