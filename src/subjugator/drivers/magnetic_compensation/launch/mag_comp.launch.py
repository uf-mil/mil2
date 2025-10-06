import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare the launch arguments
    # config file name
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(
            get_package_share_directory("magnetic_compensation"),
            "config",
            "test_config.yaml",
        ),
        description="Path to the config file",
    )

    # Get the package
    mag_comp_package = get_package_share_directory("magnetic_compensation")

    # Run a composable container with the "mil::magnetic_compensation::HardsoftCompensator" node
    container = ComposableNodeContainer(
        name="magnetic_compensation_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="magnetic_compensation",
                plugin="mil::magnetic_compensation::HardsoftCompensator",
                name="hardsoft_compensator",
                parameters=[
                    LaunchConfiguration("config_file"),
                    os.path.join(mag_comp_package, "config", "test_config.yaml"),
                ],
            ),
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(container)
    return ld
