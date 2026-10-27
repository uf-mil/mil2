"""Launch the docking mission against an already running vehicle stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("timeout", default_value="240.0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("vehicle_id", default_value="prop"),
            DeclareLaunchArgument("ack_timeout", default_value="15.0"),
            DeclareLaunchArgument("sensor_timeout", default_value="1.0"),
            Node(
                package="prop_missions",
                executable="docking_mission",
                parameters=[
                    {
                        "vehicle_id": ParameterValue(
                            LaunchConfiguration("vehicle_id"), value_type=str
                        ),
                        "ack_timeout": ParameterValue(
                            LaunchConfiguration("ack_timeout"), value_type=float
                        ),
                        "sensor_timeout": ParameterValue(
                            LaunchConfiguration("sensor_timeout"), value_type=float
                        ),
                        "timeout": ParameterValue(
                            LaunchConfiguration("timeout"), value_type=float
                        ),
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                    }
                ],
                output="screen",
            ),
        ]
    )
