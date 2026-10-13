from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # infix_sensor_bridge
            Node(
                package="infix_sensor_bridge",
                executable="infix_sensor_bridge",
                name="infix_sensor_bridge",
                output="screen",
            ),
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter_madgwick",
                output="screen",
                parameters=[
                    {
                        "publish_tf": False,
                        "mag_bias_x": 2.1277e-5,
                        "mag_bias_y": 1.8647e-5,
                        "mag_bias_z": -3.4394e-5,
                    },
                ],
            ),
            # SDGPS pipeline
            ExecuteProcess(
                cmd=[
                    "sdgps",
                    "sylphase-usbgpsimu2-raw",
                    "!",
                    "remap-streams",
                    "--imu",
                    "--mag",
                    "all",
                    "!",
                    "export-json-tcp",
                    "--port",
                    "1234",
                ],
                output="screen",
            ),
        ],
    )
