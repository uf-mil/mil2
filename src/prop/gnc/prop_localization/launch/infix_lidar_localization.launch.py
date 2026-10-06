import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def pkg_share(package: str, *parts: str) -> str:
    return os.path.join(get_package_share_directory(package), *parts)


def generate_launch_description():
    config = pkg_share("prop_localization", "config", "ekf.yaml")

    with open(pkg_share("prop_localization", "urdf", "prop.urdf")) as urdf:
        robot_description = urdf.read()

    return LaunchDescription(
        [
            # base_link -> imu and base_link -> lidar_link, out of prop.urdf.
            # rf2o needs the second one to express its estimate in base_link
            # rather than lidar_link.
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            # Lidar odometry. publish_tf must stay false -- rf2o defaults to
            # broadcasting odom -> base_link itself, which would collide with
            # ekf_local.
            Node(
                package="rf2o_laser_odometry",
                executable="rf2o_laser_odometry_node",
                name="rf2o_laser_odometry",
                parameters=[
                    {
                        "laser_scan_topic": "/scan",
                        "odom_topic": "/odom_rf2o",
                        "publish_tf": False,
                        "base_frame_id": "base_link",
                        "odom_frame_id": "odom_rf2o",
                        "init_pose_from_topic": "",
                        "freq": 20.0,
                    },
                ],
                output="screen",
            ),
            # rf2o publishes an all-zero covariance, which robot_localization
            # reads as a near perfect measurement. This stamps real values on
            # the way past and republishes as /lidar/odom.
            Node(
                package="prop_localization",
                executable="rf2o_covar.py",
                name="rf2o_covariance_relay",
                parameters=[
                    {
                        "input_topic": "/odom_rf2o",
                        "output_topic": "/lidar/odom",
                    },
                ],
                output="screen",
            ),
            # odom -> base_link, fusing /infix2/odom/rel and /lidar/odom.
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_local",
                parameters=[config],
                remappings=[("odometry/filtered", "/odometry/filtered/local")],
                output="screen",
            ),
        ],
    )
