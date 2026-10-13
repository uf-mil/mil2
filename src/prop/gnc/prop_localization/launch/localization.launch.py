"""
Localization stack: robot_state_publisher + rf2o + dual EKF + navsat_transform.

  rf2o       : /scan              -> /lidar/odom (via rf2o_cov_mapper)
  ekf_local  : IMU + lidar odom   -> odometry/filtered/local  + TF odom -> base_link
  ekf_global : local + GPS odom   -> odometry/filtered/global + TF map  -> odom
  navsat     : GPS + IMU + global odom <-> odometry/gps

Both EKFs come up disabled, so the origin is wherever the boat sits when they
are enabled rather than wherever it was at launch:

    ros2 launch prop_localization localization.launch.py
    prop-start-localization
    ros2 topic echo /odometry/filtered/global
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def generate_launch_description():
    config = pkg_share("prop_localization", "config", "killme.yaml")

    with open(pkg_share("prop_localization", "urdf", "prop.urdf")) as urdf:
        robot_description = urdf.read()

    # robot_localization advertises enable, reset, set_pose and toggle at the
    # root, so two EKFs collide. Push each under its own node name.
    def ekf(kind):
        return Node(
            package="robot_localization",
            executable="ekf_node",
            name=f"ekf_{kind}",
            parameters=[config],
            remappings=[("odometry/filtered", f"/odometry/filtered/{kind}")]
            + [(srv, f"~/{srv}") for srv in ("enable", "reset", "set_pose", "toggle")],
            output="screen",
        )

    return LaunchDescription(
        [
            # base_link -> imu and base_link -> lidar_link, out of prop.urdf.
            # The EKFs need the first one to rotate the IMU out of its 180
            # degree mounting.
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            # publish_tf stays false: odom -> base_link belongs to ekf_local.
            Node(
                package="rf2o_laser_odometry",
                executable="rf2o_laser_odometry_node",
                name="rf2o_laser_odometry",
                parameters=[
                    {
                        "publish_tf": False,
                        "odom_frame_id": "odom_rf2o",
                        "init_pose_from_topic": "",
                        "freq": 20.0,
                    },
                ],
                output="screen",
            ),
            Node(
                package="rf2o_cov_mapper",
                executable="rf2o_cov_mapper",
                output="screen",
            ),
            ekf("local"),
            ekf("global"),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                parameters=[config],
                remappings=[
                    ("imu", "/imu/data"),
                    ("odometry/filtered", "/odometry/filtered/global"),
                    ("odometry/gps", "/odometry/gps"),
                ],
                output="screen",
            ),
        ],
    )
