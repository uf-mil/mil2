import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def generate_launch_description():
    dvl_launch = IncludeLaunchDescription(
        pkg_share("waterlinked_dvl_driver", "launch", "dvl.launch.py"),
    )

    front_cam_launch = Node(
        package="front_cam",
        executable="front_cam",
        name="front_cam",
        parameters=[
            {
                "camera-id": "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.3:1.0-video-index0",
                "camera-topic": "front_cam/image_raw",
            },
        ],
        output="screen",
    )

    down_cam_launch = Node(
        package="front_cam",
        executable="front_cam",
        name="down_cam",
        exec_name="down_cam",
        parameters=[
            {
                "camera-id": "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0",
                "camera-topic": "down_camera/image_raw",
            },
        ],
        output="screen",
    )

    # IMU
    # vectornav_launch = IncludeLaunchDescription(
    #     pkg_share("vectornav", "launch", "vectornav.launch.py"),
    # )
    # mag_comp_launch = IncludeLaunchDescription(
    #     pkg_share("magnetic_compensation", "launch", "mag_comp.launch.py"),
    #     launch_arguments={
    #         "config_file": pkg_share(
    #             "subjugator_bringup",
    #             "config",
    #             "sensors",
    #             "hardsoft.yaml",
    #         ),
    #     }.items(),
    # )

    thrust_launch = IncludeLaunchDescription(
        pkg_share("thrust_and_kill_board", "launch", "thrust_and_kill_board.launch.py"),
    )

    new_depth = Node(
        package="new_depth_driver",
        executable="new_depth_driver",
        name="new_depth_driver",
        output="both",
    )

    servo_driver = Node(
        package="servo_controller",
        executable="servo_driver",
        output="both",
    )

    yolo_front = IncludeLaunchDescription(
        pkg_share("yolo_bringup", "launch", "yolov26.launch.py"),
    )

    yolo_down = IncludeLaunchDescription(
        pkg_share("yolo_bringup", "launch", "yolov26.launch.py"),
        launch_arguments={
            "input_image_topic": "/front_cam/image_raw",
            "namespace": "yolo_down",
            "model": "/home/sub9/mil2/src/subjugator/gnc/subjugator_vision/models/bins.pt"
        }
    )

    # Launch subjugator_setup.launch.py
    subjugator_setup = IncludeLaunchDescription(
        pkg_share("subjugator_bringup", "launch", "common.launch.py"),
        launch_arguments={
            "xacro_file": pkg_share(
                "subjugator_description",
                "urdf",
                "sub9.urdf.xacro",
            ),
            "gui": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            yolo,
            yolo_down,
            new_depth,
            dvl_launch,
            front_cam_launch,
            down_cam_launch,
            servo_driver,
            # vectornav_launch,
            # mag_comp_launch,
            thrust_launch,
            subjugator_setup,
        ],
    )
