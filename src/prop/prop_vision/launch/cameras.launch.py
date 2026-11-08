"""
The boat's two cameras, one looking ahead and one astern, each undistorted, run
through the detector and published in its own namespace:

    /front_camera/detections  /front_camera/camera_info  /front_camera/image_rect
    /back_camera/detections   /back_camera/camera_info   /back_camera/image_rect

    ros2 launch prop_vision cameras.launch.py \
        front_device:=/dev/v4l/by-path/<port>-video-index0 \
        back_device:=/dev/v4l/by-path/<port>-video-index0

They are the same model, so /dev/v4l/by-id cannot tell them apart and
/dev/videoN reorders between boots; by-path names the USB port, which is fixed
by how they are plugged in. A camera left without a device is not started,
rather than guessed at: the swap would put the front camera's buoys behind the
boat.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAMERAS = ("front_camera", "back_camera")


def cameras(context):
    model = LaunchConfiguration("model").perform(context)
    publish_image = LaunchConfiguration("publish_image").perform(context) == "true"

    nodes = []
    for name in CAMERAS:
        device = LaunchConfiguration(f"{name.split('_')[0]}_device").perform(context)
        if not device:
            continue
        parameters = {
            "device": device,
            "frame_id": f"{name}_optical_frame",
            "publish_image": publish_image,
        }
        if model:
            parameters["model"] = model
        nodes.append(
            Node(
                package="prop_vision",
                executable="vision_node.py",
                name="vision",
                namespace=name,
                parameters=[parameters],
                # A camera that drops off the bus comes back when it is reopened.
                respawn=True,
                respawn_delay=2.0,
                # Camera reports its resolution with print.
                additional_env={"PYTHONUNBUFFERED": "1"},
                output="screen",
            ),
        )

    if not nodes:
        raise RuntimeError(
            "no cameras to start: give front_device, back_device or both",
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "front_device",
                default_value="",
                description="The camera looking ahead: /dev/v4l/by-path/..., "
                "or an index. Empty leaves it off.",
            ),
            DeclareLaunchArgument(
                "back_device",
                default_value="",
                description="The camera looking astern. Empty leaves it off.",
            ),
            DeclareLaunchArgument(
                "model",
                default_value="",
                description="YOLO weights for both. Empty is models/best.pt "
                "beside vision_node.py.",
            ),
            DeclareLaunchArgument(
                "publish_image",
                default_value="false",
                description="Also publish image_rect, for RViz or "
                "prop_fusion's debug_image. Two 1080p streams are a lot to "
                "put on the network for nothing.",
            ),
            OpaqueFunction(function=cameras),
        ],
    )
