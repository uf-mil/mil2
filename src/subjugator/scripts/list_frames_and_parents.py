#!/usr/bin/env python3

# first ur gonna cd into this directory,
# then ./list_frames_and_parents.py
# then read output
# I am Joseph Handsome


import rclpy
import tf2_ros
from rclpy.node import Node


class FrameChecker(Node):
    def __init__(self):
        super().__init__("frame_checker")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.check_frames)

    def check_frames(self):
        frames = self.tf_buffer.all_frames_as_string()
        self.get_logger().info(f"All frames: {frames}")

        search_frame = "depth_sensor_frame"
        if search_frame in frames:
            self.get_logger().info(f"Found {search_frame} in the transform tree")

            # Get all frames
            frame_list = frames.split("\nFrame ")
            frame_list = [f.split(" exists")[0] for f in frame_list if f]

            # Check transforms to and from our target frame
            for frame in frame_list:
                if frame != search_frame:
                    try:
                        """
                        trans = self.tf_buffer.lookup_transform(
                            search_frame,
                            frame,
                            rclpy.time.Time(),
                        )
                        """
                        self.get_logger().info(
                            f"Transform from {search_frame} to {frame} exists",
                        )
                    except (
                        tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException,
                    ):
                        pass

                    try:
                        """
                        trans = self.tf_buffer.lookup_transform(
                            frame,
                            search_frame,
                            rclpy.time.Time(),
                        )
                        """
                        self.get_logger().info(
                            f"Transform from {frame} to {search_frame} exists",
                        )
                    except (
                        tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException,
                    ):
                        pass
        else:
            self.get_logger().warn(f"Frame {search_frame} not found in transform tree")


def main():
    rclpy.init()
    node = FrameChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        return

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
