#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray


class SingleDetectionAreaSum(Node):
    def __init__(self):
        super().__init__("single_detection_area_sum")

        self.subscription = self.create_subscription(
            DetectionArray,
            "/yolo/detections",
            self.callback,
            10,
        )

        self.get_logger().info("Waiting for one /yolo/detections message...")

    def callback(self, msg: DetectionArray) -> None:
        largest_red_area = None
        largest_white_area = None

        for det in msg.detections:
            width = max(0.0, float(det.bbox.size.x))
            height = max(0.0, float(det.bbox.size.y))
            area = width * height

            if det.class_name == "red-pole" and (
                largest_red_area is None or area > largest_red_area
            ):
                largest_red_area = area
            elif det.class_name == "white-pole" and (
                largest_white_area is None or area > largest_white_area
            ):
                largest_white_area = area

        if largest_red_area is None:
            self.get_logger().info("No red-pole found in this message.")
        else:
            self.get_logger().info(f"Largest red-pole area: {largest_red_area:.2f}")

        if largest_white_area is None:
            self.get_logger().info("No white-pole found in this message.")
        else:
            self.get_logger().info(f"Largest white-pole area: {largest_white_area:.2f}")

        if largest_red_area is not None and largest_white_area is not None:
            total = largest_red_area + largest_white_area
            self.get_logger().info(
                f"Sum of largest red-pole and white-pole areas: {total:.2f}",
            )
        else:
            self.get_logger().info(
                "Could not compute sum because one or both classes were missing.",
            )

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SingleDetectionAreaSum()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
