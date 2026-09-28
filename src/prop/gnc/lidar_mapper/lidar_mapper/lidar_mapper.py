#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudToLaserScan(Node):
    def __init__(self):
        super().__init__("pointcloud_to_laserscan")

        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.radians(1.0))
        self.declare_parameter("range_min", 0.1)
        self.declare_parameter("range_max", 30.0)

        self.angle_min = self.get_parameter("angle_min").value
        self.angle_max = self.get_parameter("angle_max").value
        self.angle_increment = self.get_parameter("angle_increment").value
        self.range_min = self.get_parameter("range_min").value
        self.range_max = self.get_parameter("range_max").value

        self.num_ranges = math.ceil(
            (self.angle_max - self.angle_min) / self.angle_increment,
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            "velodyne_points",
            self.pointcloud_callback,
            10,
        )

        self.publisher = self.create_publisher(LaserScan, "scan", 10)

        self.get_logger().info("PointCloud2 -> LaserScan node started")

    def pointcloud_callback(self, cloud_msg: PointCloud2):
        scan = LaserScan()

        scan.header = cloud_msg.header
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment

        scan.time_increment = 0.0
        scan.scan_time = 0.0

        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Start every bin as infinity (no return)
        ranges = [float("inf")] * self.num_ranges

        # Read XYZ points from PointCloud2
        points = point_cloud2.read_points(
            cloud_msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )

        for point in points:
            x, y, z = point

            # Ignore points outside the usable range
            distance = math.sqrt(x * x + y * y)

            if distance < self.range_min or distance > self.range_max:
                continue

            # Angle around the sensor
            angle = math.atan2(y, x)

            if angle < self.angle_min or angle > self.angle_max:
                continue

            # Convert angle to LaserScan array index
            index = int((angle - self.angle_min) / self.angle_increment)

            if (0 <= index < self.num_ranges) and (distance < ranges[index]):
                # Keep the closest obstacle in this angular bin
                ranges[index] = distance

        scan.ranges = ranges

        print("I pubbed")
        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)

    node = PointCloudToLaserScan()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
