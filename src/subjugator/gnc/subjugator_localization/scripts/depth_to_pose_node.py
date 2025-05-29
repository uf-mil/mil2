#!/usr/bin/env python3
"""
Convert /depth (mil_msgs/DepthStamped) to /depth/pose
(geometry_msgs/PoseWithCovarianceStamped) so the EKF
can fuse an absolute Z measurement in simulation.
"""
import rclpy
from rclpy.node import Node
from mil_msgs.msg import DepthStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


class DepthToPose(Node):
    def __init__(self):
        super().__init__("depth_to_pose")
        self.sub = self.create_subscription(
            DepthStamped, "/depth", self.cb, 10
        )
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, "/depth/pose", 10
        )

    def cb(self, msg: DepthStamped):
        p = PoseWithCovarianceStamped()
        p.header.stamp = msg.header.stamp
        p.header.frame_id = "base_link"  # <- the key change

        # Gazebo water depth: positive downward ⇒ world Z is -depth
        p.pose.pose.position.z = -msg.depth
        p.pose.pose.orientation.w = 1.0  
        p.pose.covariance[0] = 1e3
        p.pose.covariance[7] = 1e3
        p.pose.covariance[14] = 0.01
        p.pose.covariance[21] = 1e3
        p.pose.covariance[28] = 1e3
        p.pose.covariance[35] = 1e3    # variance on Z (0.1 m²)
        self.pub.publish(p)


def main():
    rclpy.init()
    rclpy.spin(DepthToPose())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
