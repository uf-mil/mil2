#!/usr/bin/env python3
"""
map -> odom -> base_link from Gazebo's ground truth.

    ros2 run prop_gazebo sim_truth_tf.py

For running the stack on the truth instead of the estimate: odom is made
identical to map, and base_link is where Gazebo says the hull is. The EKFs have
to be launched with publish_tf:=false alongside, or base_link gets two parents.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class TruthTf(Node):
    def __init__(self):
        super().__init__("sim_truth_tf")
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, "sim/ground_truth", self.on_truth, 10)

    def on_truth(self, msg: Odometry) -> None:
        # Separate headers: assigning msg.header to both would share one object.
        odom = TransformStamped()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "odom"
        odom.transform.rotation.w = 1.0

        base = TransformStamped()
        base.header.stamp = msg.header.stamp
        base.header.frame_id = "odom"
        base.child_frame_id = "base_link"
        p = msg.pose.pose.position
        base.transform.translation.x = p.x
        base.transform.translation.y = p.y
        base.transform.translation.z = p.z
        base.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform([odom, base])


def main():
    rclpy.init()
    node = TruthTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
