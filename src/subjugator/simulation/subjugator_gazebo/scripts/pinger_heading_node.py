#!/usr/bin/env python3

# takes the direction vector coming from /ping (which is actually world frame)
# converts it into body frame using the sub orientation
# then republishes it on hydrophones/solved so the mission planner is happy


import numpy as np
import rclpy
from mil_msgs.msg import ProcessedPing
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R


class PingerHeadingNode(Node):

    # subscribes to:
    # /ping -> direction from gazebo (world frame)
    # /odometry/filtered -> current pose of the sub

    # publishes:
    # /hydrophones/solved -> same direction but in body frame

    def __init__(self):
        # node name shows up in ros2 node list
        super().__init__("pinger_heading_node")

        # latest quaternion from odom
        # starts empty until we get something
        self.sub_orientation = None

        # odom subscription
        # we only care about orientation for rotation
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.odom_cb,
            10,
        )

        # ping from gazebo hydrophone plugin
        self.ping_sub = self.create_subscription(
            ProcessedPing,
            "/ping",
            self.ping_cb,
            10,
        )

        # mission planner listens here
        self.ping_pub = self.create_publisher(
            ProcessedPing,
            "/hydrophones/solved",
            10,
        )

        # just so we know its running
        self.get_logger().info("pinger heading node started")

    def odom_cb(self, msg: Odometry):

        # called whenever we get odometry
        # store quaternion as [x, y, z, w] since that's what scipy wants

        q = msg.pose.pose.orientation
        self.sub_orientation = [q.x, q.y, q.z, q.w]

    def ping_cb(self, msg: ProcessedPing):

        # called whenever a ping comes in
        # convert world vector -> body vector

        # if no odom yet then we can't rotate anything
        if self.sub_orientation is None:
            self.get_logger().warn(
                "no odometry yet",
                throttle_duration_sec=2.0,
            )
            return

        # this field name is misleading
        # gazebo fills it with world frame data even though it says body
        d_world = np.array(
            [
                msg.origin_direction_body.x,
                msg.origin_direction_body.y,
                msg.origin_direction_body.z,
            ],
        )

        # build rotation from quaternion
        # to go world -> body we use inverse rotation
        rot = R.from_quat(self.sub_orientation)
        d_body = rot.inv().apply(d_world)

        # build outgoing message
        out = ProcessedPing()
        out.origin_direction_body.x = float(d_body[0])
        out.origin_direction_body.y = float(d_body[1])
        out.origin_direction_body.z = float(d_body[2])

        # just pass these through
        out.frequency = msg.frequency
        out.origin_distance_m = msg.origin_distance_m

        # publish result
        self.ping_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)

    # make node
    node = PingerHeadingNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
