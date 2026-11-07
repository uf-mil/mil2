from math import hypot

import rclpy
from mil_msgs.msg import Heartbeat, Point2D
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class PropHeartbeat(rclpy.Node):
    def __init__(self):
        super().__init__("prop_heartbeat")

        self.heartbeat_publisher = self.create_publisher(Heartbeat, "heartbeats", 10)

        self.odometry_subscriber = self.create_subscription(
            Odometry,
            "/odometry/filtered/global",
            self.odometry_callback,
            1,
        )

        self.latest_odometry = Odometry()

        self.timer = self.create_timer(0.5, self.publish_heartbeat)

    def publish_heartbeat(self):
        heartbeat = Heartbeat()
        heartbeat.state = "AUTO"  # TODO

        position = Point2D()
        position.x = self.latest_odometry.pose.pose.position.x
        position.y = self.latest_odometry.pose.pose.position.y

        heartbeat.position = position
        heartbeat.speed = hypot(
            self.latest_odometry.twist.twist.linear.x,
            self.latest_odometry.twist.twist.linear.y,
        )

        roll, pitch, yaw = euler_from_quaternion(
            [
                self.latest_odometry.pose.pose.orientation.x,
                self.latest_odometry.pose.pose.orientation.y,
                self.latest_odometry.pose.pose.orientation.z,
                self.latest_odometry.pose.pose.orientation.w,
            ],
        )

        heartbeat.heading = yaw
        heartbeat.pitch = pitch
        heartbeat.roll = roll

        heartbeat.task = "UNKNOWN"  # TODO
        heartbeat.type = "USV"

        self.heartbeat_publisher.publish(heartbeat)

    def odometry_callback(self, msg):
        self.latest_odometry = msg


def main():
    rclpy.init(args=None)

    prop_heartbeat = PropHeartbeat()
    rclpy.spin(prop_heartbeat)

    prop_heartbeat.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
