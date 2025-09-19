import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class MonitoringNode(Node):
    def __init__(self):
        super().__init__("monitoring_node")
        self.data = ""
        self.pub_ = self.create_publisher(Odometry, "monitoring", 10)
        self.timer_ = self.create_timer(0.1, self.publish_text)
        self.create_subscription(Odometry, "dvl/odom", self.dvl_odom_callback, 10)

    def publish_text(self):
        msg = Odometry()
        self.pub_.publish(msg)

    def dvl_odom_callback(self, msg: Odometry):
        self.data = msg
        self.publish_text()


def main(args=None):
    rclpy.init(args=args)
    node = MonitoringNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
