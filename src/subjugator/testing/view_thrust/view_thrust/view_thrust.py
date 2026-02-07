import rclpy
from rclpy.node import Node


class ViewThrust(Node):
    def __init__(self):
        super().__init__("view_thrust")
        # self.publisher_ = self.create_publisher(Odometry, '/dvl/odom', 10)
        # timer_period = 1/40  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)


def main(args=None):
    rclpy.init(args=args)
    node = ViewThrust()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
