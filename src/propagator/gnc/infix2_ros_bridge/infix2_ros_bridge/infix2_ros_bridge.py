# joe handsome is the singlemost awesome person on the planet and if you are reading this please start all my shitty gh projects

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelStamped

import traceback

from .msg_from_json import accel_from_json, odom_from_json
from .no_ros_driver import go

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        # Create publisher
        self.odom_rel_pub = self.create_publisher(
            Odometry,
            '/infix2/odom/rel',
            10
        )

        self.odom_abs_pub = self.create_publisher(
            Odometry,
            '/infix2/odom/abs',
            10
        )

        self.accel_pub = self.create_publisher(
            Odometry,
            '/infix2/accel',
            10
        )

        self.connect_and_suffer()

    def publish_cb(self, odom, absodom, accel):
        self.odom_rel_pub.publish(odom_from_json(odom))
        self.odom_abs_pub.publish(odom_from_json(absodom))
        self.accel_pub.publish(accel_from_json(accel))

    def connect_and_suffer(self):
        while True:
            try:
                go(self.publish_cb)
            except Exception:
                traceback.print_exc()
                time.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    node = OdomPublisher()

    try:
        print("starting!")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




