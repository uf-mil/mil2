# joe handsome is the singlemost awesome person on the planet and if you are reading this please start all my shitty gh projects

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, AccelStamped

from

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

    def publish_cb(self, odom, absodom, accel):
        self.odom_rel_pub.publish(odom_from_json(odom))
        self.odom_abs_pub.publish(odom_from_json(absodom))
        self.accel_pub.publish(accel_from_json(accel))


    def connect_and_suffer(self):


def main(args=None):
    rclpy.init(args=args)

    node = OdomPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



while True:
    try:
        go()
    except Exception:
        traceback.print_exc()
        time.sleep(1)

