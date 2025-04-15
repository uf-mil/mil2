import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class OdometrySubscriber(Node):

    def __init__(self):
        super().__init__('odometry_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )

        roll, pitch, yaw = euler_from_quaternion(quaternion)

        self.get_logger().info(f'Orientation in Euler angles -> Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}')


def main(args=None):
    rclpy.init(args=args)
    odometry_subscriber = OdometrySubscriber()
    rclpy.spin(odometry_subscriber)

    odometry_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
