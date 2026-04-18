import ms5837
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class SimplePosePublisher(Node):
    def __init__(self):
        super().__init__(
            "new_depth_driver",
        )  # new meaning with blue robotics i2c driver

        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            "/depth/pose",
            10,
        )

        # We must initialize the sensor before reading it
        self.sensor = ms5837.MS5837_30BA(7)  # dumbass jetson uses i2c bus 7
        if not self.sensor.init():
            print("Sensor could not be initialized")
            exit(1)

        if not self.sensor.read():
            print("Sensor read failed!")
            exit(1)

        self.sensor.setFluidDensity(997)  # kg/m^3 mech's told me so

        self.timer = self.create_timer(1 / 20, self.publish_pose)

    def publish_pose(self):
        if not self.sensor.read():
            print("joeup bandsome")
            return

        le_depth_in_meters = self.sensor.depth() * -1

        msg = PoseWithCovarianceStamped()

        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        # Position
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = le_depth_in_meters

        # Orientation (all zeros is technically invalid quaternion,
        # but matches your "all zero" requirement)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 0.0

        # Covariance (36 zeros by default, but set explicitly)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[14] = 6.66179416e-06
        # this is the cov of the previous depth sensor and should be re-measured!

        self.publisher_.publish(msg)
        self.get_logger().info("Publishing PoseWithCovarianceStamped")


def main(args=None):
    rclpy.init(args=args)
    node = SimplePosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
