import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


class MonitoringNode(Node):
    def __init__(self):
        super().__init__("monitoring_node")
        self.data = ""
        self.pub_dvl_ = self.create_publisher(Odometry, "monitoring_dvl", 10)
        self.pub_imu_ = self.create_publisher(Imu, "monitoring_imu", 10)

        self.create_subscription(Odometry, "dvl/odom", self.dvl_odom_callback, 10)
        self.create_subscription(Imu, "imu/data", self.imu_data_callback, 10)

    def dvl_odom_callback(self, dmsg: Odometry):
        # self.get_logger().info('I heard "%s"' % dmsg.twist.twist.linear.x)
        self.pub_dvl_.publish(dmsg)

    def imu_data_callback(self, imsg: Imu):
        # self.get_logger().info('I heard "%s"' % imsg.linear_acceleration.x)
        self.pub_imu_.publish(imsg)


def main(args=None):
    rclpy.init(args=args)
    node = MonitoringNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
