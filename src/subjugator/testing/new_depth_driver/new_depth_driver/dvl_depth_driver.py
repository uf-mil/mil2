import math

import ms5837
import rclpy
import transforms3d
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

POOL_DEPTH = 1.65
COMPENSATE_DIST = 0.2


class CompensatedDepthDriver(Node):
    def __init__(self):
        super().__init__("dvl_depth_driver")

        self.dvl_sub_ = self.create_subscriber(Odometry, "/dvl/odom", 10, self.depth_cb)
        self.compensation = 0

        self.imu_sub_ = self.create_subscriber(Imu, "/imu/data", 10, self.imu_cb)
        self.orientation = None

        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            "/depth/dvl",
            10,
        )

        # sensor
        self.sensor = ms5837.MS5837_30BA(7)  # jetson uses i2c bus 7
        if not self.sensor.init():
            self.get_logger().info("Sensor could not be initialized")
            exit(1)

        if not self.sensor.read():
            self.get_logger().info("Initial sensor read failed!")
            exit(1)

        self.sensor.setFluidDensity(997)  # kg/m^3

    def imu_cb(self, msg):
        self.orientation = msg.orientation

    def odom_cb(self, msg):
        if not self.sensor.read():
            self.get_logger().info("Sensor read failed!")
            return

        raw_depth = self.sensor.depth()
        dvl_depth_hyp = msg.pose.pose.position.z - POOL_DEPTH
        depth = -raw_depth + self.compensation

        if self.orientation:
            _, angle = transforms3d.quat2axangle(
                [
                    self.orientation.w,
                    self.orientation.x,
                    self.orientation.y,
                    self.orientation.z,
                ],
            )

            if self.angle <= math.pi / 6:
                dvl_depth = dvl_depth_hyp / math.cos(angle)

                if math.abs(dvl_depth - depth) <= COMPENSATE_DIST:
                    old_depth = depth

                    depth = dvl_depth * 0.2 + depth * 0.8
                    self.compensation = depth + raw_depth
                    self.get_logger().info(
                        f"compensation {self.compensation:.2f} "
                        f"depth {old_depth:.2f} -> {depth:.2f}",
                    )
            else:
                self.get_logger().info("Angle too steep")
        else:
            self.get_logger().info("No imu orientation")

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.z = depth
        msg.pose.covariance[14] = 0.1
        self.publisher_.publish(msg)


rclpy.init()
node = CompensatedDepthDriver()
rclpy.spin(node)
rclpy.shutdown()
