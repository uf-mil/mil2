import math

import ms5837
import rclpy
import transforms3d
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

POOL_DEPTH = 1.68
COMPENSATE_DIST = 0.3
ZERO_DEPTH = 0.4


class CompensatedDepthDriver(Node):
    def __init__(self):
        super().__init__("dvl_depth_driver")

        self.dvl_sub_ = self.create_subscription(Odometry, "/dvl/odom", self.dvl_cb, 10)
        self.compensation = 0
        self.zero = 0

        self.imu_sub_ = self.create_subscription(Imu, "/imu/data", self.imu_cb, 10)
        self.orientation = None

        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            "/depth/dvl",
            10,
        )

        self.raw_pub_ = self.create_publisher(
            PoseWithCovarianceStamped,
            "/depth/pose",
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

        self.timer_ = self.create_timer(1 / 20, self.timer_cb)

    def pose(self, depth):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.z = depth
        msg.pose.covariance[14] = 0.1
        return msg

    def imu_cb(self, msg):
        self.orientation = msg.orientation

    def dvl_cb(self, msg):
        if not self.sensor.read():
            self.get_logger().info("Sensor read failed!")
            return

        zero_depth = -self.sensor.depth() - ZERO_DEPTH
        dvl_depth_hyp = msg.pose.pose.position.z - POOL_DEPTH
        depth = zero_depth + self.compensation

        if self.orientation:
            yaw, pitch, roll = transforms3d.taitbryan.quat2euler(
                [
                    self.orientation.w,
                    self.orientation.x,
                    self.orientation.y,
                    self.orientation.z,
                ],
            )
            _, angle = transforms3d.taitbryan.euler2axangle(0, pitch, roll)

            if angle <= math.pi / 6:
                dvl_depth = dvl_depth_hyp / math.cos(angle)

                if math.fabs(dvl_depth - depth) <= COMPENSATE_DIST:
                    new_depth = dvl_depth * 0.2 + depth * 0.8

                    if math.fabs(new_depth) <= 0.5:
                        self.compensation = depth - zero_depth
                        self.get_logger().info(
                            f"compensation {self.compensation:.2f} "
                            f"depth {depth:.2f} -> {new_depth:.2f}",
                        )
                    else:
                        self.get_logger().info("Compensation capped")
                else:
                    self.get_logger().info("Difference too large")
            else:
                self.get_logger().info("Angle too steep")
        else:
            self.get_logger().info("No imu orientation")

        self.publisher_.publish(self.pose(depth))

    def timer_cb(self):
        if not self.sensor.read():
            self.get_logger().info("Sensor read failed!")
            return

        depth = -self.sensor.depth()
        self.publisher_.publish(self.pose(depth - ZERO_DEPTH + self.compensation))
        self.raw_pub_.publish(self.pose(depth))


rclpy.init()
node = CompensatedDepthDriver()
rclpy.spin(node)
rclpy.shutdown()
