import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField


class MagnetNode(Node):
    def __init__(self):
        super().__init__("magnet")
        self.create_subscription(MagneticField, "/imu/mag", self.cb, 10)

    def cb(self, msg):
        mag = [msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z]
        print(", ".join([f"{m:5f}" for m in mag]))


rclpy.init()
rclpy.spin(MagnetNode())
rclpy.shutdown()
