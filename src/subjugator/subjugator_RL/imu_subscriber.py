import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import threading
import time
import os
from rclpy.executors import SingleThreadedExecutor

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data_raw', #topic for sub9 cam
            self.imu_callback,
            10
        )
        self.imu_subscription 
        self.imu_data = None

    def imu_callback(self, msg):
        time.sleep(0.1) # Unblock thread so other things can run
        self.get_logger().info("Imu received")
        self.imu_data = msg

def safe_rclpy_init():
    try:
        rclpy.init()
    except RuntimeError:
        pass

safe_rclpy_init()
imu_node = ImuSubscriber()

# def run(args=None):
#     def spin():
#         # Declare node and spin it
#         rclpy.spin(imu_node)
#         imu_node.destroy_node()
#         rclpy.shutdown()
        
#     thread = threading.Thread(target=spin, daemon=True)
#     thread.start()
