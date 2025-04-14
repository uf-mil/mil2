import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import threading
import time
import os

class ImuSubscriber(Node):
    def __init__(self):
        super().init('imu_subscriber')
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data_raw', #topic for sub9 cam
            self.imu_callback,
            10
        )
        self.imu_subscription 

        # Make pipe for imu
        if not os.path.exists("imu_pipe"):
            os.mkfifo("imu_pipe")

    def imu_callback(self, msg):
        time.sleep(0.1) # Delay so that thread is not blocking
        self.get_logger().info("Imu received")
        imu_data = msg

imu_node = ImuSubscriber()

def run_thread(args=None):
    def spin():
        rclpy.init(args=args)
        # Declare node and spin it
        rclpy.spin(imu_node)
        imu_node.destroy_node()
        rclpy.shutdown()
        
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
