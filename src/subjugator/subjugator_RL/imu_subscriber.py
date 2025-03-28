# Subscriber for imu
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import threading
import time
import os

class imu_subscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
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
        time.sleep(0.1) # Delay for imu callback
        self.get_logger().info("Imu received")

        with open("imu_pipe", 'wb') as pipe:
            print("Writing to imu pipe")
            pipe.write(msg.tobytes())
        
def main(args=None):
    rclpy.init(args=args)

    # Declare node and spin it
    node = imu_subscriber()
    rclpy.spin(node)      

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()