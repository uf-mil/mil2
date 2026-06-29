import subprocess

import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node

GPIO_PIN = 7
GPIO.setmode(GPIO.BOARD)
GPIO.setup(GPIO_PIN, GPIO.IN)


class Start_wand(Node):
    def __init__(self):
        super().__init__("start_wand")
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if not GPIO.input(GPIO_PIN):
            # imu_process = subprocess.Popen(["imu-socat"])
            # joe_process = subprocess.Popen(["python3", "joe_shmore.py"])
            _ = subprocess.Popen(
                ["ros2", "launch", "subjugator_bringup", "sub9.launch.py"],
            )


def main():
    rclpy.init()
    node = Start_wand()
    if not GPIO_PIN:
        print("sad")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
