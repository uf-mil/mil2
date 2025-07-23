import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

INPUT_PIN = 18


class SubjugatorReadSwitchNode(Node):
    def __init__(self):
        super().__init__("subjugator_read_switch_node")

        self.timer_ = self.create_timer(0.1, self.check_gpio)
        self.pub = self.create_publisher(Empty, "read_switch", 10)

    def check_gpio(self):
        value = GPIO.input(INPUT_PIN)
        if value == GPIO.HIGH:
            self.pub.publish(Empty())

    def __del__(self):
        GPIO.cleanup()


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(INPUT_PIN, GPIO.IN)

    rclpy.init()
    node = SubjugatorReadSwitchNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
