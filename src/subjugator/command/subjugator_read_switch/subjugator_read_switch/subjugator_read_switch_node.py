import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

INPUT_PIN = 18


class SubjugatorReadSwitchNode(Node):
    def __init__(self):
        super().__init__("subjugator_read_switch_node")

        self.timer_ = self.create_timer(0.1, self.check_gpio)
        self.pub = self.create_publisher(String, "read_switch", 10)
        self.prev_value = False  # false = low, true = high

    def check_gpio(self):
        value = GPIO.input(INPUT_PIN)
        msg_str = "high" if value == GPIO.HIGH else "low"
        msg = String()
        msg.data = msg_str
        self.pub.publish(msg)

        if self.prev_value and value == GPIO.LOW:
            self.reset_sub()

    def reset_sub(self):
        pass

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
