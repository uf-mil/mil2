import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool

INPUT_PIN = 18


class SubjugatorReadSwitchNode(Node):
    def __init__(self):
        super().__init__("subjugator_read_switch_node")

        self.timer_ = self.create_timer(0.1, self.check_gpio)
        self.pub = self.create_publisher(String, "read_switch", 10)
        self.prev_value = False  # false = low, true = high

        # reset localization
        self.localization_client = self.create_client(
            Empty,
            "/subjugator_localization/reset",
        )

        # reset controller
        self.controller_reset_client = self.create_client(
            Empty,
            "/pid_controller/reset",
        )

        # start vs stop controller
        self.controller_client = self.create_client(SetBool, "/pid_controller/enable")

        # unkill
        self.unkill_client = self.create_client(Empty, "/unkill")

    def check_gpio(self):
        value = GPIO.input(INPUT_PIN)
        msg_str = "high" if value == GPIO.HIGH else "low"
        msg = String()
        msg.data = msg_str
        self.pub.publish(msg)

        if self.prev_value and value == GPIO.LOW:
            self.reset_sub()

    def reset_sub(self):
        msg = Empty().Request()

        start_msg = SetBool().Request()
        start_msg.data = True

        stop_msg = SetBool().Request()
        stop_msg.data = False

        # stop-controller
        self.controller_client.call(stop_msg)
        rclpy.spin_once(self, timeout_sec=0.5)

        # unkill
        self.unkill_client.call(msg)

        # reset-localization
        self.localization_client.call(msg)

        # reset-controller
        self.controller_reset_client.call(msg)

        # start-controller
        self.controller_client.call(start_msg)

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
