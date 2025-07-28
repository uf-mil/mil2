import Jetson.GPIO as GPIO
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool

INPUT_PIN = 18


class SubjugatorReadSwitchNode(Node):
    def __init__(self):
        super().__init__("subjugator_read_switch_node")

        self.timer_ = self.create_timer(0.1, self.check_gpio)
        self.pub = self.create_publisher(String, "read_switch", 10)
        self.prev_value = True  # false = low, true = high
        self.odom_heard = False

        # reset localization
        # self.localization_client = self.create_client(
        # Empty,
        # "/subjugator_localization/reset",
        # )

        # reset controller
        # self.controller_reset_client = self.create_client(
        # Empty,
        # "/pid_controller/reset",
        # )

        # start vs stop controller
        self.controller_client = self.create_client(SetBool, "/pid_controller/enable")

        # start localization
        self.localization_client = self.create_client(
            Empty,
            "/subjugator_localization/enable",
        )

        # unkill
        self.unkill_client = self.create_client(Empty, "/unkill")

    def check_gpio(self):
        value = GPIO.input(INPUT_PIN)
        msg_str = "high" if value == GPIO.HIGH else "low"
        msg = String()
        msg.data = msg_str
        self.pub.publish(msg)

        if (not self.prev_value) and value == GPIO.HIGH:
            self.reset_sub()

        self.prev_value = value == GPIO.HIGH

    def reset_sub(self):
        msg = Empty().Request()

        start_msg = SetBool().Request()
        start_msg.data = True

        stop_msg = SetBool().Request()
        stop_msg.data = False

        self.localization_client.call(msg)

        self.odom_heard = False

        def wait_on_odom(_):
            self.odom_heard = True

        odom_sub = self.create_subscription(
            Odometry,
            "odometry/filtered",
            wait_on_odom,
            10,
        )
        while not self.odom_heard:
            rclpy.spin_once(self, timeout_sec=0.2)
        odom_sub.destroy()

        # unkill
        self.unkill_client.call(msg)

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
