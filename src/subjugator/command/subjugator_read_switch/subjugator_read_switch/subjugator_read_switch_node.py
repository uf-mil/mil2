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
        self.prev_value = True  # false = low, true = high
        self.odom_heard = False

        # reset localization
        self.reset_localization_client = self.create_client(
            Empty,
            "/subjugator_localization/reset",
        )

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

        # start mission planner
        self.start_mission_planner = self.create_client(
            Empty,
            "/mission_planner/enable",
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
            print("reset")
            self.reset_sub()
            self.timer_.cancel()

        self.prev_value = value == GPIO.HIGH

    def reset_sub(self):
        msg = Empty.Request()

        start_msg = SetBool.Request()
        start_msg.data = True

        stop_msg = SetBool.Request()
        stop_msg.data = False

        # unkill
        print("before unkilled")
        future = self.unkill_client.call_async(msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)
        print("unkilled")

        # reset-localization
        future = self.reset_localization_client.call_async(msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)

        # start-controller
        future = self.controller_client.call_async(start_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)

        # start-mission planner
        future = self.start_mission_planner.call_async(msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)

        rclpy.shutdown()

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
