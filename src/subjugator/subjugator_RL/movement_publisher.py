import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
import threading
import termios
import sys
import tty
import os
import time

PUBLISH_RATE = 10  # Hertz

class SubjugatorControl(Node):
    def __init__(self):
        super().__init__('subjugator_keyboard_control')
        self.publisher_ = self.create_publisher(Wrench, 'cmd_wrench', PUBLISH_RATE)

        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('angular_speed', 1.0)
        # self.base_linear = self.get_parameter('linear_speed').value
        self.base_linear = 10
        self.base_angular = self.get_parameter('angular_speed').value

        self.force_x = 0.0
        self.force_y = 0.0
        self.force_z = 0.0
        self.torque_x = 0.0
        self.torque_y = 0.0
        self.torque_z = 0.0

        self.running = True

        self.init_terminal()

        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.publisher_thread = threading.Thread(target=self.publish_loop)

        self.keyboard_thread.start()
        self.publisher_thread.start()

        self.get_logger().info("Publishing wrench to /cmd_wrench topic.")

    def init_terminal(self):
        self.old_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_terminal_settings)

    def force_action(self, action):
        # Action is array of two values, first is for forward/back and second is for left/right
        self.force_x = action[0]
        self.force_y = action[1]

    def publish_loop(self):
        rate = self.create_rate(PUBLISH_RATE)
        while rclpy.ok() and self.running:
            msg = Wrench()
            msg.force.x = self.force_x
            msg.force.y = self.force_y
            msg.force.z = self.force_z
            msg.torque.x = self.torque_x
            msg.torque.y = self.torque_y
            msg.torque.z = self.torque_z

            self.publisher_.publish(msg)
            rate.sleep()

    def __del__(self):
        self.restore_terminal()


def main(args=None):
    rclpy.init(args=args)
    node = SubjugatorControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
