import numpy as np
import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node


class WrenchTuner(Node):

    def __init__(self):

        super().__init__("wrench_tuner")
        self.cmd_subscription = self.create_subscription(
            Wrench,
            "cmd_wrench",
            self.listener_callback,
            10,
        )

        self.cmd_subscription  # prevent unused variable warning

        self.odom_subscription = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.odometry_callback,
            10,
        )

        self.control_wrench_publisher = self.create_publisher(
            Wrench,
            "control_wrench",
            10,
        )
        self.control_wrench_publisher  # prevent unused variable warning

        self.declare_parameter("c1", 0.0)
        self.declare_parameter("c2", 0.0)
        self.declare_parameter("c3", 0.0)
        self.declare_parameter("c4", 0.0)
        self.declare_parameter("c5", 0.0)
        self.declare_parameter("c6", 0.0)
        self.declare_parameter("rx", 0.0)
        self.declare_parameter("ry", 0.0)
        self.declare_parameter("rz", 0.0)

        self.params = [
            self.get_parameter("c1").value,
            self.get_parameter("c2").value,
            self.get_parameter("c3").value,
            self.get_parameter("c4").value,
            self.get_parameter("c5").value,
            self.get_parameter("c6").value,
            self.get_parameter("rx").value,
            self.get_parameter("ry").value,
            self.get_parameter("rz").value,
        ]

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.velocity = np.zeros(3)

    def listener_callback(self, msg):
        self.cmd_wrench = np.array(
            [
                msg.force.x,
                msg.force.y,
                msg.force.z,
                msg.torque.x,
                msg.torque.y,
                msg.torque.z,
            ],
        )

        if (
            msg.force.x == 0.0
            and msg.force.y == 0.0
            and msg.force.z == 0.0
            and msg.torque.x == 0.0
            and msg.torque.y == 0.0
            and msg.torque.z == 0.0
        ):
            # self.get_logger().info("ignoring wrench")
            control_wrench = Wrench()
            control_wrench.force.x = 0
            control_wrench.force.y = 0
            control_wrench.force.z = 0
            control_wrench.torque.x = 0
            control_wrench.torque.y = 0
            control_wrench.torque.z = 0

            self.control_wrench_publisher.publish(msg)
            return

        # square the velocity as it has a quadratic effect on drag
        self.vx = self.velocity[0] ** 2
        self.vy = self.velocity[1] ** 2
        self.vz = self.velocity[2] ** 2

        self.rx = self.params[6]
        self.ry = self.params[7]
        self.rz = self.params[8]

        self.drag_wrench = np.array(
            [
                self.params[0] * self.vx,
                self.params[1] * self.vy,
                self.params[2] * self.vz,
                self.params[3] * (self.ry * self.vz - self.rz * self.vy),
                self.params[4] * (-1 * self.rx * self.vz + self.rz * self.vx),
                self.params[5] * (self.rx * self.vy - self.ry * self.vx),
            ],
        )

        self.sum_wrench = self.cmd_wrench + self.drag_wrench

        control_wrench = Wrench()
        control_wrench.force.x = self.sum_wrench[0]
        control_wrench.force.y = self.sum_wrench[1]
        control_wrench.force.z = self.sum_wrench[2]
        control_wrench.torque.x = self.sum_wrench[3]
        control_wrench.torque.y = self.sum_wrench[4]
        control_wrench.torque.z = self.sum_wrench[5]

        self.control_wrench_publisher.publish(control_wrench)

    def odometry_callback(self, msg):
        # stores the most recent velocity from odom
        self.velocity = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ],
        )

    def parameter_callback(self, params):
        for param in params:
            for i, name in enumerate(
                ["c1", "c2", "c3", "c4", "c5", "c6", "rx", "ry", "rz"],
            ):

                if param.name == name:
                    self.params[i] = param.value
                    self.get_logger().info(f"{name} updated to {param.value}")
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    wrench_tuner = WrenchTuner()

    rclpy.spin(wrench_tuner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wrench_tuner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
