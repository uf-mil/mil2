import numpy as np
import rclpy
from geometry_msgs.msg import AccelWithCovarianceStamped, Wrench
from rclpy.node import Node

# This node estimates the effects of drag on the submarine from dynamics
# It computes the net force and torque on the submarine (from F=mA and Tau = I*Alpha)
# Then compares this to the wrench from the thrusters
# Any discrepancy between the expected wrench (from thrusters) and actual wrench (from dynamics) must come from drag/other disturbances
# We then negate that disturbance then add it back to counteract it.


class DragAdaptation(Node):

    def __init__(self):

        super().__init__("drag_adaptation")
        self.cmd_subscription = self.create_subscription(
            Wrench,
            "cmd_wrench",
            self.listener_callback,
            10,
        )

        self.cmd_subscription  # prevent unused variable warning

        self.accel_subscription = self.create_subscription(
            AccelWithCovarianceStamped,
            "accel/filtered",
            self.accel_callback,
            10,
        )
        self.control_wrench_publisher = self.create_publisher(
            Wrench,
            "control_wrench",
            10,
        )
        self.control_wrench_publisher  # prevent unused variable warning

        self.mass = 30  # kg
        self.inertia = np.diagflat(np.array([[1.63, 2.9, 3.73]]))  # kg m^2

        self.linear_accel = np.array(
            [
                0.0,
                0.0,
                0.0,
            ],
        )

        self.angular_accel = np.array(
            [
                0.0,
                0.0,
                0.0,
            ],
        )

    def listener_callback(self, msg):
        # get the command wrench, is in body frame
        self.cmd_force = np.array(
            [
                msg.force.x,
                msg.force.y,
                msg.force.z,
            ],
        )

        self.cmd_torque = np.array(
            [
                msg.torque.x,
                msg.torque.y,
                msg.torque.z,
            ],
        )

        if np.all(self.cmd_force == 0.0) and np.all(self.cmd_torque == 0.0):
            # If all elements of the command wrench are 0 (no command) return no update
            control_wrench = Wrench()
            control_wrench.force.x = 0.0
            control_wrench.force.y = 0.0
            control_wrench.force.z = 0.0
            control_wrench.torque.x = 0.0
            control_wrench.torque.y = 0.0
            control_wrench.torque.z = 0.0

            self.control_wrench_publisher.publish(control_wrench)
            return

        # Calculate force and torque on the sub not from the thrusters
        drag_force = self.mass * self.linear_accel - self.cmd_force
        drag_torque = np.matmul(self.inertia, self.angular_accel) - self.cmd_torque

        print(drag_torque)
        # Concatenate the drag force and drag torque, then negate them
        drag_compensation = np.hstack((drag_force, drag_torque))

        self.cmd_wrench = np.hstack((self.cmd_force, self.cmd_torque))
        self.sum_wrench = self.cmd_wrench + drag_compensation

        control_wrench = Wrench()
        control_wrench.force.x = self.sum_wrench[0]
        control_wrench.force.y = self.sum_wrench[1]
        control_wrench.force.z = self.sum_wrench[2]
        control_wrench.torque.x = self.sum_wrench[3]
        control_wrench.torque.y = self.sum_wrench[4]
        control_wrench.torque.z = self.sum_wrench[5]

        self.control_wrench_publisher.publish(control_wrench)

    def accel_callback(self, msg):
        # get recent acceleration
        self.linear_accel = np.array(
            [
                msg.accel.accel.linear.x,
                msg.accel.accel.linear.y,
                msg.accel.accel.linear.z,
            ],
        )
        print(self.linear_accel)

        self.angular_accel = np.array(
            [
                msg.accel.accel.angular.x,
                msg.accel.accel.angular.y,
                msg.accel.accel.angular.z,
            ],
        )


def main(args=None):
    rclpy.init(args=args)

    drag_adaptation = DragAdaptation()

    rclpy.spin(drag_adaptation)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drag_adaptation.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
