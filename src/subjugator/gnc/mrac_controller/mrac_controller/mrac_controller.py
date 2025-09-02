import rclpy
from geometry_msgs.msg import Pose, Wrench
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node


class MRAC(Node):

    def __init__(self):

        super().__init__("mrac_controller")

        self.goal_subscription = self.create_subscription(
            Pose,
            "goal_pose",
            self.goal_pose_callback,
            10,
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.odometry_callback,
            10,
        )

        self.cmd_wrench_publisher = self.create_publisher(
            Wrench,
            "cmd_wrench",
            10,
        )

        self.declare_parameter("kP", 0.0)  # PD controller proportional gain
        self.declare_parameter("kD", 0.0)  # PD controller derivative gain
        self.declare_parameter("kI", 0.0)  # Disturbance estimate learning gain
        self.learning_rate("kG", 0.0)  # Drag estimate learning gain

        self.params = [
            self.get_parameter("kP").value,
            self.get_parameter("kD").value,
            self.get_parameter("kI").value,
            self.get_parameter("kG").value,
        ]

        self.add_on_set_parameters_callback(self.parameter_callback)

    def goal_pose_callback(self, msg):
        # Sets the goal pose (likely waypoint from the trajectory planner)

        pass

    def odometry_callback(self, msg):
        # Get the latest odom

        # Get PD gains, put in matrices

        # Compute e and e_dot based on error from goal pose

        # Compute anticipation feed forward using the sub's acceleration, angular acceleration, mass, and inertia

        # Compute drag regressor - transform velocity into sub frame and compute cross-coupling effects - this will be a super fun 6x9 matrix :)

        # Decide if using just PD or PD + adaption

        # If only PD, compute the wrench with the errors and gains, then publish wrench

        # If using adaptation: wrench = PD + feedforward + disturbance estimate Ddisturb + drag effort Ddrag

        # Update the disturbance estimate and the drag estimate:
        # Ddisturb = Ddisturb + kG * drag regressor * (e + e_dot) * dt
        # Ddrag = Ddrag+ kI*e*dt

        # Repeat!
        pass

    def parameter_callback(self, params):
        for param in params:
            for i, name in enumerate(
                ["kP", "kI", "kD", "kG"],
            ):

                if param.name == name:
                    self.params[i] = param.value
                    self.get_logger().info(f"{name} updated to {param.value}")
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    mrac_controller = MRAC()

    rclpy.spin(mrac_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mrac_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
