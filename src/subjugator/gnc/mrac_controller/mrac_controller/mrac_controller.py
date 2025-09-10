import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import rotation as R


class MRAC(Node):

    def __init__(self):

        self.use_adaptive = True
        super().__init__("mrac_controller")

        self.goal_subscription = self.create_subscription(
            Pose,
            "goal/trajectory",
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

        self.output_wrench = Wrench()

        # Initialize goal and current pose (x,y,z,roll,pitch,yaw)
        self.goal = []
        self.current = []

        # Initialize gains for the PD controller, can parameterize these later
        P = np.array([100.0, 100.0, 150.0, 10.0, 5.0, 25.0])
        D = np.array([60.0, 0.0, 20.0, 0.0, 0.0, 0.0])

        # Initialize physical params
        mass = 29.5  # kg
        inertia = [
            0,
            0,
            0,
        ]  # kgm2  (ixx,iyy, izz)  Can calculate from fusion model (probably?)

        self.physical_properties = np.array(
            [mass, mass, mass, inertia[0], inertia[1], inertia[2]],
        )

        ones_vec = np.transpose(np.array([1, 1, 1, 1, 1, 1]))

        # Create a square matrix with the gain values along the diagonal (useful for multiplying later)
        self.Pmat = np.matmul(ones_vec, P)
        self.Dmat = np.matmul(ones_vec, D)

        # Initialize adaptive vectors
        self.disturbance_estimate = np.zeros(6)
        self.last_disturbance_estimate = np.zeros(6)
        self.drag_estimate = np.zeros(9)
        self.last_drag_estimate = np.zeros(9)

        self.ki = 0.1  # learning gain for the disturbance estimate
        self.kg = 0.1  # learning gain for the drag estimate
        self.dt = 0.05  # step size for learning

    def goal_pose_callback(self, msg):
        # Hears a goal pose from the trajectory planner, then deconstructs it

        # Convert the quaternion from the goal pose into rpy
        goal_rot = quat_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

        # Construct goal pose (x,y,z, roll, pitch, yaw)
        self.goal = [
            msg.position.x,
            msg.position.y,
            msg.position.z,
            goal_rot[0],
            goal_rot[1],
            goal_rot[2],
        ]

    def odometry_callback(self, msg):
        # Get the latest pose, then computes control action necessary

        # Convert the current rotation to rpy
        current_rot = quat_to_euler(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

        # Create pose
        self.current = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            current_rot[0],
            current_rot[1],
            current_rot[2],
        ]

        # Velocity array
        self.vel = [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z,
        ]

        # Compute e and e_dot based on error from goal pose

        self.error = get_error(self.goal, self.current)
        self.error_dot = np.array(self.vel)

        # Decide if using just PD or PD + adaption
        if self.use_adaptive:
            # TODO Compute anticipation feed forward using the sub's acceleration, angular acceleration, mass, and inertia

            # Compute disturbance estimate
            self.disturbance_estimate = (
                self.last_disturbance_estimate + self.ki * self.dt * self.error
            )

            # Compute drag regressor - transform velocity into sub frame and compute cross-coupling effects - this will be a super fun 6x9 matrix :)

            # need to add more terms, currently is just vel and angular vel
            self.drag_regression = np.array(
                [
                    [self.vel[0], 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, self.vel[1], 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, self.vel[2], 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, self.vel[3], 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, self.vel[4], 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, self.vel[5]],
                ],
            )

            self.drag_estimate = (
                self.last_drag_estimate
                + self.kg
                * self.dt
                * np.transpose(self.drag_regression)
                @ (self.error + self.error_dot)
            )

            wrench = (
                self.Pmat @ self.error
                + self.Dmat @ self.error_dot
                + self.disturbance_estimate
                + np.transpose(self.drag_estimate) @ np.transpose(self.drag_regression)
            )

        # If only PD, compute the wrench with the errors and gains, then publish wrench
        else:
            wrench = self.Pmat @ self.error + self.Dmat @ self.error_dot

        # Update the disturbance and drag estimate
        self.last_disturbance_estimate = self.disturbance_estimate
        self.last_drag_estimate = self.drag_estimate

        self.output_wrench.force.x = wrench[0]
        self.output_wrench.force.y = wrench[1]
        self.output_wrench.force.z = wrench[2]
        self.output_wrench.torque.x = wrench[3]
        self.output_wrench.torque.y = wrench[4]
        self.output_wrench.torque.z = wrench[5]


# Computer error e = goal-current
def get_error(goal, current):
    e = np.eye(6)
    for pos in range(len(goal)):
        e[pos] = goal[pos] - current[pos]

    return e


def quat_to_euler(x, y, z, w):
    # Get the pose as a quaternion
    q = [x, y, z, w]

    # Create a scipy rotation object from quaternion
    rot = R.from_quat(q)

    # Convert the quaternion to roll, pitch, yaw (in rads)
    rot.as_euler("xyz", degrees=False)

    return rot


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
