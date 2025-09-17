import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R


class MRAC(Node):

    def __init__(self):

        # Config:
        self.use_adaptive = False

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

        # Initialize error and error dot
        self.error = np.zeros((6, 1))
        self.error_dot = np.zeros((6, 1))

        # Initialize gains for the PD controller, can parameterize these later
        self.Pmat = np.diagflat([[80.0, 30.0, 45.0, 5.0, 5.0, 5.0]])
        self.Dmat = np.diagflat([[135.0, 0.0, 2.0, 10.0, 0.0, 5.0]])

        # Initialize physical params
        mass = 29.5  # kg
        inertia = [
            0.0,
            0.0,
            0.0,
        ]  # kgm2  (ixx,iyy, izz)  Can calculate from fusion model

        self.physical_properties = np.array(
            [mass, mass, mass, inertia[0], inertia[1], inertia[2]],
        )

        # Initialize adaptive vectors
        self.disturbance_estimate = np.zeros((6, 1))
        self.last_disturbance_estimate = np.zeros((6, 1))

        self.drag_estimate = np.zeros((6, 1))
        self.last_drag_estimate = np.zeros((6, 1))

        self.ki = 0.01  # learning gain for the disturbance estimate
        self.kg = 0.1  # learning gain for the drag estimate
        self.dt = 0.05  # step size for learning

        self.heard_goal = False

    def goal_pose_callback(self, msg):

        print("MRAC heard goal pose")
        # Hears a goal pose from the trajectory planner, then deconstructs it

        # Convert the quaternion from the goal pose into rpy
        goal_quat = R.from_quat(
            [
                msg.orientation.w,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
            ],
        )

        goal_rot = goal_quat.as_euler("xyz", degrees=False)

        # Construct goal pose (x,y,z, roll, pitch, yaw)
        self.goal = [
            msg.position.x,
            msg.position.y,
            msg.position.z,
            goal_rot[0],
            goal_rot[1],
            goal_rot[2],
        ]

        self.heard_goal = True

    def odometry_callback(self, msg):

        if self.heard_goal:
            # Convert the current rotation to rpy
            current_quat = R.from_quat(
                [
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                ],
            )

            current_rot = current_quat.as_euler("xyz", degrees=True)

            # Current pose
            self.current = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                current_rot[0],
                current_rot[1],
                current_rot[2],
            ]

            # Velocity array
            self.vel = [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            ]

            # Compute e and e_dot
            self.error = get_error(self, self.goal, self.current)
            self.error_dot = np.array(self.vel)

            # Decide if using PD or PD + adaption
            if self.use_adaptive:
                self.disturbance_estimate = (
                    self.last_disturbance_estimate + self.ki * self.dt * self.error
                )
                # This matrix should represent how drag is created, it probably needs some work - will look at sub dynamics to create
                # self.drag_regression = np.array(
                #     [
                #         [self.vel[0] ** 2, 0.0, 0.0, 0.0, 0.0, 0.0],
                #         [0.0, self.vel[1] ** 2, 0.0, 0.0, 0.0, 0.0],
                #         [0.0, 0.0, self.vel[2] ** 2, 0.0, 0.0, 0.0],
                #         [0.0, 0.0, 0.0, self.vel[0] ** 2, 0.0, 0.0],
                #         [0.0, 0.0, 0.0, 0.0, self.vel[1] ** 2, 0.0],
                #         [0.0, 0.0, 0.0, 0.0, 0.0, self.vel[2] ** 2],
                #     ],
                # )

                # Need to convert top left 3x3 to body frame
                # Need bottom right 3x3 to be undersired angular velocity

                # Matrix is: top right 3x3 is linear drag, top mid 3x3 assumed 0 linear drag from rotation, top right 3x3 is 0
                # Bottom left 3x3 is linear drag from rotation (assume 0), bottom middle 3x3 is moment caused by linear velocity (main component of drag), bottom right 3x3 is undesired angular velocity
                self.drag_regression = np.array(
                    [
                        [self.vel[0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, self.vel[1], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, self.vel[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, self.vel[0] ** 2, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, self.vel[1] ** 2, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, self.vel[2] ** 2],
                    ],
                )

                self.drag_estimate = (
                    self.last_drag_estimate
                    + self.kg
                    * self.dt
                    * np.matmul(
                        np.transpose(self.drag_regression),
                        (self.error + self.error_dot),
                    )
                )

                wrench = (
                    self.Pmat @ self.error
                    + self.Dmat @ self.error_dot
                    + self.disturbance_estimate
                    + np.transpose(self.drag_estimate)
                    @ np.transpose(self.drag_regression)
                )
            else:
                wrench = self.Pmat @ self.error  # + self.Dmat @ self.error_dot
                print(f"error: {(np.linalg.norm(self.error))}")

            # Flatten wrench to ensure shape (6,)
            wrench = np.array(wrench).flatten()

            # Update adaptive states
            self.last_disturbance_estimate = self.disturbance_estimate
            self.last_drag_estimate = self.drag_estimate

            # Assign as floats (fix for numpy.float64 issue)
            self.output_wrench.force.x = float(wrench[0])
            self.output_wrench.force.y = float(wrench[1])
            self.output_wrench.force.z = float(wrench[2])
            self.output_wrench.torque.x = float(wrench[3])
            self.output_wrench.torque.y = float(wrench[4])
            self.output_wrench.torque.z = float(wrench[5])

            # Weigh each element of the error so the large value of the angular error (in degs) doesn't overpower the linear values (in m)
            self.error_weight = np.array(
                [
                    [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0 / 360.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 1.0 / 360.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0 / 360.0],
                ],
            )

            # multiply the weight matrix by the error
            self.error_dist = self.error_weight @ self.error

            # If the error is < 0.2, the goal pose has been arrived at. Set heard goal to false and send a 0 command (to stop sub spinning infinitely)
            if np.linalg.norm(self.error_dist) < 0.2:
                self.heard_goal = False

                self.output_wrench.force.x = 0.0
                self.output_wrench.force.y = 0.0
                self.output_wrench.force.z = 0.0
                self.output_wrench.torque.x = 0.0
                self.output_wrench.torque.y = 0.0
                self.output_wrench.torque.z = 0.0
            self.cmd_wrench_publisher.publish(self.output_wrench)


# Compute error e = goal-current


def get_error(self, goal, current):
    # x,y,z,roll,pitch,yaw
    self.error = np.zeros((6, 1))
    for pos in range(len(goal)):
        self.error[pos] = goal[pos] - current[pos]

    return self.error


def main(args=None):
    rclpy.init(args=args)
    mrac_controller = MRAC()
    rclpy.spin(mrac_controller)
    mrac_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
