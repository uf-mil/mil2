import sys

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

        # Get epsilon to avoid dividing by 0
        self.eps = sys.float_info.epsilon

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
            self.current_quat = R.from_quat(
                [
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                ],
            )

            current_rot = self.current_quat.as_euler("xyz", degrees=True)

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

            # Convert velocities to body frame
            body_vel = self.current_quat.apply(self.vel[0:3])

            # Compute e and e_dot
            self.error = get_error(self, self.goal, self.current)
            self.error_dot = body_vel

            # Decide if using PD or PD + adaption
            if self.use_adaptive:

                # get the sign of each part of the translational velocity

                # Get the sign of each element of the velocity and twist
                self.sx = self.vel[0] / abs(self.vel[0] + self.eps)
                self.sy = self.vel[1] / abs(self.vel[1] + self.eps)
                self.sz = self.vel[2] / abs(self.vel[2] + self.eps)
                self.stx = self.vel[3] / abs(self.vel[3] + self.eps)
                self.stx = self.vel[4] / abs(self.vel[4] + self.eps)
                self.stz = self.vel[5] / abs(self.vel[5] + self.eps)

                # Get the velocity squared while keeping its sign - this is our proxy for drag
                self.vx2 = self.sx * body_vel[0] ** 2
                self.vy2 = self.sx * body_vel[1] ** 2
                self.vz2 = self.sx * body_vel[2] ** 2

                # Put the squared velocities in a matrix for easier calculation
                self.drag_regression = np.array(
                    [[self.vx2, 0.0, 0.0], [0.0, self.vy2, 0.0], [0.0, 0.0, self.vz2]],
                )

                # multiply the angular error by drag gains (these will be learned eventually). Here we are essentially only compensating for undersired angular error
                self.drag_gains = np.transpose(
                    np.array(
                        [
                            abs(self.error[3]) * 0.0,
                            abs(self.error[4]) * 0.0,
                            abs(self.error[5]) * -100.0,
                        ],
                    ),
                )

                # Get a 3x1 vec of twist compensation
                self.twist_comp = np.matmul(self.drag_gains, self.drag_regression)

                print(
                    f"thetaZ error: {self.error[5]} , Z twist comp: {self.twist_comp[0][2]}",
                )
                # Put the 3x1 twist comp below a 3x1 of zeros
                self.drag_comp = np.transpose(
                    np.array(
                        [
                            [
                                0.0,
                                0.0,
                                0.0,
                                self.twist_comp[0][0],
                                self.twist_comp[0][1],
                                self.twist_comp[0][2],
                            ],
                        ],
                    ),
                )

                wrench = (
                    self.Pmat @ self.error
                    # + self.Dmat @ self.error_dot
                    + self.drag_comp
                )

            else:
                wrench = self.Pmat @ self.error  # + self.Dmat @ self.error_dot
                print(f"error: {(np.linalg.norm(self.error))}")

            # Flatten wrench to ensure shape (6,)
            wrench = np.array(wrench).flatten()

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

            # If the error is < 0.2, the goal pose has been arrived at
            if np.linalg.norm(self.error_dist) < 0.2:
                print("Arrived at goal pose")

                self.heard_goal = False

                # Send a final force of 0s to avoid spinning out
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
