import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

# This node is for testing a new controller for SubjuGator 9.
# Once working, this should ideally just become part of the existing subjugator_controller node


# Though called MRAC (that was the originally intent), this is not an MRAC controller
# I will call it the DGA controller (Drag Go Away)
class MRAC(Node):

    def __init__(self):

        # Config:
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

        # Initialize error and error dot
        self.error = np.zeros((6, 1))
        self.error_dot = np.zeros((6, 1))

        # Initialize gains for the PD controller, can parameterize these later
        self.Pmat = np.diagflat([[100.0, 125.0, 45.0, 5.0, 5.0, 5.0]])
        self.Dmat = np.diagflat([[135.0, 0.0, 2.0, 10.0, 0.0, 5.0]])

        self.drag_gains = np.zeros((9, 1))

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

            # Compute e and e_dot
            self.error = get_error(self, self.goal, self.current)
            self.error_dot = np.array(self.vel)

            # Decide if using PD or PD + adaption
            if self.use_adaptive:

                # get the sign of each part of the translational velocity

                eps = sys.float_info.epsilon
                self.sx = self.vel[0] / abs(self.vel[0] + eps)
                self.sy = self.vel[1] / abs(self.vel[1] + eps)
                self.sz = self.vel[2] / abs(self.vel[2] + eps)

                self.stz = self.vel[5] / abs(self.vel[5] + eps)

                # Need to convert top left 3x3 to body frame
                # Need bottom right 3x3 to be undersired angular velocity

                # Matrix is: top right 3x3 is linear drag, top mid 3x3 assumed 0 linear drag from rotation, top right 3x3 is 0
                # Bottom left 3x3 is linear drag from rotation (assume 0), bottom middle 3x3 is moment caused by linear velocity (main component of drag), bottom right 3x3 is undesired angular velocity
                # Need to convert velocity to body frame for top left 3x3

                # Convert velocities to body frame
                body_vel = self.current_quat.apply(self.vel[0:3])
                self.vx = body_vel[0]
                self.vx = body_vel[1]
                self.vx = body_vel[2]

                self.drag_regression = np.array(
                    [
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [
                            0.0,
                            0.0,
                            0.0,
                            self.sx * self.vel[0] ** 2,
                            0.0,
                            self.sz * self.vel[2] ** 2,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        [
                            0.0,
                            0.0,
                            0.0,
                            self.sx * self.vel[0] ** 2,
                            0.0,
                            self.sz * self.vel[2] ** 2,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        [
                            0.0,
                            0.0,
                            0.0,
                            self.sx * self.vel[0] ** 2,
                            self.sy * self.vel[1] ** 2,
                            self.sz * self.vel[2] ** 2,
                            0.0,
                            0.0,
                            self.stz * self.vel[5] ** 2,
                        ],
                    ],
                )

                self.drag_gains = np.transpose(
                    np.array(
                        [
                            [
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                -40.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                            ],
                        ],
                    ),
                )

                self.drag_comp = np.matmul(self.drag_regression, self.drag_gains)

                print(self.drag_comp)

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
