import time
import unittest

import launch
import launch_ros
import launch_testing.actions
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, Path


def generate_test_description():
    return (
        launch.LaunchDescription(
            [
                launch_ros.actions.Node(
                    package="subjugator_trajectory_planner",
                    executable="trajectory_planner",
                    name="trajectory_planner",
                    output="screen",
                    parameters=[
                        {
                            "goal_tolerance": 0.5,
                        },
                    ],
                ),
                launch.actions.TimerAction(
                    period=0.5,
                    actions=[launch_testing.actions.ReadyToTest()],
                ),
            ],
        ),
        {},
    )


class TestTrajectoryPlanner(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_trajectory_planner")
        self.odom_publisher = self.node.create_publisher(
            Odometry,
            "/odometry/filtered",
            10,
        )
        self.path_publisher = self.node.create_publisher(Path, "/path", 10)
        self.goal_subscription = self.node.create_subscription(
            Pose,
            "/goal/trajectory",
            self.goal_callback,
            10,
        )
        self.goal = None

    def tearDown(self):
        self.node.destroy_node()

    def goal_callback(self, msg):
        self.goal = msg

    def test_trajectory_planner_initialization(self):
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), "test_trajectory_planner")

    def test_simple_path(self):
        # create and publish a path
        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = self.node.get_clock().now().to_msg()
        for i in range(5):
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "odom"
            pose_msg.header.stamp = self.node.get_clock().now().to_msg()
            pose_msg.pose.position.x = float(i)
            pose_msg.pose.position.y = float(i)
            pose_msg.pose.position.z = 0.0 + i * 0.1
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            path.poses.append(pose_msg)
        self.path_publisher.publish(path)

        # publish odometry data
        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.header.stamp = self.node.get_clock().now().to_msg()
        self.odom_publisher.publish(odom_msg)

        # wait for goal to come in, assert its the first pose in path
        timeout = time.time() + 5  # 5 seconds timeout
        while self.goal is None and timeout > time.time():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertIsNotNone(self.goal)
        # print goal
        self.node.get_logger().info(f"goal: {self.goal}")
        self.node.get_logger().info(f"path: {path.poses[0].pose}")
        self.assertEqual(self.goal, path.poses[1].pose)
