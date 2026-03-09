#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.client import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from subjugator_msgs.action import Move, TrashPickup
from yolo_msgs.msg import DetectionArray

IMAGE_WIDTH = 840
IMAGE_HEIGHT = 680


class TrashPickupServer(Node):
    def __init__(self):
        super().__init__("trashpickup")

        # sensor subscriptions
        self.front_detections = []
        self.create_subscription(
            DetectionArray,
            "/yolo/detections",
            self._front_det_cb,
            10,
        )

        self.current_odom = Odometry()
        self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self._odom_cb,
            10,
        )

        # gripper control (Gazebo)
        self.keypress_pub = self.create_publisher(String, "/keyboardkeypress", 10)
        self.gripper_is_open = False

        # action client to command movement
        self.move_client = ActionClient(self, Move, "move")

        # node's action server
        self._action_server = ActionServer(
            self,
            TrashPickup,
            "trashpickup",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info("TrashPickupServer ready")

    def _front_det_cb(self, msg):
        self.front_detections = msg.detections

    def _odom_cb(self, msg):
        self.current_odom = msg

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"Received trash pickup goal for: {goal_request.target_label}",
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Trash pickup cancelled")
        return CancelResponse.ACCEPT

    def open_gripper(self):
        if not self.gripper_is_open:
            self._send_gripper_toggle()
            self.gripper_is_open = True

    def close_gripper(self):
        if self.gripper_is_open:
            self._send_gripper_toggle()
            self.gripper_is_open = False

    def _send_gripper_toggle(self):
        msg = String()
        msg.data = "u"
        self.keypress_pub.publish(msg)
        time.sleep(0.5)

    def send_move_and_wait(
        self,
        move_type,
        x=0.0,
        y=0.0,
        z=0.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
    ):
        goal = Move.Goal()
        goal.type = move_type
        goal.goal_pose = Pose()
        goal.goal_pose.position.x = float(x)
        goal.goal_pose.position.y = float(y)
        goal.goal_pose.position.z = float(z)
        goal.goal_pose.orientation.x = float(qx)
        goal.goal_pose.orientation.y = float(qy)
        goal.goal_pose.orientation.z = float(qz)
        goal.goal_pose.orientation.w = float(qw)

        future = self.move_client.send_goal_async(goal)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Move goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        return result_future.result().result.success

    def find_target(self, detections, label, min_confidence=0.3):
        best = None
        best_score = 0.0
        for det in detections:
            if (
                det.class_name == label
                and det.score >= min_confidence
                and det.score > best_score
            ):
                best = det
                best_score = det.score
        return best

    def execute_callback(self, goal_handle):
        label = goal_handle.request.target_label
        timeout = goal_handle.request.timeout
        self.get_logger().info(f"Executing trash pickup for: {label}")

        feedback = TrashPickup.Feedback()
        start_time = time.time()

        # phase 1: search
        feedback.status = "searching"
        goal_handle.publish_feedback(feedback)

        found = False
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_set=0.1)

            det = self.find_target(self.front_detections, label)
            if det is not None:
                self.get_logger().info(f"Found {label} (conf={det.score:.2f})")
                found = True
                break

        # rotate ~45 degrees and look again
        yaw_rad = math.radiands(45) / 2
        self.send_move_and_wait(
            "Relative",
            qz=math.sin(yaw_rad),
            qw=math.cos(yaw_rad),
        )

        # report result
        result = TrashPickup.Result()
        if found:
            result.success = True
            result.message = f"Found {label}"
            goal_handle.succeed()
        else:
            result.success = False
            result.message = f"Could not find {label} within timeout"
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = TrashPickupServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
