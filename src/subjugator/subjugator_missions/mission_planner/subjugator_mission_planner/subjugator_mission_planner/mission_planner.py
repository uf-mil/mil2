import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from rclpy.action.client import ActionClient
from rclpy.node import Node
from subjugator_msgs.action import (
    Movement,
    NavigateAround,
    Wait,
    TrackObject,
)


class MissionPlanner(Node):
    def __init__(self):
        super().__init__("mission_planner")

        package_share = get_package_share_directory("subjugator_mission_planner")
        print(f"PATH: {package_share}")
        default_mission_file = os.path.join(package_share, "missions", "poop.yaml")
        self.declare_parameter("mission_file", default_mission_file)
        mission_file = os.path.join(
            package_share,
            "missions",
            self.get_parameter("mission_file").get_parameter_value().string_value,
        )

        self.mission = self.load_mission_file(mission_file)
        self.current_task_index = 0
        self.executing_task = False

        # Create action clients for each task type
        self.movement_client = ActionClient(
            self,
            Movement,
            "movement_server",
        )
        self.navigate_around_client = ActionClient(
            self,
            NavigateAround,
            "navigate_around_object",
        )
        self.wait_client = ActionClient(
            self,
            Wait,
            "wait",
        )
        self.track_object_client = ActionClient(
            self,
            TrackObject,
            "track_object",
        )

        # Timer to periodically check mission progress and start tasks
        self.timer = self.create_timer(0.5, self.execute_mission)

    # Load mission file from yaml
    def load_mission_file(self, filepath):
        try:
            with open(filepath) as f:
                mission_data = yaml.safe_load(f)
            self.get_logger().info(f"Mission file loaded: {filepath}")
            return mission_data["mission"]
        except Exception as e:
            self.get_logger().error(f"Failed to load mission file: {e}")
            return []

    def execute_mission(self):
        if self.executing_task:
            # Currently running a task; wait for it to complete
            return

        if self.current_task_index >= len(self.mission):
            self.get_logger().info("Mission complete!")
            return

        # Iterate through the mission yaml completing each task
        task = self.mission[self.current_task_index]
        task_name = task.get("task")
        params = task.get("parameters", {})

        self.get_logger().info(
            f"Starting task {self.current_task_index + 1}/{len(self.mission)}: {task_name}",
        )

        # Dispatch to the correct action client
        if task_name == "move":
            self.send_move_to_goal(params)
        elif task_name == "navigate_around_object":
            self.send_navigate_around_goal(params)
        elif task_name == "wait":
            self.send_wait_goal(params)
        elif task_name == "track_object":
            self.send_track_object_goal(params)
        else:
            self.get_logger().error(f"Unknown task: {task_name}, skipping")
            self.current_task_index += 1

    def send_move_to_goal(self, params):
        if not self.movement_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Movement action server not available")
            return

        goal_pose = Pose()
        goal_pose.position.x = params.get("x", 0.0)
        goal_pose.position.y = params.get("y", 0.0)
        goal_pose.position.z = params.get("z", 0.0)
        goal_pose.orientation.x = params.get("i", 0.0)
        goal_pose.orientation.y = params.get("j", 0.0)
        goal_pose.orientation.z = params.get("k", 0.0)
        goal_pose.orientation.w = params.get("w", 1.0)

        goal_msg = Movement.Goal()
        goal_msg.goal_pose = goal_pose
        goal_msg.type = params.get("type", "Relative")

        self.executing_task = True
        self._send_goal(self.movement_client, goal_msg)

    def send_track_object_goal(self, params):
        if not self.track_object_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Track object goal no working")
            return

        goal_msg = TrackObject.Goal()
        goal_msg.object_type = "green"
        goal_msg.type_of_movement = params.get("type_of_movement", "rotate") # either translate or rotate

        self.executing_task = True
        self._send_goal(self.track_object_client, goal_msg)

    def send_navigate_around_goal(self, params):
        if not self.track_object_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("track_object action server not available")
            return

        goal_msg = TrackObject.Goal()
        goal_msg.object_type = "idk"

        self.executing_task = True
        self._send_goal(self.navigate_around_client, goal_msg)

    def send_wait_goal(self, params):
        if not self.wait_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Wait action server not available")
            return

        goal_msg = Wait.Goal()
        goal_msg.time = params.get("time", 0.0)

        self.executing_task = True
        self._send_goal(self.wait_client, goal_msg)

    def _send_goal(self, client: ActionClient, goal_msg):
        send_goal_future = client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server")
            self.executing_task = False
            self.current_task_index += 1
            return

        self.get_logger().info("Goal accepted, executing...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: {feedback.status_message}")

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"Task {self.current_task_index + 1} succeeded")
        else:
            self.get_logger().error(
                f'Task {self.current_task_index + 1} failed: {getattr(result, "message", "")}',
            )
            # Here you can add retry logic or abort mission if needed

        self.executing_task = False
        self.current_task_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
