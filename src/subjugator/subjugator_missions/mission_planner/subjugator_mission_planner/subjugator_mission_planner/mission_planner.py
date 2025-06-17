import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node

from subjugator_mission_planner.action import (
    NavigateAround,
    NavigateThrough,
    SearchObject,
)


class MissionPlanner(Node):
    def __init__(self):
        super().__init__("mission_planner")

        self.declare_parameter("mission_file", "mission.yaml")
        mission_file = (
            self.get_parameter("mission_file").get_parameter_value().string_value
        )

        self.mission = self.load_mission_file(mission_file)
        self.current_task_index = 0
        self.executing_task = False

        # Create action clients for each task type
        self.search_client = ActionClient(self, SearchObject, "search_for_object")
        self.navigate_through_client = ActionClient(
            self,
            NavigateThrough,
            "navigate_through_object",
        )
        self.navigate_around_client = ActionClient(
            self,
            NavigateAround,
            "navigate_around_object",
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
        if task_name == "search_for_object":
            self.send_search_goal(params)
        elif task_name == "navigate_through_object":
            self.send_navigate_through_goal(params)
        elif task_name == "navigate_around_object":
            self.send_navigate_around_goal(params)
        else:
            self.get_logger().error(f"Unknown task: {task_name}, skipping")
            self.current_task_index += 1

    def send_search_goal(self, params):
        if not self.search_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("SearchObject action server not available")
            return

        goal_msg = SearchObject.Goal()
        goal_msg.object = params.get("object", "")

        self.executing_task = True
        self._send_goal(self.search_client, goal_msg)

    def send_navigate_through_goal(self, params):
        if not self.navigate_through_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("NavigateThroughObject action server not available")
            return

        goal_msg = NavigateThrough.Goal()
        goal_msg.object_id = params.get("object_id", "")

        self.executing_task = True
        self._send_goal(self.navigate_through_client, goal_msg)

    def send_navigate_around_goal(self, params):
        if not self.navigate_around_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("NavigateAroundObject action server not available")
            return

        goal_msg = NavigateAround.Goal()
        goal_msg.object_id = params.get("object_id", "")

        self.executing_task = True
        self._send_goal(self.navigate_around_client, goal_msg)

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
