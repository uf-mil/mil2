import inspect
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from rclpy.action.client import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty
from subjugator_msgs import action as action_interfaces


class MissionPlanner(Node):
    def heard_start(self, req: Empty.Request, res: Empty.Response):
        self.waiting = False
        return res

    def __init__(self):
        super().__init__("mission_planner")

        # create waiter for coin
        self.waiting = True
        self.coin_sub = self.create_service(
            Empty,
            "/mission_planner/enable",
            self.heard_start,
        )
        while self.waiting:
            rclpy.spin_once(self, timeout_sec=0.1)

        package_share = get_package_share_directory("subjugator_mission_planner")
        print(f"PATH: {package_share}")

        # Find a mission file, if none is specified use prequal
        default_mission_file = os.path.join("wait_test.yaml")
        self.declare_parameter("mission_file", default_mission_file)
        mission_file = os.path.join(
            package_share,
            "missions",
            self.get_parameter("mission_file").get_parameter_value().string_value,
        )

        self.mission = self.load_mission_file(mission_file)
        self.current_task_index = 0
        self.executing_task = False

        # Create a dictionary to store all of the action clients
        self.action_clients = {}

        # Dictionary is populated by parsing through actions in the subjugator_msgs.action directory
        # The name of the task (i.e what should be used in the mission .yaml file) is an all lowercase of the action name
        # E.g to use the action NavigateAround.action, the mission yaml should contain navigatearound. To use StartGate.action, the mission should use startgate
        self.available_actions = {
            name.lower(): action
            for name, action in inspect.getmembers(action_interfaces, inspect.isclass)
            if hasattr(action, "Goal")
        }

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
        time.sleep(0.1)
        if self.executing_task:
            # Currently running a task; wait for it to complete
            return

        if self.current_task_index >= len(self.mission):
            self.get_logger().info("Mission complete!")
            return

        # Iterate through the mission yaml by loading each task

        # get the task, task name, and the task parameters at each index
        task = self.mission[self.current_task_index]
        task_name = task.get("task")
        params = task.get("parameters", {})

        self.get_logger().info(
            f"Starting task {self.current_task_index + 1}/{len(self.mission)}: {task_name}",
        )

        action_class = self.available_actions.get(task_name.lower())
        if not action_class:
            self.get_logger().error(f"Unknown task: {task_name}")
            self.current_task_index += 1
            return

        client = self.get_or_create_client(task_name, action_class)

        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"{task_name} server not available")
            self.current_task_index += 1
            return
        self.get_logger().info(f"Goal parameters: {params}")

        goal_msg = self.build_goal_message(action_class, params)

        self.executing_task = True
        self._send_goal(client, goal_msg)

    # Create action clients for each action that was found
    def get_or_create_client(self, task_name, action_class):
        if task_name not in self.action_clients:
            self.action_clients[task_name] = ActionClient(
                self,
                action_class,
                task_name,
                callback_group=ReentrantCallbackGroup(),
            )
        return self.action_clients[task_name]

    # Create a generalizable goal message
    def build_goal_message(self, action_class, params):
        goal = action_class.Goal()
        print("Goal fields:", goal.__slots__)
        print("Params received:", params)

        # Populate fields automatically from YAML parameters
        for field_name in goal.__slots__:
            clean_name = field_name.lstrip("_")  # Remove leading underscore
            val = params.get(clean_name)
            if val is not None:
                setattr(goal, clean_name, val)
                print(f"Set {clean_name} to {val}")
                # Handle nested Pose fields
            elif isinstance(getattr(goal, field_name), Pose):
                pose = Pose()
                pose.position.x = params.get("x", 0.0)
                pose.position.y = params.get("y", 0.0)
                pose.position.z = params.get("z", 0.0)
                pose.orientation.x = params.get("i", 0.0)
                pose.orientation.y = params.get("j", 0.0)
                pose.orientation.z = params.get("k", 0.0)
                pose.orientation.w = params.get("w", 1.0)
                setattr(goal, field_name, pose)

        return goal

    # Send the goal message to the relevant action client
    def _send_goal(self, client: ActionClient, goal_msg):
        self.send_goal_future = client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    # Handle the goal response from the action client
    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error(
                f"Goal rejected by server {self.current_task_index}",
            )
            self.executing_task = False
            self.current_task_index += 1
            return

        self.get_logger().info(f"Goal accepted, executing... {self.current_task_index}")
        self.goal_handle_result = self.goal_handle.get_result_async().add_done_callback(
            self.get_result_callback,
        )
        self.get_logger().warn("CALLBACK HAS BEEN SET!!")

    # Handle feedback from the action clients - currently not used
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback: {feedback_msg.feedback}")

    def sleep_for(self, time: float):
        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=time)

        while (self.get_clock().now() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)  # Allow callbacks while waiting

    # Handles results feedback from the action clients - currently not used but would use for behavior tree type behavior
    def get_result_callback(self, future):
        self.get_logger().warn("get_result_callback()")
        try:
            result = future.result().result
            success = getattr(result, "success", False)
            message = getattr(result, "message", "")
            self.get_logger().warn(f"In result callback!!{message}")

            if success:
                self.get_logger().info(f"Task {self.current_task_index + 1} succeeded")
            else:
                self.get_logger().error(
                    f"Task {self.current_task_index + 1} failed: {message}",
                )

            self.executing_task = False
            self.current_task_index += 1
        except Exception:
            import traceback

            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
