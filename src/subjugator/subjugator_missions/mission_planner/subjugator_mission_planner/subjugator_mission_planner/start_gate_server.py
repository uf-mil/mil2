import rclpy
from geometry_msgs.msg import Pose, Quaternion
from rclpy.action.client import ActionClient
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
from subjugator_msgs.action import Move, StartGate


class ActionUser:
    """
    class to make calling other missions easier, you do still kinda have to construct the goals on your own sry
    """

    def __init__(self, node: Node, action_type, action_name: str):
        self.node = node
        self.ac = ActionClient(node, action_type, action_name)

    # 0 timeout seconds implies no timeout
    # TODO rn there is nothing for timeout_sec, would be a great first issue for someone :) (use self.node.get_clock().now())
    # _=0 should be timeout_sec = 0
    def send_goal_and_block_until_done(self, goal, _=0):
        future = self.ac.send_goal_async(goal)  # rn no feedback
        running = True
        while running:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if future.done():
                result = future.result()
                result_future = result.get_result_async()

                while not result_future.done():  # TODO this could be better
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                running = False

    def send_goal_and_return_future(self, goal):
        return self.ac.send_goal_async(goal)


class StartGateNode(Node):
    def __init__(self):
        super().__init__("startgate")

        # Action server
        self._action_server = ActionServer(
            self,
            StartGate,
            "startgate",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.move_client = ActionUser(self, Move, "move")

        self.imu_sub = self.create_subscription(
            Imu,
            "imu/data",
            self.imu_cb,
            10,
        )
        self.last_imu = Imu()
        self.heard_imu = False  # TODO del meee??

    def sleep_for(self, time: float):
        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=time)

        while (self.get_clock().now() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)  # Allow callbacks while waiting

    def imu_cb(self, msg: Imu):
        self.last_imu = msg
        self.heard_imu = True

    def goal_callback(self, goal_request):
        self.goal_x = goal_request.x
        self.goal_y = goal_request.y
        self.goal_z = goal_request.z
        self.goal_w = goal_request.w
        self.get_logger().info("goal to do start gate")
        return GoalResponse.ACCEPT

    def cancel_callback(self, _):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: StartGate.Goal):
        while not self.heard_imu:
            self.sleep_for(0.5)
        self.heard_imu = False

        current_imu = Quaternion()
        current_imu.x = self.last_imu.orientation.x
        current_imu.y = self.last_imu.orientation.y
        current_imu.z = self.last_imu.orientation.z
        current_imu.w = self.last_imu.orientation.w

        goal_quat = Quaternion()
        goal_quat.x = self.goal_x
        goal_quat.y = self.goal_y
        goal_quat.z = self.goal_z
        goal_quat.w = self.goal_w

        q1 = R.from_quat(
            [current_imu.x, current_imu.y, current_imu.z, current_imu.w],
        )  # Starting orientation
        q2 = R.from_quat(
            [self.goal_x, self.goal_y, self.goal_z, self.goal_w],
        )  # Target orientation

        q_delta = q2 * q1.inv()  # Rotation that transforms q1 to q2

        print(q_delta.as_quat())  # Outputs (x, y, z, w)
        (x, y, z, w) = q_delta.as_quat()  # Outputs (x, y, z, w)

        # move to look at the abs goal (this will send us to xyz=000
        looking_at_gate = Pose()
        looking_at_gate.orientation.x = x
        looking_at_gate.orientation.y = y
        looking_at_gate.orientation.z = z
        looking_at_gate.orientation.w = w

        goal = Move.Goal()
        goal.type = "Relative"
        goal.goal_pose = looking_at_gate
        self.move_client.send_goal_and_block_until_done(goal)
        self.sleep_for(0.5)

        # goal_handle.succeed()
        result = StartGate.Result()
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)
    node = StartGateNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
