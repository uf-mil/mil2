import rclpy
from dataclasses import dataclass
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

rclpy.init()
node = Node("admission")
tasks = []

class AwaitableSubscription:
    def __init__(self, typ, topic):
        self.msg = None
        self.sub = node.create_subscription(typ, topic, self.cb, 10)

    def cb(self, msg):
        self.msg = msg

    def __await__(self):
        return (yield self)

odom_sub = AwaitableSubscription(Odometry, "/odometry/filtered")

goal_pub = node.create_publisher(Pose, "/goal_pose", 10)

@dataclass(slots=True)
class Task:
    co: any
    wait: any

def run(co):
    global tasks
    tasks = [Task(co, co.send(None))]
    while tasks:
        rclpy.spin_once(node)
        for t in tasks:
            if t.wait.msg:
                try:
                    t.wait = t.co.send(t.wait.msg)
                except StopIteration:
                    t.co = None
        # clear out finished tasks
        new_tasks = []
        for t in tasks:
            if t.co is None:
                continue
            t.wait.msg = None
            new_tasks.append(t)
        tasks = new_tasks
    rclpy.shutdown()
