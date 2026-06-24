import rclpy
from dataclasses import dataclass
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from yolo_msgs.msg import DetectionArray
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

    def get(self):
        return self.msg

    def reset(self):
        self.msg = None

odom_sub = AwaitableSubscription(Odometry, "/odometry/filtered")
yolo_sub = AwaitableSubscription(DetectionArray, "/yolo/detections")
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
            if msg := t.wait.get():
                try:
                    t.wait = t.co.send(msg)
                except StopIteration:
                    t.co = None
        # clear out finished tasks
        new_tasks = []
        for t in tasks:
            if t.co is None:
                continue
            t.wait.reset()
            new_tasks.append(t)
        tasks = new_tasks
    rclpy.shutdown()
