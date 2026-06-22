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

def run(mission):
    co = mission.send(None)
    tasks.append(co)
    while True:
        rclpy.spin_once(node)
        for task in tasks:
            if task.msg:
                msg = task.msg
                task.msg = None
                mission.send(msg)
    rclpy.shutdown()
