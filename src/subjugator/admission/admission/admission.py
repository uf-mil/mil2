import asyncio
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Wrench
from nav_msgs.msg import Odometry
from yolo_msgs.msg import DetectionArray

rclpy.init()
node = Node("admission")
executor = rclpy.executors.SingleThreadedExecutor()
executor.add_node(node)

class ROSSelector:
    def select(self, timeout):
        if rclpy.ok():
            executor.spin_once(timeout)

class ROSLoop(asyncio.BaseEventLoop):
    def __init__(self):
        super().__init__()
        self._selector = ROSSelector()

    def _process_events(self, events_list):
        pass

loop = ROSLoop()

class Sub:
    def __init__(self, typ, topic):
        self.sub = node.create_subscription(typ, topic, self.cb, 10)
        self.futs = []

    def cb(self, msg):
        for fut in self.futs:
            fut.set_result(msg)
        self.futs = []

    def __call__(self):
        fut = loop.create_future()
        self.futs.append(fut)
        return fut

odom_sub = Sub(Odometry, "/odometry/filtered")
yolo_sub = Sub(DetectionArray, "/yolo/detections")

goal_pub = node.create_publisher(Pose, "/goal_pose", 10)
add_wrench_pub = node.create_publisher(Wrench, "/add_wrench", 10)

def run(co):
    task = loop.create_task(co)
    try:
        loop.run_until_complete(task)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        to_cancel = asyncio.all_tasks(loop)
        if not to_cancel:
            return
        for task in to_cancel:
            task.cancel()
        loop.run_until_complete(asyncio.gather(*to_cancel, return_exceptions=True))
