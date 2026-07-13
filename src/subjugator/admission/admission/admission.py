import asyncio
import importlib
from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped, Wrench
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from yolo_msgs.msg import DetectionArray
from subjugator_msgs.srv import Servo

rclpy.init()
node = Node("admission")
executor = rclpy.executors.MultiThreadedExecutor()
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
yolo_sub = Sub(DetectionArray, "/yolo/tracking")
yolo_down_sub = Sub(DetectionArray, "/yolo/down_tracking")
frontcam_sub = Sub(Image, "/front_cam/image_raw")

goal_pub = node.create_publisher(Pose, "/goal_pose", 10)
add_wrench_pub = node.create_publisher(Wrench, "/add_wrench", 10)
debug_pub = node.create_publisher(Image, "/debug_img", 10)

dropper_srv = node.create_client(Servo, "/dropper")

# republish for rviz
goal_pub_publish = goal_pub.publish
goal_stamped_pub = node.create_publisher(PoseStamped, "/goal_pose_stamped", 10)
def publish_stamped(pose):
    goal_pub_publish(pose)

    stamped = PoseStamped()
    stamped.pose = pose
    stamped.header.frame_id = "odom"
    goal_stamped_pub.publish(stamped)
goal_pub.publish = publish_stamped

marker_pub = node.create_publisher(Marker, "/markers", 10)

class Join:
    def __init__(self, *subs):
        self.subs = subs
        self.futs = {s(): i for i, s in enumerate(subs)}
        self.done = deque()

    def __aiter__(self):
        return self

    async def __anext__(self):
        if self.done:
            return self.done.popleft()

        done, _ = await asyncio.wait(
            self.futs.keys(),
            return_when=asyncio.FIRST_COMPLETED
        )

        for fut in done:
            i = self.futs.pop(fut)
            lst = [None] * len(self.subs)
            lst[i] = fut.result()
            self.done.append(lst)
            self.futs[self.subs[i]()] = i

        return self.done.popleft()

def fut(ros_fut):
    f = loop.create_future()
    def cb(msg):
        f.set_result(msg)
    ros_fut.add_done_callback(cb)
    return f

should_shutdown = True

def run(co):
    task = loop.create_task(co)
    try:
        loop.run_until_complete(task)
    except KeyboardInterrupt:
        pass
    finally:
        if should_shutdown:
            if rclpy.ok():
                rclpy.shutdown()
            asyncio_shutdown()

def asyncio_shutdown():
    try:
        # Lib/asyncio/runners.py : _cancel_all_tasks
        to_cancel = asyncio.all_tasks(loop)
        if not to_cancel:
            return

        for task in to_cancel:
            task.cancel()

        loop.run_until_complete(asyncio.gather(*to_cancel, return_exceptions=True))

        for task in to_cancel:
            if task.cancelled():
                continue
            if task.exception() is not None:
                loop.call_exception_handler({
                    'message': 'unhandled exception during asyncio.run() shutdown',
                    'exception': task.exception(),
                    'task': task,
                })

        # Runner.close
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.run_until_complete(
            loop.shutdown_default_executor())
    finally:
        loop.close()
