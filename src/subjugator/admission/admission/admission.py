import asyncio
from collections import deque
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

def run(co):
    task = loop.create_task(co)
    try:
        loop.run_until_complete(task)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
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
            loop.shutdown_default_executor(300))
    finally:
        loop.close()
