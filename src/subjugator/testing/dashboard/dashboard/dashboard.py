# TODO add localization, pid running

import rclpy
import json
from rclpy.node import Node
from rclpy.subscription import Subscription
import time
from collections import defaultdict, deque
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import math
from geometry_msgs.msg import PoseWithCovarianceStamped

from rich.console import Console
from rich.table import Table

class TopicHz(Node):
    def __init__(self):
        super().__init__('topic_hz')

        # publishes frequencies to a topic
        self.pub_ = self.create_publisher(String, "/dashboard/hz", 10)

        # stores subscriptions with their topic names
        self.subscriptions_: dict[str, Subscription] = {}
        # stores delta-t for each subscription to calculate Hz later
        self.timestamp_history = defaultdict(lambda: deque(maxlen=20)) # dict[str: Subscription]

        self.add_topic("/imu/data", Imu)
        self.add_topic("/dvl/odom", Odometry)
        self.add_topic("/depth/pose", PoseWithCovarianceStamped)

        # Timer to print frequencies every 2 seconds TODO del me
        self.create_timer(2.0, self.report_frequencies)
        # self.create_timer(0.5, self.print_it)

    def add_topic(self, new_topic_name: str, topic_msg_type):
        if new_topic_name in self.subscriptions_.keys():
            raise Exception(f"topic name: {new_topic_name} already listened to")

        self.subscriptions_[new_topic_name] = self.create_subscription(
            topic_msg_type,
            new_topic_name,
            self._make_callback(new_topic_name),
            10
        )

    def print_it(self, frequencies: dict[str, float]):
        # Create a Console object
        console = Console()
        console.clear()

        # Create a Table object
        table = Table(title="Topic Status Table")

        # Add columns
        table.add_column("Topic Name", style="cyan", no_wrap=True)
        table.add_column("Hz", justify="right", style="magenta")
        table.add_column("Status", style="green")

        # Add some example rows
        for name, hz in frequencies.items():
            status_emoji = ":white_check_mark:" if math.floor(hz) != 0 else ":x:"
            table.add_row(name, str(hz), status_emoji)

        # Print the table to the console
        console.print(table)

    # curry = yummy
    def _make_callback(self, topic_name):
        def callback(_):
            now = time.time()
            self.timestamp_history[topic_name].append(now)
        return callback

    def report_frequencies(self):
        frequencies: dict[str, float] = {}
        for topic, timestamps in self.timestamp_history.items():
            # check for not enough data
            if len(timestamps) < 2:
                frequencies[topic]=0
                continue

            # check for data stopped coming
            seconds_since_last_msg = abs(time.time() - timestamps[-1])
            timeout_seconds = 1
            if seconds_since_last_msg > timeout_seconds:
                frequencies[topic]=0
                continue

            # calculate Hz
            intervals = [
                t2 - t1 for t1, t2 in zip(timestamps, list(timestamps)[1:])
            ]
            avg_interval = sum(intervals) / len(intervals)
            hz = 1.0 / avg_interval if avg_interval > 0 else 0.0
            frequencies[topic]=hz
        # self.to_json_and_publish(frequencies)
        self.print_it(frequencies)

    def to_json_and_publish(self, frequencies: dict[str, float]):
        json_dict:str = json.dumps(frequencies)
        msg = String()
        msg.data = json_dict
        self.pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    monitor = TopicHz()
    rclpy.spin(monitor)
    
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
