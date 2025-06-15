# TODO add localization, pid running

# wrenches
# ping navtube, ping dvl:q

import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
import time
from collections import defaultdict, deque
from nav_msgs.msg import Odometry
from rich.align import Align
from rich.padding import Padding
from sensor_msgs.msg import Imu
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, Wrench
import subprocess

from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table

# TODO this won't work when there are two cameras...
def find_dell_camera() -> str | None:
    try:
        output = subprocess.check_output(['v4l2-ctl', '--list-devices'], text=True)
        devices = output.strip().split('\n\n')
        for device_block in devices:
            if 'Dell' in device_block:
                # print("Dell camera found:")
                return device_block
        # print("Dell camera not found.")
    except subprocess.CalledProcessError as e:
        # print("Error running v4l2-ctl:", e)
        pass
    except FileNotFoundError:
        # print("v4l2-ctl not installed.")
        pass
    return None

class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        self.console = Console()
        self.console.clear()
        self.live = Live(console=self.console, refresh_per_second=10, transient=False)
        self.live.start()

        # stores subscriptions with their topic names
        self.subscriptions_: dict[str, Subscription] = {}
        # stores delta-t for each subscription to calculate Hz later
        self.timestamp_history = defaultdict(lambda: deque(maxlen=20)) # dict[str: Subscription]

        self.add_topic("/imu/data", Imu)
        self.add_topic("/dvl/odom", Odometry)
        self.add_topic("/depth/pose", PoseWithCovarianceStamped)
        self.add_topic("/odometry/filtered", Odometry) # UNTESTED

        self.create_timer(0.1, self.print_it)

        self.wrench_sub_ = self.create_subscription(Wrench, "/cmd/wrench", self.wrench_cb, 10)
        self.most_recent_wrench: Wrench = Wrench()

    def wrench_cb(self, msg):
        self.most_recent_wrench = msg

    def add_topic(self, new_topic_name: str, topic_msg_type):
        if new_topic_name in self.subscriptions_.keys():
            raise Exception(f"topic name: {new_topic_name} already listened to")

        self.subscriptions_[new_topic_name] = self.create_subscription(
            topic_msg_type,
            new_topic_name,
            self._make_callback(new_topic_name),
            10
        )

    # print it does a bunch of stuff... 
    # 1. report topics
    # 2. is the sub launched?
    # 3. is the sub killed? (sorta hard to check)
    # 4. wrenches?
    # 5. cameras
    def print_it(self):

        topic_table = self.make_table_of_frequencies()
        camera_panel = self.make_panel_for_cam()
        wrench_table = self.make_wrench_table()

        group = Group(camera_panel, topic_table, wrench_table)

        panel = Panel.fit(group, title="SubjuGator Dashboard", border_style="green", padding=(1, 2))
        panel = Align.center(panel)
        panel = Padding(panel, (3, 0, 0, 0))

        self.live.update(panel)

    def make_wrench_table(self) -> Table:
        table = Table(title="Wrench Message", show_header=True, header_style="bold magenta")

        table.add_column("Component", justify="center", style="cyan", no_wrap=True)
        table.add_column("X", justify="right")
        table.add_column("Y", justify="right")
        table.add_column("Z", justify="right")

        # Add force and torque rows
        table.add_row("Force",
                      f"{self.most_recent_wrench.force.x:.3f}",
                      f"{self.most_recent_wrench.force.y:.3f}",
                      f"{self.most_recent_wrench.force.z:.3f}")

        table.add_row("Torque",
                      f"{self.most_recent_wrench.torque.x:.3f}",
                      f"{self.most_recent_wrench.torque.y:.3f}",
                      f"{self.most_recent_wrench.torque.z:.3f}")

        return table

    def make_panel_for_cam(self) -> Panel:
        cmd_output: None | str = find_dell_camera()

        
        panel: Panel
        if cmd_output is not None:
            panel = Panel.fit(cmd_output, title="Dell Camera Device Info", style="green")
        else:
            panel = Panel.fit("No camera", title="Dell Camera Device Info", style="red")

        return panel


    def make_table_of_frequencies(self) -> Table:
        frequencies = self.report_frequencies()

        # Create a Table object
        table = Table(title="Topic Status Table")

        # Add columns
        table.add_column("Topic Name", style="cyan", no_wrap=True)
        table.add_column("Hz", justify="right", style="magenta")
        table.add_column("Status", style="green")

        for topic_name in self.subscriptions_.keys():
            there_is_topic_data = topic_name in frequencies.keys()
            if not there_is_topic_data:
                table.add_row(topic_name, "0", ":x:")
                continue

            hz:float = frequencies[topic_name]
            status_emoji = ":white_check_mark:" if math.floor(hz) != 0 else ":x:"
            table.add_row(topic_name, str(round(hz,2)), status_emoji)
        return table

    # curry = yummy
    def _make_callback(self, topic_name):
        def callback(_):
            now = time.time()
            self.timestamp_history[topic_name].append(now)
        return callback

    def report_frequencies(self) -> dict[str, float]:
        frequencies: dict[str, float] = {}
        for topic, timestamps in self.timestamp_history.items():
            # check for not enough data
            if len(timestamps) < 2:
                frequencies[topic]=0.0
                continue

            # check for data stopped coming
            seconds_since_last_msg = abs(time.time() - timestamps[-1])
            timeout_seconds = 1
            if seconds_since_last_msg > timeout_seconds:
                frequencies[topic]=0.0
                continue

            # calculate Hz
            intervals = [
                t2 - t1 for t1, t2 in zip(timestamps, list(timestamps)[1:])
            ]
            avg_interval = sum(intervals) / len(intervals)
            hz = 1.0 / avg_interval if avg_interval > 0 else 0.0
            frequencies[topic]=hz
        return frequencies

def main(args=None):
    rclpy.init(args=args)
    
    monitor = DashboardNode()
    rclpy.spin(monitor)
    
    monitor.live.stop()
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
