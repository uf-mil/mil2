"""
Task 4: while boat_timeout has the boat for an AssistanceRequest, the missions
stand aside.

boat_timeout says what it is doing on /latlon_goal/state, latched: IDLE, or
NAVIGATING, HOLDING and RETURNING while it has the boat. A mission checks
`active` before sending goals. When boat_timeout is back where it took the boat
from, on_end is called with the seconds lost, so the mission can push its clocks
on by that much and send its goal again.
"""

from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

BUSY = ("NAVIGATING", "HOLDING", "RETURNING")
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Incident:
    def __init__(self, node, on_end):
        self.node = node
        self.on_end = on_end
        self.active = False
        self.began = None
        node.create_subscription(String, "/latlon_goal/state", self.on_state, LATCHED)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        busy = msg.data in BUSY
        if busy == self.active:
            return
        self.active = busy
        log = self.node.get_logger()
        if busy:
            self.began = self.now()
            log.info("Task 4: boat_timeout has the boat; standing aside")
            return
        lost = self.now() - self.began
        log.info(f"Task 4 over after {lost:.0f} s; carrying on")
        self.on_end(lost)
