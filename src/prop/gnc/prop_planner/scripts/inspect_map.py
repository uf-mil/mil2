#!/usr/bin/env python3
"""
Watch what the lidar can see against what the planner remembers.

    ros2 run prop_planner inspect_map.py

Two numbers matter, side by side. "in view" is how many boxes pcl_tracker is
publishing this instant, which is only what is in the beam right now. "known"
is how many obstacles the planner has kept, which should only ever grow as the
boat covers ground.

That is the test for whether the memory works: point the boat away from a buoy,
or drive past it until it drops under the lidar's bottom beam. "in view" falls
and "known" does not. If they fall together, nothing is being remembered.

    ros2 run prop_planner inspect_map.py --list

prints the remembered obstacles one per line instead of the running counters,
with its axis length, radius, heading and hit count. An entry stuck below
min_hits is being seen but not consistently enough to plan around, which is a
different fault from not being seen at all. A length of zero means a round
obstacle; anything longer is a capsule, and the heading is where it points.
"""

import argparse
import math
import sys
import time

from mil_msgs.msg import PerceptionObjectArray
from rclpy import init, shutdown, spin_once
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class Inspector(Node):
    def __init__(self):
        super().__init__("inspect_map")

        self.in_view = 0
        self.tracks_seen = False
        self.obstacles = []  # one entry per remembered obstacle
        self.map_seen = False
        self.last_print = time.monotonic()

        self.create_subscription(MarkerArray, "tracked_markers", self.on_tracks, 10)
        # The map as data, not as the MarkerArray that draws it. A capsule
        # renders as two end caps and a body, so counting markers counts shapes
        # rather than obstacles, and every change to the drawing would break
        # whatever tried to read it back.
        self.create_subscription(PerceptionObjectArray, "obstacles", self.on_map, 10)

    def on_tracks(self, msg):
        self.tracks_seen = True
        self.in_view = sum(
            1 for m in msg.markers if m.type == Marker.CUBE and m.action == Marker.ADD
        )

    def on_map(self, msg):
        self.map_seen = True
        self.obstacles = [
            {
                "x": o.pose.position.x,
                "y": o.pose.position.y,
                "length": o.scale.x,
                "radius": 0.5 * o.scale.y,
                "yaw": math.degrees(
                    2.0 * math.atan2(o.pose.orientation.z, o.pose.orientation.w),
                ),
                "hits": int(o.confidence),
                "state": o.labeled_classification,
            }
            for o in msg.objects
        ]

    def confirmed(self):
        return [o for o in self.obstacles if o["state"] == "confirmed"]

    def missing(self):
        absent = []
        if not self.tracks_seen:
            absent.append("tracked_markers (is the pcd stack running?)")
        if not self.map_seen:
            absent.append("obstacles (is the planner running?)")
        return absent


def counters(node):
    """Print one line a second: what is visible now against what has been kept."""
    print(f"{'time':>8}  {'in view':>8}  {'known':>6}  {'confirmed':>10}", flush=True)
    while True:
        spin_once(node, timeout_sec=0.1)
        if time.monotonic() - node.last_print < 1.0:
            continue
        node.last_print = time.monotonic()

        absent = node.missing()
        if absent:
            for note in absent:
                print(f"  waiting for {note}", flush=True)
            continue

        print(
            f"{time.strftime('%H:%M:%S'):>8}  {node.in_view:>8}  "
            f"{len(node.obstacles):>6}  {len(node.confirmed()):>10}",
            flush=True,
        )


def listing(node):
    """Print the remembered obstacles themselves, once the map has arrived."""
    deadline = time.monotonic() + 10.0
    while not node.map_seen and time.monotonic() < deadline:
        spin_once(node, timeout_sec=0.1)

    if not node.map_seen:
        raise SystemExit("nothing on obstacles after 10s. Is the planner running?")

    if not node.obstacles:
        print("the map is empty: no detection has reached the planner yet.")
        print("Check that tracked_markers carries boxes, and that TF can reach")
        print("the map frame from the frame those boxes are stamped in.")
        return

    print(
        f"{'x':>8}  {'y':>8}  {'length':>7}  {'radius':>7}  "
        f"{'heading':>8}  {'hits':>5}  state",
    )
    for o in sorted(node.obstacles, key=lambda o: -o["hits"]):
        # A zero length axis is a round obstacle, where the heading means
        # nothing and printing one would only invite reading into it.
        heading = f"{o['yaw']:>8.0f}" if o["length"] > 1e-6 else "   round"
        print(
            f"{o['x']:>8.2f}  {o['y']:>8.2f}  {o['length']:>7.2f}  "
            f"{o['radius']:>7.2f}  {heading}  {o['hits']:>5}  {o['state']}",
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--list",
        action="store_true",
        help="print the remembered obstacles instead of the running counters",
    )
    args = parser.parse_args(sys.argv[1:])

    init()
    node = Inspector()
    try:
        listing(node) if args.list else counters(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        shutdown()


if __name__ == "__main__":
    main()
