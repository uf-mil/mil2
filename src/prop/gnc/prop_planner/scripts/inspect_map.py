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
with the hit count each one has and whether it is confirmed. An entry stuck
below min_hits is being seen but not consistently enough to plan around, which
is a different fault from not being seen at all.
"""

import argparse
import sys
import time

from rclpy import init, shutdown, spin_once
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class Inspector(Node):
    def __init__(self):
        super().__init__("inspect_map")

        self.in_view = 0
        self.tracks_seen = False
        self.obstacles = {}  # (namespace, id) -> (x, y, radius, hits)
        self.map_seen = False
        self.last_print = time.monotonic()

        self.create_subscription(MarkerArray, "tracked_markers", self.on_tracks, 10)
        self.create_subscription(MarkerArray, "obstacle_map", self.on_map, 10)

    def on_tracks(self, msg):
        self.tracks_seen = True
        self.in_view = sum(
            1 for m in msg.markers if m.type == Marker.CUBE and m.action == Marker.ADD
        )

    def on_map(self, msg):
        self.map_seen = True
        # The planner leads with a DELETEALL, so each message is the whole map.
        discs, hits = {}, {}
        for m in msg.markers:
            if m.action != Marker.ADD:
                continue
            if m.type == Marker.CYLINDER:
                discs[(m.ns, m.id)] = (
                    m.pose.position.x,
                    m.pose.position.y,
                    m.scale.x / 2.0,
                )
            elif m.type == Marker.TEXT_VIEW_FACING:
                hits[(m.ns.removesuffix("_hits"), m.id)] = int(m.text)
        self.obstacles = {
            key: (*value, hits.get(key, 0)) for key, value in discs.items()
        }

    def confirmed(self):
        return {
            key: value for key, value in self.obstacles.items() if key[0] == "confirmed"
        }

    def missing(self):
        absent = []
        if not self.tracks_seen:
            absent.append("tracked_markers (is the pcd stack running?)")
        if not self.map_seen:
            absent.append("obstacle_map (is the planner running?)")
        return absent


def counters(node):
    """Print one line a second: what is visible now against what has been kept."""
    print(f"{'time':>8}  {'in view':>8}  {'known':>6}  {'confirmed':>10}")
    while True:
        spin_once(node, timeout_sec=0.1)
        if time.monotonic() - node.last_print < 1.0:
            continue
        node.last_print = time.monotonic()

        absent = node.missing()
        if absent:
            for note in absent:
                print(f"  waiting for {note}")
            continue

        print(
            f"{time.strftime('%H:%M:%S'):>8}  {node.in_view:>8}  "
            f"{len(node.obstacles):>6}  {len(node.confirmed()):>10}",
        )


def listing(node):
    """Print the remembered obstacles themselves, once the map has arrived."""
    deadline = time.monotonic() + 10.0
    while not node.map_seen and time.monotonic() < deadline:
        spin_once(node, timeout_sec=0.1)

    if not node.map_seen:
        raise SystemExit("nothing on obstacle_map after 10s. Is the planner running?")

    if not node.obstacles:
        print("the map is empty: no detection has reached the planner yet.")
        print("Check that tracked_markers carries boxes, and that TF can reach")
        print("the map frame from the frame those boxes are stamped in.")
        return

    print(f"{'x':>9}  {'y':>9}  {'radius':>7}  {'hits':>5}  state")
    for key, (x, y, radius, hits) in sorted(
        node.obstacles.items(),
        key=lambda kv: -kv[1][3],
    ):
        print(f"{x:>9.2f}  {y:>9.2f}  {radius:>7.2f}  {hits:>5}  {key[0]}")


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
