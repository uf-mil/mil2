#!/usr/bin/env python3

"""
Move the sub through a sequence of poses via /goal/trajectory.

Publishes each pose directly to the PID controller and waits until
the sub reaches it (within tolerance) before advancing to the next.

Usage:
  python3 move_poses.py --poses 0,0,-1,0  2,0,-1,0  2,2,-1,90
  python3 move_poses.py --file poses.yaml
  python3 move_poses.py --tolerance 0.3 --timeout 30 --poses 0,0,-1,0

Pose format: x,y,z,yaw_deg

YAML file format:
  poses:
    - {x: 0.0, y: 0.0, z: -1.0, yaw_deg: 0.0}
    - {x: 2.0, y: 0.0, z: -1.0, yaw_deg: 90.0}
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _yaw_to_quat(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw_rad / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    return q


def _make_pose(x: float, y: float, z: float, yaw_deg: float) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation = _yaw_to_quat(math.radians(yaw_deg))
    return p


def _distance(a: Pose, b: Pose) -> float:
    dx = a.position.x - b.position.x
    dy = a.position.y - b.position.y
    dz = a.position.z - b.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class MovePoses(Node):
    def __init__(self, poses: list[Pose], tolerance: float, timeout: float):
        super().__init__("move_poses")
        self.poses = poses
        self.tolerance = tolerance
        self.timeout = timeout
        self.current_pose: Pose | None = None

        self.goal_pub = self.create_publisher(Pose, "/goal/trajectory", 1)
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self._odom_cb, 10
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose

    def run(self) -> None:
        print("Waiting for odometry...")
        while rclpy.ok() and self.current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        print(
            f"Moving through {len(self.poses)} pose(s) "
            f"[tolerance={self.tolerance}m  timeout={self.timeout}s]"
        )

        for i, pose in enumerate(self.poses):
            p = pose.position
            print(f"\n[{i + 1}/{len(self.poses)}] Goal → x={p.x:.2f}  y={p.y:.2f}  z={p.z:.2f}")
            self.goal_pub.publish(pose)

            start = time.monotonic()
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                dist = _distance(self.current_pose, pose)
                elapsed = time.monotonic() - start

                if dist < self.tolerance:
                    print(f"  Reached in {elapsed:.1f}s  (dist={dist:.3f}m)")
                    break

                if elapsed > self.timeout:
                    print(f"  Timeout after {elapsed:.1f}s  (dist={dist:.3f}m) — advancing anyway")
                    break

        print("\nDone.")


def _parse_pose(s: str) -> Pose:
    parts = s.split(",")
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(f"Expected x,y,z,yaw_deg — got: {s!r}")
    try:
        x, y, z, yaw_deg = map(float, parts)
    except ValueError as e:
        raise argparse.ArgumentTypeError(str(e))
    return _make_pose(x, y, z, yaw_deg)


def _load_yaml(path: str) -> list[Pose]:
    import yaml

    with open(path) as f:
        data = yaml.safe_load(f)
    return [
        _make_pose(
            float(e["x"]), float(e["y"]), float(e["z"]), float(e.get("yaw_deg", 0.0))
        )
        for e in data["poses"]
    ]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Move sub through a sequence of poses via /goal/trajectory",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Example: python3 move_poses.py --poses 0,0,-1,0  2,0,-1,90",
    )
    parser.add_argument(
        "--poses", nargs="+", type=_parse_pose, metavar="x,y,z,yaw_deg"
    )
    parser.add_argument("--file", metavar="PATH", help="YAML file with pose list")
    parser.add_argument("--tolerance", type=float, default=0.3, help="Goal tolerance in meters (default: 0.3)")
    parser.add_argument("--timeout", type=float, default=60.0, help="Per-pose timeout in seconds (default: 60)")

    args, _ = parser.parse_known_args()

    if args.poses:
        poses = args.poses
    elif args.file:
        poses = _load_yaml(args.file)
    else:
        parser.error("Provide --poses or --file")

    rclpy.init()
    node = MovePoses(poses, args.tolerance, args.timeout)
    try:
        node.run()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
