#!/usr/bin/env python3
"""
NavigateChannel (pose-driven version)

High-level logic:
  • Scan (rotate) until we see:    1 red pole  +  1 white pole to its left.
  • Compute yaw error + send a small RELATIVE Move goal (surge & yaw).
  • Wait for the Move action to finish (odometry-based).
  • Repeat until the pair disappears for > row_timeout → next row.
  • After `max_rows` rows  → succeed.
"""

import math
import asyncio
from enum import IntEnum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from geometry_msgs.msg import Pose
from mil_msgs.msg import PerceptionTarget, PerceptionTargetArray
from scipy.spatial.transform import Rotation as R

from subjugator_msgs.action import NavigateChannel, Move


# ───── constants you may tune ──────────────────────────────────
IMAGE_WIDTH      = 1280
IMAGE_CX         = IMAGE_WIDTH // 2
H_FOV_DEG        = 70.0                      # camera horizontal FOV
H_FOV_RAD        = math.radians(H_FOV_DEG)
SEARCH_YAW_RATE  = 0.2                       # rad/s while spinning
FORWARD_STEP     = 0.4                       # metres per Move goal
TARGET_OFFSET_PX = -50                       # keep 50 px left of white pole
ROW_TIMEOUT_DEF  = 3.0
MAX_ROWS_DEF     = 3
Kp_YAW_LIMIT     = 0.35                      # cap yaw command (rad)

class NavState(IntEnum):
    SEARCH = auto()
    ADVANCE = auto()

# ───── helper functions ────────────────────────────────────────
def bbox_area(d) -> float:
    return d.width * d.height

def px_to_yaw(err_px: float) -> float:
    """Convert pixel offset to yaw radians (positive CCW)."""
    return err_px * H_FOV_RAD / IMAGE_WIDTH

def dur_s(dur):
    return dur.nanoseconds * 1e-9

# ───── server node ─────────────────────────────────────────────
class NavigateChannelServer(Node):
    def __init__(self):
        super().__init__("navigation_channel_server")

        # start NavigateChannel action server
        self._as = ActionServer(
            self, NavigateChannel, "navigatechannel",
            execute_callback = self.execute_cb,
            goal_callback    = lambda _: GoalResponse.ACCEPT,
            cancel_callback  = lambda _: CancelResponse.ACCEPT,
        )

        # Move action client (talks to MovementServer /move)
        self.move_client = ActionClient(self, Move, "move")

        # subscribe to YOLO detections
        self.create_subscription(PerceptionTargetArray, "/perception/targets", self.percep_cb, 30)

        self.detections = []     # ring buffer size 1

    def percep_cb(self, msg: PerceptionTargetArray):
        self.detections = list(msg.targets)

    # helper: build + send a relative Move goal, await result
    async def send_relative_move(self, surge_m: float, yaw_rad: float):
        pose = Pose()
        pose.position.x = surge_m
        q = R.from_euler('z', yaw_rad).as_quat()  # [x,y,z,w]
        pose.orientation.x, pose.orientation.y, \
        pose.orientation.z, pose.orientation.w = q

        goal = Move.Goal()
        goal.type = "Relative"
        goal.goal_pose = pose
        move_fut = self.move_client.send_goal_async(goal)
        goal_handle = await move_fut
        result = await goal_handle.get_result_async()
        return result.result.success

    # ── main coroutine – one goal execution ────────────────────
    async def execute_cb(self, goal_handle):
        g = goal_handle.request
        row_timeout = g.row_timeout  or ROW_TIMEOUT_DEF
        max_rows    = g.max_rows     or MAX_ROWS_DEF
        offset_px   = g.target_offset_px if not math.isnan(g.target_offset_px) \
                                         else TARGET_OFFSET_PX

        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Move action server not available")
            goal_handle.abort()
            return NavigateChannel.Result(success=False, message="move server absent")

        state = NavState.SEARCH
        row_idx = 0
        row_timer = None
        start_time = self.get_clock().now()

        while rclpy.ok():
            # overall timeout guard
            if dur_s(self.get_clock().now() - start_time) > g.overall_timeout:
                goal_handle.abort()
                return NavigateChannel.Result(success=False, message="overall timeout")

            # snapshot perception
            dets = self.detections[:]
            self.detections.clear()
            reds   = [d for d in dets if d.label == g.red_label]
            whites = [d for d in dets if d.label == g.white_label]

            self.get_logger().info(f"SEARCH loop: reds={len(reds)}  whites={len(whites)}")


            closest_red = max(reds, key=bbox_area,   default=None)
            left_whites = [w for w in whites if closest_red and w.cx < closest_red.cx]
            closest_wl  = max(left_whites, key=bbox_area, default=None)

            # ------------- SEARCH state -------------
            if state == NavState.SEARCH:
                if closest_red and closest_wl:
                    state = NavState.ADVANCE
                    row_timer = self.get_clock().now()
                    continue
                # rotate slowly (Twist publisher from MovementServer or another node)
                await self.send_relative_move(0.0, SEARCH_YAW_RATE * 0.5)  # small yaw step
                continue

            # ------------- ADVANCE state -------------
            if state == NavState.ADVANCE:
                if not (closest_red and closest_wl):
                    # lost the pair – check timeout
                    if (dur_s(self.get_clock().now() - row_timer)) > row_timeout:
                        row_idx += 1
                        if row_idx >= max_rows:
                            goal_handle.succeed()
                            return NavigateChannel.Result(
                                success=True,
                                message=f"cleared {row_idx} rows",
                            )
                        state = NavState.SEARCH
                    continue
                else:
                    row_timer = self.get_clock().now()

                # compute yaw error from left-white pole
                desired_px = closest_wl.cx + offset_px
                err_px     = desired_px - IMAGE_CX
                yaw_err    = px_to_yaw(err_px)
                yaw_err    = max(-Kp_YAW_LIMIT, min(Kp_YAW_LIMIT, yaw_err))

                # send relative move (surge + small yaw)
                ok = await self.send_relative_move(FORWARD_STEP, yaw_err)
                if not ok:
                    self.get_logger().warn("Move action failed; retrying")

                fb = NavigateChannel.Feedback(
                        distance_to_gap=float(err_px),
                        row_index=row_idx)
                goal_handle.publish_feedback(fb)

        # node shutting down
        goal_handle.abort()
        return NavigateChannel.Result(success=False, message="node shutdown")


# ───── main entrypoint ──────────────────────────────────────────
def main():
    rclpy.init()
    rclpy.spin(NavigateChannelServer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
