#!/usr/bin/env python3
import math
from typing import List

import rclpy
from geometry_msgs.msg import Pose
from mil_msgs.msg import PerceptionTargetArray
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from subjugator_msgs.action import Move
from subjugator_msgs.action import NavigateChannel as NavigateChannelAction

IMAGE_W = 960
CX = IMAGE_W // 2
HFOV_RAD = math.radians(50.0)

SURGE_STEP = 0.40  # normal stride
SURGE_STEP_SMALL = 0.35  # closing on small poles
NO_POLES_SURGE = 2.00  # surge when no poles ever seen

SEARCH_YAW_STEP = 0.35
Kp_LIMIT = math.radians(50)

MIN_AREA_CLOSE = 16000  # big pole threshold to avoid getting too close
MIN_AREA_DETECT = 400  # smallest we consider "real"
MIN_AREA_STEER = 1500  # minimum area to use pole for steering

OFFSET_PX = 100  # INCREASED: lateral offset (pixels) for safer clearance
LOST_TIMEOUT = 4.0  # seconds before exit scan
LOOK_OFFSET_RAD = math.radians(20)
LOOK_DWELL_SEC = 0.3  # seconds to pause when scanning
HEADING_EPS = 0.05  # rad tolerance for yaw_to

IDLE_SEC = 0.02  # loop breath

# small-poles gains to keep turning even when far
SMALL_TAU_GAIN = 2.4
MIN_YAW_SMALL = math.radians(10)  # floor so it doesn't sit at ~0


def px2yaw(px_err: float) -> float:
    """+px means gap centre to right → need CW (-yaw)."""
    return -px_err * HFOV_RAD / IMAGE_W


class NavigateChannelServer(Node):
    def __init__(self) -> None:
        super().__init__("navigation_channel_server")

        self._as = ActionServer(
            self,
            NavigateChannelAction,
            "navigatechannel",
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )

        self._move = ActionClient(self, Move, "move")

        self._detections: List = []
        self.create_subscription(
            PerceptionTargetArray,
            "/perception/targets",
            lambda msg: setattr(self, "_detections", list(msg.targets)),
            qos_profile=30,
        )

        self._ever_seen = False
        self._ever_seen_steerable = False
        self._lost_since = None

    async def _sleep(self, sec=IDLE_SEC):
        fut = rclpy.task.Future()
        self.create_timer(sec, lambda: fut.set_result(True))
        await fut

    async def move_rel(self, surge, yaw):
        pose = Pose()
        pose.position.x = surge
        pose.position.y = 0.0
        pose.position.z = 0.0

        # Create quaternion from yaw
        q = R.from_euler("z", yaw).as_quat()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        goal = Move.Goal(type="Relative", goal_pose=pose)
        self.get_logger().info(
            f"Sending move command: surge={surge:.2f}m, yaw={yaw:.3f}rad",
        )
        self.get_logger().debug(
            f"Full pose: pos=({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})",
        )

        gh = await self._move.send_goal_async(goal)
        result = await gh.get_result_async()
        if result and result.result.success:
            self.get_logger().info("Move completed successfully")
            return True
        else:
            self.get_logger().warning(
                f"Move failed: {result.result.message if result else 'No result'}",
            )
            return False

    # Look left and right for object, return true if found
    async def inspection(self, inspection_rad, precision=3):
        self.get_logger().info("Starting scanning area")

        step = inspection_rad / max(1, precision)

        # Look left first
        self.get_logger().info("Looking left...")
        for _ in range(precision):
            success = await self.move_rel(0.0, step)
            if success:
                await self._sleep(LOOK_DWELL_SEC)
                if self._small() or self._big():
                    self.get_logger().info("Found poles on left side")
                    return True

        # Back to center
        self.get_logger().info("TESTING1!!!!!")
        await self.move_rel(0.0, -inspection_rad)
        self.get_logger().info("TESTING2!!!!!")

        # Look right from center
        self.get_logger().info("Looking right...")
        for _ in range(precision):

            success = await self.move_rel(0.0, -step)
            if success:
                await self._sleep(LOOK_DWELL_SEC)
                if self._small() or self._big():
                    self.get_logger().info("Found poles on right side")
                    return True

        # Back to center
        await self.move_rel(0.0, inspection_rad)

        self.get_logger().info("No poles found in inspection")
        return False

    def _filter(self, label, min_area) -> List:
        return [
            d
            for d in self._detections
            if d.label == label and d.width * d.height > min_area
        ]

    def _big(self):
        return self._filter("red-pole", MIN_AREA_CLOSE) + self._filter(
            "white-pole",
            MIN_AREA_CLOSE,
        )

    def _small(self):
        return self._filter("red-pole", MIN_AREA_DETECT) + self._filter(
            "white-pole",
            MIN_AREA_DETECT,
        )

    def _log_areas(self):
        if self._detections:
            areas = ", ".join(
                f"{d.label}:{d.width*d.height:.0f}" for d in self._detections
            )
            self.get_logger().info(f"Pole areas → {areas}")

    def inRangeWhite(self, objectCenter, imageWidth=960, acceptanceWidth=6):
        return objectCenter >= imageWidth / acceptanceWidth

    def inRangeRed(self, objectCenter, imageWidth=960, acceptanceWidth=6):
        return objectCenter <= (acceptanceWidth - 1) * (imageWidth / acceptanceWidth)

    @staticmethod
    def clamp01(x: float) -> float:
        return 0.0 if x <= 0.0 else 1.0 if x >= 1.0 else x

    @staticmethod
    def smoothstep01(t: float) -> float:
        t = NavigateChannelServer.clamp01(t)
        return t * t * (3.0 - 2.0 * t)

    def area_weight(self, area: float, a0: float = 400.0, a1: float = 8000.0) -> float:
        t = (area - a0) / (a1 - a0)
        t = max(0.0, min(1.0, t))  # Clamp to [0, 1]
        # Exponential curve: rises quickly then levels off
        weight = min(1.3, (1 - math.exp(-3 * t)) * 1.45)
        self.get_logger().info(f"Area {area:.0f} → weight: {weight:.3f}")
        return weight

    @staticmethod
    def encroachment_weight(
        label: str,
        cx: float,
        image_w: int = 960,
        deadband_px: float = 0,
        overshoot_gain: float = 1.2,
        overshoot_exp: float = 1.6,
    ) -> float:
        cx_mid = image_w * 0.5
        half = image_w * 0.5

        if label == "red-pole":
            on_correct_side = cx >= cx_mid
            dist_from_center = abs(cx - cx_mid)
        else:
            on_correct_side = cx <= cx_mid
            dist_from_center = abs(cx - cx_mid)

        if on_correct_side:
            # Linear response, not smoothstep!
            norm_dist = dist_from_center / half
            return 1.0 - (norm_dist * 0.5)  # Only reduce by half at edge
        else:
            # Wrong side - stronger response
            norm_dist = dist_from_center / half
            return 1.0 + overshoot_gain * norm_dist

    async def final_inspection(self, inspection_rad, precision=3):

        self.get_logger().info("Starting FINAL inspection (will return to center)")

        step = inspection_rad / max(1, precision)
        found_any = False

        # Look left
        self.get_logger().info("Final inspection: Looking left...")
        for i in range(precision):
            success = await self.move_rel(0.0, step)
            if success:
                await self._sleep(LOOK_DWELL_SEC)
                if (self._small() or self._big()) and not found_any:
                    found_any = True
                    self.get_logger().info("Final inspection: Found poles on left")

        # return to center from left
        await self.move_rel(0.0, -inspection_rad)

        # Look right
        self.get_logger().info("Final inspection: Looking right...")
        for i in range(precision):
            success = await self.move_rel(0.0, -step)
            if success:
                await self._sleep(LOOK_DWELL_SEC)
                if (self._small() or self._big()) and not found_any:
                    found_any = True
                    self.get_logger().info("Final inspection: Found poles on right")

        # ALWAYS return to center from right
        await self.move_rel(0.0, inspection_rad)

        self.get_logger().info(f"Final inspection complete. Found poles: {found_any}")
        return found_any

    async def _execute(self, gh, absenceSurge=1.5, imageWidth=960):
        if not self._move.wait_for_server(timeout_sec=5.0):
            gh.abort()
            return NavigateChannelAction.Result(
                success=False,
                message="move server unavailable",
            )

        self._ever_seen = False
        self._lost_since = self.get_clock().now()

        while rclpy.ok():
            # current detections
            real = [
                d for d in self._detections if d.width * d.height >= MIN_AREA_DETECT
            ]
            big = [d for d in real if d.width * d.height >= MIN_AREA_CLOSE]
            steerable = [d for d in real if d.width * d.height >= MIN_AREA_STEER]
            total = len(real)
            self._log_areas()

            # Initial search pattern - keep going until we find poles
            if not self._ever_seen and total == 0:
                self.get_logger().info("=== SEARCH PATTERN: No poles ever seen ===")
                self.get_logger().info(f"Moving forward {absenceSurge}m...")
                await self.move_rel(absenceSurge, 0.0)
                found = await self.inspection(math.radians(30))
                if found:
                    self._ever_seen = True
                    self._lost_since = self.get_clock().now()
                    self.get_logger().info("Poles found! Exiting search pattern")
                else:
                    self.get_logger().info(
                        "No poles found, continuing search pattern...",
                    )
                continue

            # First sighting or re-sighting
            if total > 0:
                if not self._ever_seen:
                    self.get_logger().info("FIRST POLE SIGHTING!")
                self._ever_seen = True
                if len(steerable) > 0:
                    self._ever_seen_steerable = True
                self._lost_since = self.get_clock().now()

            # Lost after seen, how we say the task is finished
            if self._ever_seen_steerable and total == 0:
                elapsed = (self.get_clock().now() - self._lost_since).nanoseconds * 1e-9
                self.get_logger().info(
                    f"Poles lost for {elapsed:.1f}s (timeout={LOST_TIMEOUT}s)",
                )

                if elapsed > LOST_TIMEOUT:
                    self.get_logger().info(
                        "Lost timeout reached, performing FINAL scan",
                    )

                    # First, move forward a bit to see if we just need to advance
                    await self.move_rel(0.5, 0.0)  # Small forward move
                    await self._sleep(0.5)  # Give time for detection

                    # Check if we can see poles now
                    if (
                        len(
                            [
                                d
                                for d in self._detections
                                if d.width * d.height >= MIN_AREA_DETECT
                            ],
                        )
                        > 0
                    ):
                        self.get_logger().info(
                            "Found poles after moving forward, continuing navigation",
                        )
                        self._lost_since = self.get_clock().now()
                        continue

                    # If still no poles, final inspection time
                    found = await self.final_inspection(math.radians(30))

                    # Most likely means we have not completely passed through the channel
                    if found:
                        self.get_logger().info(
                            "Poles detected in final scan, pushing through straight",
                        )
                        await self.move_rel(1.0, 0.0)
                        self._lost_since = self.get_clock().now()
                    else:
                        # We're done!
                        self.get_logger().info(
                            "No poles found in final scan, channel cleared!",
                        )
                        gh.succeed()
                        return NavigateChannelAction.Result(
                            success=True,
                            message="channel cleared",
                        )
                else:
                    # Before timeout, just go straight (don't rotate looking for poles)
                    self.get_logger().info("Moving straight while waiting for timeout")
                    await self.move_rel(absenceSurge, 0.0)
                continue

            # too close protection
            if len(big) > 0:
                max_pole = max(big, key=lambda d: d.width * d.height)
                max_area = max_pole.width * max_pole.height
                if max_area > 6000:
                    self.get_logger().warning(
                        f"VERY close to {max_pole.label} (area={max_area:.0f}), steering away",
                    )
                    yaw_cmd = Kp_LIMIT if max_pole.label == "red-pole" else -Kp_LIMIT
                    await self.move_rel(0.0, yaw_cmd)
                    await self.move_rel(0.5, 0.0)
                    gh.publish_feedback(
                        NavigateChannelAction.Feedback(
                            distance_to_gap=float(max_pole.cx - CX),
                        ),
                    )
                    continue

            # approach small poles (original behavior)
            if not self._ever_seen_steerable and (
                len(steerable) == 0 and len(real) > 0
            ):
                # For small poles, steer TOWARD them (center them), don't push to sides
                weighted_cx_sum = 0.0
                weight_sum = 0.0

                for d in real:
                    area = float(d.width * d.height)
                    # Simple weight based on area - bigger poles matter more
                    weight = area / MIN_AREA_DETECT
                    weighted_cx_sum += d.cx * weight
                    weight_sum += weight

                if weight_sum > 0:
                    # Find the weighted center of all poles
                    avg_cx = weighted_cx_sum / weight_sum
                    # Steer to center the poles
                    err_px = (
                        avg_cx - CX
                    )  # Positive = poles to right, need to turn right
                    yaw_cmd = px2yaw(err_px)

                    # Apply minimum yaw if needed
                    if abs(yaw_cmd) < MIN_YAW_SMALL:
                        yaw_cmd = math.copysign(MIN_YAW_SMALL, yaw_cmd)
                    yaw_cmd = max(-Kp_LIMIT, min(Kp_LIMIT, yaw_cmd))
                else:
                    yaw_cmd = 0.0

                self.get_logger().info(
                    f"Approaching {len(real)} small poles: yaw={yaw_cmd:.3f}",
                )
                await self.move_rel(0.0, yaw_cmd)
                await self.move_rel(SURGE_STEP_SMALL, 0.0)
                continue

            # τ-steering using largest detection
            if len(steerable) == 0:
                self.get_logger().info(
                    "No poles large enough for steering, moving straight",
                )
                await self.move_rel(SURGE_STEP, 0.0)
                continue

            # Find the largest pole - it's our primary reference
            largest = max(steerable, key=lambda d: d.width * d.height)
            max_area = largest.width * largest.height

            self.get_logger().info(
                f"Using largest pole: {largest.label} at cx={largest.cx}, area={max_area:.0f}",
            )

            # Calculate weights
            area = float(largest.width * largest.height)
            w_a = self.area_weight(area, a0=MIN_AREA_STEER, a1=MIN_AREA_CLOSE)
            w_c = self.encroachment_weight(
                largest.label,
                largest.cx,
                image_w=IMAGE_W,
                deadband_px=(0),
                overshoot_gain=1.0,
                overshoot_exp=0.9,
            )
            tau = w_a * w_c

            # CORRECTED: Steer to keep poles on their proper sides
            # Red poles belong on the RIGHT - always turn LEFT (positive yaw)
            # The further left the pole, the stronger we turn left
            # White poles belong on the LEFT - always turn RIGHT (negative yaw)
            # The further right the pole, the stronger we turn right
            err_px = CX - largest.cx if largest.label == "red-pole" else largest.cx - CX
            # Ensure err_px is positive

            yaw_cmd = px2yaw(err_px) * tau

            # Apply limits
            yaw_cmd = max(-Kp_LIMIT, min(Kp_LIMIT, yaw_cmd))

            # Reduce forward speed when poles are large (we're close)
            surge = SURGE_STEP if max_area <= MIN_AREA_CLOSE else SURGE_STEP_SMALL * 0.9

            self.get_logger().info(
                f"Steering based on largest {largest.label}: tau={tau:.2f}, yaw={yaw_cmd:.3f}, surge={surge:.2f}",
            )
            await self.move_rel(0.0, yaw_cmd)
            await self.move_rel(surge, 0.0)
            gh.publish_feedback(
                NavigateChannelAction.Feedback(
                    distance_to_gap=float(err_px if tau > 1e-6 else 0.0),
                ),
            )
            continue

        gh.abort()
        return NavigateChannelAction.Result(success=False, message="node shutdown")


def main() -> None:
    rclpy.init()
    rclpy.spin(NavigateChannelServer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()


"""
- If we see clumpse of poles that are too far(small) go to the average point of all those poles
- Once we see poles that are above the threshold size stop and begin the nav channel logic

- If we see red on our left we yaw right and go straight
- If we see white on our right we yaw left and go straight
    - Amount of yaw could be a function of distance(area) and position on screen
- If we only see a red pole and it is on our right, go straight
- If we only see a white pole and it is on our left, go straight
    - Need to make center range
    - Each point in the range will need to take into account size of area
"""
