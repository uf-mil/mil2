#!/usr/bin/env python3
import math
import time
from enum import Enum

import rclpy
import rclpy.duration
from geometry_msgs.msg import Pose
from mil_msgs.msg import PerceptionTarget, PerceptionTargetArray
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from subjugator_msgs.action import Move, StartGate


class StartGateState(Enum):
    SEARCHING = 1
    ALIGNING = 2
    CENTERING = 3
    PASSING_THROUGH = 4
    COMPLETE = 5


class StartGateServer(Node):
    def __init__(self):
        super().__init__("start_gate_server")
        
        # Action server
        self._action_server = ActionServer(
            self,
            StartGate,
            "start_gate",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
        )
        
        # Action client for movement
        self.move_client = ActionClient(self, Move, "move")
        
        # Subscribe to perception targets from YOLO
        self.perception_sub = self.create_subscription(
            PerceptionTargetArray,
            "/perception/targets",
            self.perception_callback,
            10,
        )
        
        # Parameters
        self.declare_parameter("rotation_direction", "left")  # "left" or "right"
        self.declare_parameter("rotation_step_degrees", 15.0)  # Degrees per rotation step
        self.declare_parameter("max_rotation_degrees", 180.0)  # Max rotation before giving up
        self.declare_parameter("camera_fov_degrees", 90.0)  # Camera field of view
        self.declare_parameter("target_offset_y", 1.5)  # How far to move forward under the gate
        self.declare_parameter("centering_threshold", 50.0)  # Pixels from center to be considered centered
        self.declare_parameter("depth_offset", -0.5)  # How much to descend to go under
        
        # Image dimensions (assuming standard camera)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        
        # State variables
        self.current_targets = []
        self.state = StartGateState.SEARCHING
        self.total_rotation = 0.0
        
        self.get_logger().info("Start Gate Server initialized")
    
    def goal_callback(self, goal_request):
        self.get_logger().info("Received start gate navigation request")
        return GoalResponse.ACCEPT
    
    def perception_callback(self, msg):
        """Store the latest perception targets"""
        self.current_targets = msg.targets
    
    def find_target(self, label):
        """Find a specific target by label"""
        for target in self.current_targets:
            if target.label.lower() == label.lower():
                return target
        return None
    
    def calculate_pixel_to_angle(self, pixel_x, image_width, fov_degrees):
        """Convert pixel position to angle offset"""
        # Normalize pixel position (-1 to 1)
        normalized_x = (pixel_x - image_width / 2) / (image_width / 2)
        # Convert to angle
        angle_rad = math.radians(fov_degrees / 2) * normalized_x
        return math.degrees(angle_rad)
    
    def move_relative(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """Execute a relative movement"""
        goal_msg = Move.Goal()
        goal_msg.type = "Relative"
        goal_msg.goal_pose = Pose()
        goal_msg.goal_pose.position.x = x
        goal_msg.goal_pose.position.y = y
        goal_msg.goal_pose.position.z = z
        
        # Convert yaw to quaternion
        rot = R.from_euler('xyz', [0, 0, math.radians(yaw)])
        quat = rot.as_quat()
        goal_msg.goal_pose.orientation.x = quat[0]
        goal_msg.goal_pose.orientation.y = quat[1]
        goal_msg.goal_pose.orientation.z = quat[2]
        goal_msg.goal_pose.orientation.w = quat[3]
        
        self.get_logger().info(f"Moving relative: x={x}, y={y}, z={z}, yaw={yaw}")
        
        # Wait for action server
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Move action server not available")
            return False
        
        # Send goal and wait for result
        future = self.move_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Move goal rejected")
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        return result_future.result().result.success
    
    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing start gate navigation")
        
        # Get parameters
        rotation_dir = self.get_parameter("rotation_direction").value
        rotation_step = self.get_parameter("rotation_step_degrees").value
        max_rotation = self.get_parameter("max_rotation_degrees").value
        fov = self.get_parameter("camera_fov_degrees").value
        image_width = self.get_parameter("image_width").value
        image_height = self.get_parameter("image_height").value
        centering_threshold = self.get_parameter("centering_threshold").value
        depth_offset = self.get_parameter("depth_offset").value
        target_offset_y = self.get_parameter("target_offset_y").value
        
        # Determine rotation direction
        rotation_multiplier = 1.0 if rotation_dir == "left" else -1.0
        
        result = StartGate.Result()
        
        # State machine for start gate navigation
        while self.state != StartGateState.COMPLETE:
            
            if self.state == StartGateState.SEARCHING:
                self.get_logger().info("SEARCHING: Looking for shark and swordfish")
                
                # Look for both targets
                shark = self.find_target("shark")
                swordfish = self.find_target("swordfish")
                
                if shark and swordfish:
                    self.get_logger().info("Found both shark and swordfish!")
                    self.state = StartGateState.ALIGNING
                else:
                    # Rotate to search
                    if abs(self.total_rotation) < max_rotation:
                        self.get_logger().info(f"Rotating {rotation_step} degrees {rotation_dir}")
                        self.move_relative(yaw=rotation_step * rotation_multiplier)
                        self.total_rotation += rotation_step
                        time.sleep(0.5)  # Wait for perception to update
                    else:
                        self.get_logger().error("Could not find both targets after full rotation")
                        goal_handle.abort()
                        result.success = False
                        result.message = "Could not find shark and swordfish"
                        return result
            
            elif self.state == StartGateState.ALIGNING:
                self.get_logger().info("ALIGNING: Positioning under the gate")
                
                shark = self.find_target("shark")
                swordfish = self.find_target("swordfish")
                
                if not shark or not swordfish:
                    self.get_logger().warn("Lost sight of targets, returning to search")
                    self.state = StartGateState.SEARCHING
                    continue
                
                # Calculate center point between shark and swordfish
                center_x = (shark.cx + swordfish.cx) / 2
                
                # Calculate angle to rotate to center the gate
                angle_offset = self.calculate_pixel_to_angle(center_x, image_width, fov)
                
                if abs(angle_offset) > 2.0:  # If more than 2 degrees off
                    self.get_logger().info(f"Rotating {angle_offset:.2f} degrees to center gate")
                    self.move_relative(yaw=angle_offset)
                    time.sleep(0.5)
                else:
                    self.get_logger().info("Gate is centered, moving to centering state")
                    self.state = StartGateState.CENTERING
            
            elif self.state == StartGateState.CENTERING:
                self.get_logger().info("CENTERING: Positioning swordfish for passage")
                
                swordfish = self.find_target("swordfish")
                
                if not swordfish:
                    self.get_logger().warn("Lost sight of swordfish, returning to search")
                    self.state = StartGateState.SEARCHING
                    continue
                
                # Calculate horizontal offset (in pixels)
                x_offset = swordfish.cx - (image_width / 2)
                
                # Check if swordfish is in upper portion of image (we want to go under it)
                # Target should be in upper 40% of image
                target_y_position = image_height * 0.3  # Upper 30% of image
                y_offset = swordfish.cy - target_y_position
                
                self.get_logger().info(f"Swordfish position: x={swordfish.cx:.1f}, y={swordfish.cy:.1f}")
                self.get_logger().info(f"Offsets: x={x_offset:.1f}, y={y_offset:.1f}")
                
                # Translate to center swordfish horizontally
                if abs(x_offset) > centering_threshold:
                    # Convert pixel offset to meters (rough approximation)
                    # Negative because -y is right, +y is left in the robot frame
                    y_translation = -x_offset * 0.002  # Scale factor for pixel to meter
                    self.get_logger().info(f"Translating y={y_translation:.3f}m to center swordfish")
                    self.move_relative(y=y_translation)
                    time.sleep(0.5)
                
                # Move forward if swordfish is too centered (we need it higher in frame)
                elif y_offset > 50:  # If swordfish is too low in frame
                    self.get_logger().info("Moving forward to position swordfish higher in frame")
                    self.move_relative(x=0.3)  # Move forward 30cm
                    time.sleep(0.5)
                
                # Descend if needed
                elif y_offset < -50:  # If swordfish is too high
                    self.get_logger().info("Descending to position correctly under gate")
                    self.move_relative(z=depth_offset)
                    time.sleep(0.5)
                
                else:
                    self.get_logger().info("Swordfish properly positioned, ready to pass through")
                    self.state = StartGateState.PASSING_THROUGH
            
            elif self.state == StartGateState.PASSING_THROUGH:
                self.get_logger().info("PASSING_THROUGH: Moving through the gate")
                
                # Move forward through the gate
                self.get_logger().info(f"Moving forward {target_offset_y}m through gate")
                success = self.move_relative(x=target_offset_y)
                
                if success:
                    self.get_logger().info("Successfully passed through start gate!")
                    self.state = StartGateState.COMPLETE
                else:
                    self.get_logger().error("Failed to move through gate")
                    goal_handle.abort()
                    result.success = False
                    result.message = "Failed to move through gate"
                    return result
            
            # Allow for goal cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Goal canceled"
                return result
            
            # Brief pause between state checks
            time.sleep(0.1)
        
        # Success
        goal_handle.succeed()
        result.success = True
        result.message = "Successfully navigated through start gate"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = StartGateServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()