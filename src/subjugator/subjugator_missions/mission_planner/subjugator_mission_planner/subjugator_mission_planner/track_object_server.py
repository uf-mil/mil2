from rclpy.node import Node
import rclpy
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from tf_transformations import quaternion_multiply, quaternion_from_euler

from subjugator_msgs.action import TrackObject

import time
import cv2
import numpy as np

# +---------------------+---------------------+
# |      DIRECTION      |       MOVEMENT      |
# +---------------------+---------------------+
# |         -X          |      Move Left      |
# |         +X          |      Move Right     |
# |         -Y          |       Move Up       |
# |         +Y          |      Move Down      |
# +---------------------+---------------------+
def detect_lime_green_webcam(ret, frame):
    if not ret:
        print("Error: Could not read frame.")
        return False, 0, 0, 0
        
    # Get image dimensions
    height, width = frame.shape[:2]
    image_center_x = width // 2
    image_center_y = height // 2
    
    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define lime green color range in HSV
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])
    
    # Create mask for lime green
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # Apply morphological operations to reduce noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Default return values for if notin found
    found = False
    center_x = 0.0
    center_y = 0.0
    area = 0.0
    
    # If contours found, get the largest one (assuming it's our target)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        # Only proceed if the area is significant
        if area > 100:  # Minimum area threshold
            # Calculate centroid of the contour
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:  # Avoid division by zero
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                # Convert to coordinates relative to center (-1.0 to 1.0)
                center_x = (cx - image_center_x) / (width / 2.0)
                center_y = (cy - image_center_y) / (height / 2.0)
                found = True
    
    return found, center_x, center_y, area


class TrackObjectServer(Node):
    def __init__(self):
        super().__init__("track_object_server")

        # odom sub
        self._odom_sub = self.create_subscription(Odometry, "odometry/filtered", self.odom_cb, 10)
        self.last_odom = Odometry()

        # goal pub
        self.goal_pub = self.create_publisher(Pose, "goal_pose", 10)
    
        # Action server
        self._action_server = ActionServer(
            self,
            TrackObject,
            "track_object",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def init_camera(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open camera")
            return

        time.sleep(1)

    def tear_down_camera(self):
        self.cap.release()

    def translate_to_track(self, y_offset, z_offset) -> Pose | None:
        y_movement_needed = abs(y_offset) > 0.1
        z_movement_needed = abs(z_offset) > 0.1

        no_movement_needed = not (y_movement_needed or z_movement_needed)
        if no_movement_needed:
            return None

        # negative y? move left
        delta_y = 0.3 if y_offset < 0 else -0.3
        delta_z = 0.15 if z_offset < 0 else -0.15

        p = Pose()
        p.position.x = self.last_odom.pose.pose.position.x
        p.position.y = self.last_odom.pose.pose.position.y
        p.position.z = self.last_odom.pose.pose.position.z
        p.orientation.x = self.last_odom.pose.pose.orientation.x
        p.orientation.y = self.last_odom.pose.pose.orientation.y
        p.orientation.z = self.last_odom.pose.pose.orientation.z
        p.orientation.w = self.last_odom.pose.pose.orientation.w

        p.position.y += delta_y
        p.position.z += delta_z

        return p

    def rotate_to_track(self, y_offset, z_offset) -> Pose | None:
        yaw_needed = abs(y_offset) > 0.1

        if not yaw_needed:
            return None

        # negative y? move left
        delta_yaw = 0.1 if y_offset < 0 else -0.1

        p = Pose()
        p.position.x = self.last_odom.pose.pose.position.x
        p.position.y = self.last_odom.pose.pose.position.y
        p.position.z = self.last_odom.pose.pose.position.z

        # Get current orientation as a list
        current_quat = [
            self.last_odom.pose.pose.orientation.x,
            self.last_odom.pose.pose.orientation.y,
            self.last_odom.pose.pose.orientation.z,
            self.last_odom.pose.pose.orientation.w
        ]
        
        # Create a quaternion representing the yaw increment
        # Parameters are (roll, pitch, yaw)
        yaw_quat = quaternion_from_euler(0, 0, delta_yaw)
        
        # Apply the yaw rotation to the current orientation
        new_quat = quaternion_multiply(yaw_quat, current_quat)
        
        # Set the new orientation
        p.orientation.x = new_quat[0]
        p.orientation.y = new_quat[1]
        p.orientation.z = new_quat[2]
        p.orientation.w = new_quat[3]

        return p

    def odom_cb(self, msg: Odometry):
        self.last_odom = msg

    def goal_callback(self, goal_request):
        print(goal_request.object_type)
        self.type_of_movement = goal_request.type_of_movement
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        print('hi')
        self.init_camera()

        while True:
            ret, frame = self.cap.read()
            found, y_offset, z_offset, _ = detect_lime_green_webcam(ret, frame)

            if not found:
                continue

            if self.type_of_movement == "translate":
                p = self.translate_to_track(y_offset, z_offset)
            elif self.type_of_movement == "rotate":
                p = self.rotate_to_track(y_offset, z_offset)
            else:
                print("helpme")

            if p is not None:
                self.goal_pub.publish(p)

        self.tear_down_camera()

        goal_handle.succeed()
        result = TrackObject.Result()
        result.success = True
        result.message = "todo"
        time.sleep(1.0)
        return result

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

def main():
    rclpy.init()
    node = TrackObjectServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

## below is for debugging only
def detect_lime_green_webcam_debug():
    """
    Opens webcam, detects lime green objects, shows the result,
    and returns coordinates relative to center.
    """
    # Open webcam
    cap = cv2.VideoCapture(0)  # 0 is usually the default webcam
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return False, 0, 0, 0
    
    # Wait a moment for the camera to initialize
    time.sleep(1)
    
    print("Press 'q' to quit...")
    
    while True:
        # Read a frame from webcam
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break
        
        # Get image dimensions
        height, width = frame.shape[:2]
        image_center_x = width // 2
        image_center_y = height // 2
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define lime green color range in HSV
        lower_green = np.array([35, 100, 100])  # Adjust these values for your specific lime green
        upper_green = np.array([85, 255, 255])
        
        # Create mask for lime green
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Apply morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Default return values if nothing is found
        found = False
        center_x = 0.0
        center_y = 0.0
        area = 0.0
        
        # Create a copy of the frame for visualization
        display_frame = frame.copy()
        
        # Draw crosshair at center of the image
        cv2.line(display_frame, (image_center_x - 20, image_center_y), 
                (image_center_x + 20, image_center_y), (0, 0, 255), 2)
        cv2.line(display_frame, (image_center_x, image_center_y - 20), 
                (image_center_x, image_center_y + 20), (0, 0, 255), 2)
        
        # If contours found, get the largest one (assuming it's our target)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            # Only proceed if the area is significant
            if area > 100:  # Minimum area threshold
                # Draw the contour
                cv2.drawContours(display_frame, [largest_contour], -1, (0, 255, 0), 2)
                
                # Calculate centroid of the contour
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:  # Avoid division by zero
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # Convert to coordinates relative to center (-1.0 to 1.0)
                    center_x = (cx - image_center_x) / (width / 2.0)
                    center_y = (cy - image_center_y) / (height / 2.0)
                    
                    found = True
                    
                    # Draw centroid
                    cv2.circle(display_frame, (cx, cy), 7, (255, 0, 0), -1)
                    cv2.putText(display_frame, f"({center_x:.2f}, {center_y:.2f})", 
                                (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    
                    # Draw line from center to object
                    cv2.line(display_frame, (image_center_x, image_center_y), 
                            (cx, cy), (255, 0, 0), 2)
        
        # Add text indicating detection status
        status = "Tracking: YES" if found else "Tracking: NO"
        cv2.putText(display_frame, status, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255) if not found else (0, 255, 0), 2)
        
        # Show the mask (optional)
        cv2.imshow('Mask', mask)
        
        # Show the result
        cv2.imshow('Lime Green Detector', display_frame)
        
        # Print coordinates to console
        if found:
            print(f"Object at ({center_x:.2f}, {center_y:.2f}), area: {area:.1f}")
        
        # Check for key press
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        
    # Clean up
    cap.release()
    cv2.destroyAllWindows()
    
    return found, center_x, center_y, area
