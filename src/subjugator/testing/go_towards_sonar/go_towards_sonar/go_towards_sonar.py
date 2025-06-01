import rclpy
from rclpy.node import Node
from mil_msgs.msg import ProcessedPing
import numpy as np
from geometry_msgs.msg import Pose, Vector3, Quaternion, Point
from tf_transformations import quaternion_about_axis
import tf_transformations

class SonarFollower(Node):
    def __init__(self):
        super().__init__('sonar_follower')
        self.timer = self.create_timer(5, self.move_there_via_pid)  # 10 Hz
        self.sub_ = self.create_subscription(ProcessedPing, "hydrophones/solved", self.sonar_cb, 10)
        self.pub_ = self.create_publisher(Pose, "/goal/trajectory", 10)
        self.quat = Quaternion()

    def sonar_cb(self, msg: ProcessedPing):
        direction = msg.origin_direction_body
        quat = vector3_to_quaternion(direction)
        self.quat = quat

    def move_there_via_pid(self):
        local_forward = np.array([1, 0.0, 0.0])  # Move 1m forward in local frame
        q_np = np.array([self.quat.x, self.quat.y, self.quat.z, self.quat.w])
        world_translation = tf_transformations.quaternion_multiply(
            tf_transformations.quaternion_multiply(q_np, np.append(local_forward, 0)),
            tf_transformations.quaternion_conjugate(q_np)
        )[:3]

        # Create pose
        pose = Pose()
        pose.position = Point(x=world_translation[0], y=world_translation[1], z=world_translation[2])
        pose.orientation = self.quat

        # WARNING this could send the sub to venus
        self.pub_.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = SonarFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# chat wrote this whole function
def vector3_to_quaternion(direction: Vector3) -> Quaternion:
    # Convert to numpy array
    dir_vec = np.array([direction.x, direction.y, direction.z], dtype=np.float64)

    # Normalize direction vector
    norm = np.linalg.norm(dir_vec)
    if norm == 0.0:
        raise ValueError("Direction vector must be non-zero")
    dir_vec /= norm

    # Default axis to align from (e.g., x-axis)
    default = np.array([1.0, 0.0, 0.0])

    # Compute axis of rotation (cross product)
    axis = np.cross(default, dir_vec)
    axis_norm = np.linalg.norm(axis)

    # If direction is the same as default (or exact opposite)
    if axis_norm < 1e-8:
        if np.dot(default, dir_vec) > 0:
            q = [0.0, 0.0, 0.0, 1.0]  # Identity quaternion
        else:
            # 180-degree rotation around any axis orthogonal to x (e.g., y-axis)
            q = quaternion_about_axis(np.pi, [0, 1, 0])
    else:
        axis /= axis_norm
        angle = np.arccos(np.clip(np.dot(default, dir_vec), -1.0, 1.0))
        q = quaternion_about_axis(angle, axis)

    # Convert to geometry_msgs/Quaternion
    quat_msg = Quaternion()
    quat_msg.x = q[0]
    quat_msg.y = q[1]
    quat_msg.z = q[2]
    quat_msg.w = q[3]

    return quat_msg

if __name__ == '__main__':
    main()
