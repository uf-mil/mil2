from geometry_msgs.msg import Quaternion, Vector3
import rclpy
from rclpy.node import Node
from mil_msgs.msg import ProcessedPing
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from sonar_tracker.sonar_tracker import SonarTracker

class SonarNode(Node):
    def __init__(self):
        super().__init__('sonar_node')
        
        self.pinger_sub_ = self.create_subscription(ProcessedPing, "hydrophones/solved", self.sonar_cb, 10)
        self.tracker = SonarTracker()

        self.odom_sub_ = self.create_subscription(Odometry, "odometry/filtered", self.odom_cb, 10)
        self.current_pose = Odometry()

        self.marker_pub = self.create_publisher(Marker, '/sonar/marker', 10)
        self.create_timer(10, self.timer_cb)

    def timer_cb(self):
        pinger_pose = self.tracker.triangulate()
        if pinger_pose is None:
            self.get_logger().warn("Error finding pinger!")
            return

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pinger"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(pinger_pose[0])
        marker.pose.position.y = float(pinger_pose[1])
        marker.pose.position.z = float(pinger_pose[2])

        # i love formatting
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


    def odom_cb(self, msg: Odometry):
        self.current_pose = msg

    def sonar_cb(self, msg: ProcessedPing):
        # transform ping from base_link to odom
        ping_in_odom = quaternion_rotate_vector(self.current_pose.pose.pose.orientation, msg.origin_direction_body)

        self.tracker.add_ping(
            self.current_pose.pose.pose.position.x,
            self.current_pose.pose.pose.position.y,
            self.current_pose.pose.pose.position.z,
            ping_in_odom[0],
            ping_in_odom[1],
            ping_in_odom[2],
        )

def quaternion_rotate_vector(quat: Quaternion, vector: Vector3) -> tuple[float, float, float]:
    """
        Rotate a vector by a quaternion

        In our case we rotate the ping from base_link into odom
    """
    # Extract quaternion components (assuming geometry_msgs quaternion: x, y, z, w)
    qx, qy, qz, qw = quat.x, quat.y, quat.z, quat.w
    
    # Extract vector components
    vx, vy, vz = vector.x, vector.y, vector.z
    
    # Apply quaternion rotation formula
    # v' = q * v * q_conjugate, simplified for efficiency
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    
    transformed_x = vx + qw * tx + qy * tz - qz * ty
    transformed_y = vy + qw * ty + qz * tx - qx * tz
    transformed_z = vz + qw * tz + qx * ty - qy * tx
    
    return (transformed_x, transformed_y, transformed_z)


def main(args=None):
    rclpy.init(args=args)
    node = SonarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

