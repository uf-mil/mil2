from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion

class ConverterNode(Node):
    def __init__(self):
        super().__init__("converter_node")
        self.publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        self.sub = self.create_subscription(Pose, "/goal/trajectory", self.goal_cb, 10)
        self.latest:Pose = Pose()

    def goal_cb(self, msg:Pose):
        self.latest = msg

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "orientation"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = msg.position.x
        marker.pose.position.y = msg.position.y
        marker.pose.position.z = msg.position.z

        # Orientation: rotate 90 degrees around Z (yaw = π/2)
        marker.pose.orientation = Quaternion(x=msg.orientation.x, y=msg.orientation.y, z=msg.orientation.z, w=msg.orientation.w)

        marker.scale.x = 1.0  # Arrow length
        marker.scale.y = 0.1  # Arrow width
        marker.scale.z = 0.1  # Arrow height

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.publisher.publish(marker)

def main():
    rclpy.init()
    my_node = ConverterNode()
    rclpy.spin(my_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
