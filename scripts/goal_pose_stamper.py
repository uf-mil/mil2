import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped

rclpy.init()
node = Node("goal_pose_stamper")
pub = node.create_publisher(PoseStamped, "/goal_pose_stamped", 10)

def goal_pose(pose):
    stamped = PoseStamped()
    stamped.header.frame_id = "odom"
    stamped.pose = pose
    pub.publish(stamped)

sub = node.create_subscription(Pose, "/goal_pose", goal_pose, 10)
rclpy.spin(node)
rclpy.shutdown()
