import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty

from subjugator_waypoints import wp_manager
from subjugator_msgs.srv import StringTrigger

class WpNode(Node):
    def __init__(self):
        super().__init__("wp_node")
        self.wps = wp_manager.WaypointManager()

        self.reset_sub_ = self.create_subscription(Empty,  "localization/was_reset", self.reset_cb, 10)

        self.odom_sub_ = self.create_subscription(Odometry, "odometry/filtered", self.odom_cb, 10)
        self.current_pose: Odometry

        self.goal_pose_pub_ = self.create_publisher(Pose, "/goal_pose", 10)

        self.srv_ = self.create_service(StringTrigger, 'wp/set', self.set_wp)
        self.srv_ = self.create_service(StringTrigger, 'wp/goto', self.goto_wp)

    def set_wp(self, req, res):
        self.wps.add_waypoint(req.wp_name, wp_manager.pose_from_odom(self.current_pose))
        self.get_logger().info(f"adding new waypoint: {req.wp_name}")
        return res

    def goto_wp(self, req, res):
        wp_name = req.wp_name

        wp_exists = wp_name in self.wps.waypoints.keys()
        if not wp_exists:
            self.get_logger().warn(f"waypoint: {wp_name} does not exist...")
            return res

        goal: Pose = self.wps.waypoints[wp_name].as_geo_msg_pose()
        self.get_logger().info(f"going to waypoint: {req.wp_name}")


        self.goal_pose_pub_.publish(goal)

        return res

    def reset_cb(self, _: Empty):
        self.wps.reset_localization(wp_manager.pose_from_odom(self.current_pose))
        self.get_logger().info("localization was reset!")

    def odom_cb(self, msg: Odometry):
        self.current_pose = msg

def main():
    rclpy.init()
    node = WpNode()
    node.current_pose = Odometry()
    rclpy.spin(node)
    rclpy.shutdown()
        
if __name__ == "__main__":
    main()
