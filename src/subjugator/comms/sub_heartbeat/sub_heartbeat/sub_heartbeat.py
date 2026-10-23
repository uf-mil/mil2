from math import hypot

import rclpy
from mil_msgs.msg import Heartbeat, Point2D
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from tf_transformations import euler_from_quaternion


class SubHeartbeat(rclpy.Node):
    def __init__(self):
        super().__init__("prop_heartbeat")

        self.heartbeat_publisher = self.create_publisher(
            Heartbeat,
            "heartbeat_sending",
            10,
        )

        self.odometry_subscriber = self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.odometry_callback,
            1,
        )

        self.mission_parameter_service_client = self.create_client(
            GetParameters,
            "/mission_planner/get_parameters",
        )

        self.latest_odometry = Odometry()

        self.timer = self.create_timer(0.5, self.request_heartbeat)

    def request_heartbeat(self):
        mission_parameters_request = GetParameters.Request()
        mission_parameters_request.names = ["mission"]
        request_future = self.mission_parameter_service_client.call_async(
            mission_parameters_request,
        )
        request_future.add_done_callback(self.publish_heartbeat)

    def publish_heartbeat(self, parameter_future):
        try:
            parameter_values = parameter_future.result()
            mission = parameter_values[0].string_value
        except Exception:
            self.get_logger().warn("could not fetch current mission")

        heartbeat = Heartbeat()
        heartbeat.state = "AUTO"  # TODO

        position = Point2D()
        position.x = self.latest_odometry.pose.pose.position.x
        position.y = self.latest_odometry.pose.pose.position.y

        heartbeat.position = position
        heartbeat.speed = hypot(
            self.latest_odometry.twist.twist.linear.x,
            self.latest_odometry.twist.twist.linear.y,
        )

        roll, pitch, yaw = euler_from_quaternion(
            [
                self.latest_odometry.pose.pose.orientation.x,
                self.latest_odometry.pose.pose.orientation.y,
                self.latest_odometry.pose.pose.orientation.z,
                self.latest_odometry.pose.pose.orientation.w,
            ],
        )

        heartbeat.heading = yaw
        heartbeat.pitch = pitch
        heartbeat.roll = roll

        if mission == "PingerSurveyMission":
            heartbeat.task = "INFRA_SURVEY_REPAIR"
        else:
            heartbeat.task = "NONE"

        heartbeat.type = "UUV"

        self.heartbeat_publisher.publish(heartbeat)

    def odometry_callback(self, msg):
        self.latest_odometry = msg


def main():
    rclpy.init(args=None)

    sub_heartbeat = SubHeartbeat()
    rclpy.spin(sub_heartbeat)

    sub_heartbeat.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
