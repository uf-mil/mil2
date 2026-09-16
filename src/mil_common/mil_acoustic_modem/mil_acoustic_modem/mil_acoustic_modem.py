# This node will run on the boat and sub

import protobuf.pipeline_survey_report_pb2
import rclpy
from driver import acoustic_modem
from google.protobuf import any_pb2
from mil_msgs.msg import PipelineSegmentStatus, PipelineSurveyReport, Point2D
from rclpy.node import Node


class AcousticModem(Node):
    def __init__(self):
        super().__init__("acoustic_modem")

        self.declare_parameter("modem_serial_port", "COM9")

        self.pipeline_survey_report_publisher = self.create_publisher(
            PipelineSurveyReport,
            "pipeline_survey_report_received",
            5,
        )

        # 2 seconds (equal to the serial read timeout)
        self.timer = self.create_timer(2, self.read_latest_data)

        self.modem = acoustic_modem(
            "modem",
            self.get_parameter("modem_serial_port").get_parameter_value().string_value,
        )

    def read_latest_data(self):
        message = self.modem.read_im()
        parsed_message = any_pb2()
        parsed_message.ParseFromString(message)

        if parsed_message.Is(
            protobuf.pipeline_survey_report_pb2.PipelineSurveyReport.DESCRIPTOR,
        ):
            unpacked_message = (
                protobuf.pipeline_survey_report_pb2.PipelineSurveyReport()
            )
            parsed_message.Unpack(unpacked_message)

            ros_msg = PipelineSurveyReport()

            point_2d = Point2D()
            point_2d.x = unpacked_message.active_buoy_position_x
            point_2d.y = unpacked_message.active_buoy_position_y

            ros_msg.active_buoy_position = ros_msg
            ros_msg.segments = []

            for status in unpacked_message.segments:
                segment_status = PipelineSegmentStatus()
                segment_status.status = status

                ros_msg.segments.append(segment_status)

            self.pipeline_survey_report_publisher._publish(ros_msg)


def main(args=None):
    rclpy.init(args=args)

    acoustic_modem = AcousticModem()

    rclpy.spin(acoustic_modem)

    acoustic_modem.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
