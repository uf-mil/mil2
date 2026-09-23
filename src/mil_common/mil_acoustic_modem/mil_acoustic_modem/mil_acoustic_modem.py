# This node will run on the boat and sub

import rclpy
from google.protobuf import any_pb2
from mil_msgs.msg import PipelineSegmentStatus, PipelineSurveyReport, Point2D
from rclpy.node import Node

from mil_acoustic_modem.hardware_modem_interface import HardwareModemInterface
from mil_acoustic_modem.protobuf import pipeline_survey_report_pb2
from mil_acoustic_modem.testing_modem_interface import TestingModemInterface


class AcousticModem(Node):
    def __init__(self):
        super().__init__("acoustic_modem")

        self.declare_parameter("modem_serial_port", "COM9")
        self.declare_parameter("local_address", 0)
        self.declare_parameter("remote_address", 0)
        self.declare_parameter("carrier_waveform_id", 0)

        self.pipeline_survey_report_publisher = self.create_publisher(
            PipelineSurveyReport,
            "pipeline_survey_report_received",
            5,
        )
        self.pipeline_survey_report_subscriber = self.create_subscription(
            PipelineSurveyReport,
            "pipeline_survey_report_sending",
            self.send_pipeline_survey_report,
            10,
        )

        # 2 seconds (equal to the serial read timeout)
        self.timer = self.create_timer(2, self.read_latest_data)

        if (
            self.get_parameter("modem_serial_port").get_parameter_value().string_value
            == "TESTING"
        ):
            self.modem = TestingModemInterface()
        else:
            self.modem = HardwareModemInterface(
                "modem",
                self.get_parameter("modem_serial_port")
                .get_parameter_value()
                .string_value,
            )
            self.modem.init_modem()
            self.modem.set_max_addr(2)
            self.modem.set_local_addr(
                self.get_param("local_address").get_parameter_value().integer_value,
            )
            self.modem.set_remote_addr(
                self.get_param("remote_address").get_parameter_value().integer_value,
            )
            self.modem.set_carrier_waveform_id(
                self.get_param("carrier_waveform_id")
                .get_parameter_value()
                .integer_value,
            )

    def read_latest_data(self):
        message_bytes = self.modem.read_im()
        parsed_message = any_pb2.Any()
        protobuf_bytes = message_bytes.split(b",")[-1]
        parsed_message.ParseFromString(protobuf_bytes)

        if parsed_message.Is(
            pipeline_survey_report_pb2.PipelineSurveyReport.DESCRIPTOR,
        ):
            unpacked_message = pipeline_survey_report_pb2.PipelineSurveyReport()
            parsed_message.Unpack(unpacked_message)

            ros_msg = PipelineSurveyReport()

            point_2d = Point2D()
            point_2d.x = unpacked_message.active_buoy_position_x
            point_2d.y = unpacked_message.active_buoy_position_y

            ros_msg.active_buoy_position = point_2d
            ros_msg.segments = []

            for status in unpacked_message.segments:
                segment_status = PipelineSegmentStatus()
                segment_status.status = status

                ros_msg.segments.append(segment_status)

            self.pipeline_survey_report_publisher.publish(ros_msg)

    def send_pipeline_survey_report(self, msg):
        protobuf_message = pipeline_survey_report_pb2.PipelineSurveyReport()
        protobuf_message.active_buoy_position_x = msg.active_buoy_position.x
        protobuf_message.active_buoy_position_y = msg.active_buoy_position.y
        for segment in msg.segments:
            protobuf_message.segments.append(segment.status)

        protobuf_string = protobuf_message.SerializeToString()

        self.modem.send_im(protobuf_string)


def main(args=None):
    rclpy.init(args=args)

    acoustic_modem = AcousticModem()

    rclpy.spin(acoustic_modem)

    acoustic_modem.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
