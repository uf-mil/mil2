# This node will run on the boat and sub

import rclpy
import serial.tools.list_ports
from mil_msgs.msg import PipelineSegmentStatus, PipelineSurveyReport, Point2D
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mil_acoustic_modem.hardware_modem_interface import HardwareModemInterface
from mil_acoustic_modem.protobuf import pipeline_survey_report_pb2
from mil_acoustic_modem.testing_modem_interface import TestingModemInterface


class AcousticModem(Node):
    def __init__(self):
        super().__init__("acoustic_modem")

        self.declare_parameter("modem_serial_port", "AUTO")
        self.declare_parameter("local_address", 0)
        self.declare_parameter("remote_address", 0)
        self.declare_parameter("carrier_waveform_id", 0)
        self.declare_parameter("gain", 0)

        self.sub_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()

        self.pipeline_survey_report_publisher = self.create_publisher(
            PipelineSurveyReport,
            "pipeline_survey_report_received",
            5,
        )
        self.pipeline_survey_report_subscriber = self.create_subscription(
            PipelineSurveyReport,
            f"pipeline_survey_report_sending_{self.get_parameter("modem_serial_port").get_parameter_value().string_value[-1]}",
            self.send_pipeline_survey_report,
            10,
            callback_group=self.sub_group,
        )

        # 2 seconds (serial read timeout + 1 to give time for sending)
        self.timer = self.create_timer(
            3,
            self.read_latest_data,
            callback_group=self.timer_group,
        )

        if (
            self.get_parameter("modem_serial_port").get_parameter_value().string_value
            == "TESTING"
        ):
            self.modem = TestingModemInterface()
        else:
            serial_port = (
                self.get_parameter("modem_serial_port")
                .get_parameter_value()
                .string_value
            )
            if serial_port == "AUTO":
                ports = serial.tools.list_ports.comports()

                if not ports:
                    print("No USB serial devices found")
                    raise SystemExit

                modem_port = next(
                    (port for port in ports if "04D8:00DF" in port.hwid),
                    None,
                )

                if modem_port is None:
                    print("No modem serial device found")
                    raise SystemExit

                serial_port = modem_port.device

            self.modem = HardwareModemInterface("modem", serial_port)
            self.modem.init_modem()
            self.modem.set_setting("Highest Address", 2)
            self.modem.set_setting(
                "Local Address",
                self.get_parameter("local_address").get_parameter_value().integer_value,
            )
            self.modem.set_setting(
                "Remote Address",
                self.get_parameter("remote_address")
                .get_parameter_value()
                .integer_value,
            )
            self.modem.set_setting(
                "Carrier Waveform ID",
                self.get_parameter("carrier_waveform_id")
                .get_parameter_value()
                .integer_value,
            )
            self.modem.set_setting(
                "Gain",
                self.get_parameter("gain").get_parameter_value().integer_value,
            )

            print("modem setup!")

    def read_latest_data(self):
        print("running read...")
        protobuf_bytes = self.modem.read_im()

        if protobuf_bytes is None:
            return

        parsed_message = pipeline_survey_report_pb2.PipelineSurveyReport()
        parsed_message.ParseFromString(protobuf_bytes)

        print("received msg")
        print(protobuf_bytes)
        print(parsed_message)

        ros_msg = PipelineSurveyReport()

        point_2d = Point2D()
        point_2d.x = parsed_message.active_buoy_position_x
        point_2d.y = parsed_message.active_buoy_position_y

        ros_msg.active_buoy_position = point_2d
        ros_msg.segments = []

        for status in parsed_message.segments:
            segment_status = PipelineSegmentStatus()
            segment_status.status = status

            ros_msg.segments.append(segment_status)

        self.pipeline_survey_report_publisher.publish(ros_msg)

    def send_pipeline_survey_report(self, msg):
        self.get_logger().info("got message!")
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

    executor = MultiThreadedExecutor()
    executor.add_node(acoustic_modem)

    try:
        executor.spin()
    except SystemExit:
        print("Quitting")

    acoustic_modem.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
