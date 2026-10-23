# This node will run on the boat and sub

import rclpy
import serial.tools.list_ports
from mil_msgs.msg import (
    AcousticPingerConfig,
    Heartbeat,
    PipelineSegmentStatus,
    PipelineSurveyReport,
    Point2D,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mil_acoustic_modem.hardware_modem_interface import HardwareModemInterface
from mil_acoustic_modem.protobuf import (
    acoustic_pinger_config_pb2,
    heartbeat_pb2,
    mil_message_pb2,
    pipeline_survey_report_pb2,
)
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
            "pipeline_survey_report_sending",
            self.send_pipeline_survey_report,
            10,
            callback_group=self.sub_group,
        )

        self.heartbeat_publisher = self.create_publisher(
            Heartbeat,
            "heartbeat_received",
            5,
        )
        self.heartbeat_subscriber = self.create_subscription(
            Heartbeat,
            "heartbeat_sending",
            self.send_heartbeat,
            10,
            callback_group=self.sub_group,
        )

        self.acoustic_pinger_config_publisher = self.create_publisher(
            AcousticPingerConfig,
            "acoustic_pinger_config_received",
            5,
        )

        self.acoustic_pinger_config_subscriber = self.create_subscriber(
            AcousticPingerConfig,
            "acoustic_pinger_config_sending",
            self.send_acoustic_pinger_config,
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
                    self.get_logger().error("no USB serial devices found")
                    raise SystemExit

                modem_port = next(
                    (port for port in ports if "04D8:00DF" in port.hwid),
                    None,
                )

                if modem_port is None:
                    self.get_logger().error("no modem serial device found")
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

            self.get_logger().info("acoustic modem setup")

    def read_latest_data(self):
        protobuf_bytes = self.modem.read_im()

        if protobuf_bytes is None:
            return

        parsed_mil_message = mil_message_pb2.MilMessage()
        parsed_mil_message.ParseFromString(protobuf_bytes)

        match parsed_mil_message.type:
            case mil_message_pb2.MilMessageType.MIL_MESSAGE_TYPE_HEARTBEAT:
                heartbeat_msg = Heartbeat()
                body = parsed_mil_message.heartbeat

                heartbeat_msg.state = body.state

                point_2d = Point2D()
                point_2d.x = body.position_x
                point_2d.y = body.position_y

                heartbeat_msg.position = point_2d

                heartbeat_msg.speed = body.speed
                heartbeat_msg.heading = body.heading
                heartbeat_msg.roll = body.roll
                heartbeat_msg.pitch = body.pitch
                heartbeat_msg.depth = body.depth

                heartbeat_msg.task = body.task
                heartbeat_msg.type = body.type

                self.heartbeat_publisher.publish(heartbeat_msg)
            case mil_message_pb2.MilMessageType.MIL_MESSAGE_TYPE_PIPELINE_SURVEY_REPORT:
                report_msg = PipelineSurveyReport()
                body = parsed_mil_message.pipeline_survey_report

                point_2d = Point2D()
                point_2d.x = body.active_buoy_position_x
                point_2d.y = body.active_buoy_position_y

                report_msg.active_buoy_position = point_2d
                report_msg.segments = []

                for status in body.segments:
                    segment_status = PipelineSegmentStatus()
                    segment_status.status = status

                    report_msg.segments.append(segment_status)

                self.pipeline_survey_report_publisher.publish(report_msg)
            case mil_message_pb2.MilMessageType.MIL_MESSAGE_TYPE_ACOUSTIC_PINGER_CONFIG:
                config_msg = AcousticPingerConfig()
                body = parsed_mil_message.acoustic_pinger_config

                config_msg.frequency = body.frequency

                self.acoustic_pinger_config_publisher.publish(config_msg)

    def send_pipeline_survey_report(self, msg):
        report_protobuf = pipeline_survey_report_pb2.PipelineSurveyReport()
        report_protobuf.active_buoy_position_x = msg.active_buoy_position.x
        report_protobuf.active_buoy_position_y = msg.active_buoy_position.y
        for segment in msg.segments:
            report_protobuf.segments.append(segment.status)

        mil_protobuf = mil_message_pb2.MilMessage()
        mil_protobuf.type = (
            mil_message_pb2.MilMessageType.MIL_MESSAGE_TYPE_PIPELINE_SURVEY_REPORT
        )
        mil_protobuf.pipeline_survey_report = report_protobuf

        protobuf_string = mil_protobuf.SerializeToString()

        self.modem.send_im(protobuf_string)

    def send_heartbeat(self, msg):
        heartbeat_protobuf = heartbeat_pb2.Heartbeat()
        heartbeat_protobuf.state = msg.state
        heartbeat_protobuf.position_x = msg.position.x
        heartbeat_protobuf.position.y = msg.position.y
        heartbeat_protobuf.speed = msg.speed
        heartbeat_protobuf.heading = msg.heading
        heartbeat_protobuf.roll = msg.roll
        heartbeat_protobuf.pitch = msg.pitch
        heartbeat_protobuf.depth = msg.depth
        heartbeat_protobuf.task = msg.task
        heartbeat_protobuf.type = msg.type

        mil_protobuf = mil_message_pb2.MilMessage()
        mil_protobuf.type = mil_message_pb2.MilMessageType.MIL_MESSAGE_TYPE_HEARTBEAT
        mil_protobuf.heartbeat = heartbeat_protobuf

        protobuf_string = mil_protobuf.SerializeToString()

        self.modem.send_im(protobuf_string)

    def send_acoustic_pinger_config(self, msg):
        config_protobuf = acoustic_pinger_config_pb2.AcousticPingerConfig()
        config_protobuf.frequency = msg.frequency

        mil_protobuf = mil_message_pb2.MilMessage()
        mil_protobuf.type = (
            mil_message_pb2.MilMessageType.MIL_MESSAGE_TYPE_ACOUSTIC_PINGER_CONFIG
        )
        mil_protobuf.acoustic_pinger_config = config_protobuf

        protobuf_string = mil_protobuf.SerializeToString()

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
