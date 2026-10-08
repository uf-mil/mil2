from collections import defaultdict

import paho.mqtt.client as mqtt
import pymavlink.mavutil
import rclpy
from mil_msgs.msg import AcousticPingerConfig
from paho.mqtt.enums import CallbackAPIVersion
from proto import (
    FlightPhase,
    Heartbeat,
    RobotState,
    RunDeclaration,
    RxCourse,
    RxReport,
    RxRequest,
    RxTask,
    TaskTier,
    VehicleType,
)
from rclpy.node import Node
from std_msgs.msg import Empty


class OCS(Node):
    def __init__(self):
        super().__init__("ocs")

        self.declare_parameter("team_id", "UFLO")
        self.declare_parameter("host", "localhost")
        self.declare_parameter(
            "mavlink_port",
            "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00HS6A-if00-port0",
        )

        self.create_subscription(Empty, "/heartbeats", self.on_heartbeat, 10)

        self.acoustic_frequency_config_publisher = self.create_publisher(
            AcousticPingerConfig,
            "/acoustic_frequency_config_sending",
            10,
        )

        self.client = mqtt.Client(callback_api_version=CallbackAPIVersion.VERSION2)

        self.client.subscribe("robocommand/robotx/course")
        self.client.subscribe(
            f"robocommand/robotx/{self.get_parameter("team_id").get_parameter_value().string_value}/command",
        )

        self.client.message_callback.add("robocommand/robotx/course", self.on_course)
        self.client.message_callback.add(
            f"robocommand/robotx/{self.get_parameter("team_id").get_parameter_value().string_value}/command",
            self.on_command,
        )

        self.client.connect(
            self.get_parameter("host").get_parameter_value().string_value,
        )
        self.client.loop_start()

        self.mavconn = pymavlink.mavutil.mavlink_connection(
            self.get_parameter("mavlink_port").get_parameter_value().string_value,
            baud=57600,
        )

        self.rx_report_seqs = {}

    def rx_run_declaration(self, **kwargs):
        req = RxRequest(
            team_id=self.get_parameter("team_id").get_parameter_value().string_value,
            seq=self.rx_run_declaration_seq,
            run_declaration=RunDeclaration(**kwargs),
        )
        req.sent_at.GetCurrentTime()
        self.rx_run_declaration_seq += 1
        self.client.publish(
            f"robocommand/robotx/{self.get_parameter("team_id").get_parameter_value().string_value}/request",
            req.SerializeToString(),
        )

    rx_report_seqs = defaultdict(int)

    def rx_report(self, *, vehicle_id, **kwargs):
        rep = RxReport(
            team_id=self.get_parameter("team_id").get_parameter_value().string_value,
            vehicle_id=vehicle_id,
            seq=self.rx_report_seqs[vehicle_id],
            heartbeat=Heartbeat(**kwargs),
        )
        rep.sent_at.GetCurrentTime()
        self.rx_report_seqs[vehicle_id] += 1
        self.client.publish(
            f"robocommand/robotx/{vehicle_id}/report",
            rep.SerializeToString(),
        )

    def on_course(self, _client, _, msg):
        course = RxCourse()
        course.ParseFromString(msg.payload)

        config = AcousticPingerConfig()
        config.frequency = course.pinger_freq_hz
        self.acoustic_frequency_config_publisher.publish(config)

        # Set boundary parameters on sub/boat (does it need it)

        self.rx_run_declaration(
            vehicle_ids=["UUV", "USV", "UAV"],
            task1_tier=TaskTier.TIER_CORE,
            task2_tier=TaskTier.TIER_CORE,
            task3_tier=TaskTier.TIER_CORE,
            task4_tier=TaskTier.TIER_CORE,
            uav_geofence=course.corners,
        )

    def on_command(_client, _, msg):
        pass

    def loop_mav(self):
        skip = False  # 1 Hz -> 2 Hz
        while True:
            self.mavconn.wait_heartbeat()
            skip = not skip
            if skip:
                continue
            self.rx_report(
                vehicle_id="UAV",
                state=RobotState.STATE_AUTO,
                current_task=RxTask.TASK_NONE,
                vehicle_type=VehicleType.TYPE_UAV,
                flight_phase=FlightPhase.FLIGHT_PHASE_GROUNDED,
            )

    # todo - implement heartbeats on boat and sub
    # sub heartbeats through acoustic modem
    def on_heartbeat(self, msg):
        self.rx_report(
            vehicle_id=msg.vehicle_type,
            state=RobotState[f"STATE_{msg.state}"],
            current_task=RxTask[f"TASK_{msg.task}"],
            vehicle_type=VehicleType[f"TYPE_{msg.vehicle_type}"],
        )


def main():
    rclpy.init(args=None)

    ocs = OCS()
    rclpy.spin(ocs)

    ocs.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
