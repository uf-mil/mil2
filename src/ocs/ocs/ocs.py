import http.server
import json
import os
import socketserver
from collections import defaultdict
from functools import partial
from pathlib import Path

import paho.mqtt.client as mqtt
import rclpy
from mil_msgs.msg import AcousticPingerConfig
from mil_msgs.msg import Heartbeat as RosHeartbeat
from mil_msgs.msg import PipelineSurveyReport as RosPipelineSurveyReport
from paho.mqtt.enums import CallbackAPIVersion
from rclpy.node import Node

from ocs.proto import (
    FlightPhase,
    Heartbeat,
    LatLng,
    PipelineSurveyReport,
    RobotState,
    RunDeclaration,
    RxCommand,
    RxCourse,
    RxReport,
    RxRequest,
    RxTask,
    TaskTier,
    VehicleType,
)

DASHBOARD_DIR = Path(__file__).parent / "dashboard" / "dist"


class DashboardHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, status_handler, *args, **kwargs):
        self.status_handler = status_handler
        super().__init__(*args, directory=DASHBOARD_DIR, **kwargs)

    def do_GET(self):
        if self.path.startswith("/api/status"):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()

            status = self.status_handler()
            status_json = json.dumps(status)

            self.wfile.write(str.encode(status_json))
        else:
            super().do_GET()


class OCS(Node):
    def __init__(self):
        super().__init__("ocs")

        self.declare_parameter("team_id", "UFLO")
        self.declare_parameter("host", "localhost")
        self.declare_parameter(
            "mavlink_port",
            "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00HS6A-if00-port0",
        )
        self.declare_parameter("dashboard_port", 3000)

        self.create_subscription(RosHeartbeat, "/heartbeats", self.on_heartbeat, 10)
        self.create_subscription(
            RosPipelineSurveyReport,
            "/pipeline_survey_report_received",
            self.on_pipeline_survey_report,
            10,
        )

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

        self.client.message_callback_add("robocommand/robotx/course", self.on_course)
        self.client.message_callback_add(
            f"robocommand/robotx/{self.get_parameter("team_id").get_parameter_value().string_value}/command",
            self.on_command,
        )

        # self.client.connect(
        #     self.get_parameter("host").get_parameter_value().string_value,
        # )
        # self.client.loop_start()

        # self.mavconn = pymavlink.mavutil.mavlink_connection(
        #     self.get_parameter("mavlink_port").get_parameter_value().string_value,
        #     baud=57600,
        # )

        self.rx_report_seqs = {}

        self.start_dashboard()

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

    def rx_heartbeat(self, *, vehicle_id, **kwargs):
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

    current_run_id = None

    def on_command(self, _client, _, msg):
        command = RxCommand()
        command.ParseFromString(msg.payload)

        if command.run_start is not None:
            if command.run_start.declaration_seq != self.rx_run_declaration_seq:
                print("mismatched RunStart")
                return

            self.current_run_id = command.run_start.run_id
            # TODO start??
        elif command.assistance_request is not None:
            if command.assistance_request.vehicle_type == VehicleType.TYPE_USV:
                # TODO send to prop?
                pass
            elif command.assistance_request.vehicle_type == VehicleType.TYPE_UAV:
                # TODO send to mav?
                pass
        elif command.readiness_confirm is not None:
            # TODO
            pass

    def loop_mav(self):
        skip = False  # 1 Hz -> 2 Hz
        while True:
            self.mavconn.wait_heartbeat()
            skip = not skip
            if skip:
                continue
            self.rx_heartbeat(
                vehicle_id="UAV",
                state=RobotState.STATE_AUTO,
                current_task=RxTask.TASK_NONE,
                vehicle_type=VehicleType.TYPE_UAV,
                flight_phase=FlightPhase.FLIGHT_PHASE_GROUNDED,
            )

    def start_dashboard(self):
        if not os.path.exists(DASHBOARD_DIR):
            print(
                f"Error: Directory '{DASHBOARD_DIR}' not found. Run 'npm run build' first.",
            )
            exit(1)

        socketserver.TCPServer.allow_reuse_address = True

        port = self.get_parameter("dashboard_port").get_parameter_value().integer_value

        handler_class = partial(DashboardHandler, self.get_status)
        with socketserver.TCPServer(("", port), handler_class) as httpd:
            httpd.serve_forever()

    def get_status(self):
        return {"robocommand": True}

    def on_heartbeat(self, msg):
        self.rx_heartbeat(
            vehicle_id=msg.vehicle_type,
            state=RobotState[f"STATE_{msg.state}"],
            current_task=RxTask[f"TASK_{msg.task}"],
            vehicle_type=VehicleType[f"TYPE_{msg.vehicle_type}"],
        )

    def on_pipeline_survey_report(self, msg):
        rep = RxReport(
            team_id=self.get_parameter("team_id").get_parameter_value().string_value,
            vehicle_id="USV",
            seq=self.rx_report_seqs["USV"],
            pipeline_survey=PipelineSurveyReport(
                active_buoy_position=LatLng(
                    latitude=msg.active_buoy_position.y,
                    longitude=msg.active_buoy_position.x,
                ),
                segments=[s.status for s in msg.segments],
            ),
        )
        rep.sent_at.GetCurrentTime()
        self.rx_report_seqs["USV"] += 1
        self.client.publish(
            "robocommand/robotx/USV/report",
            rep.SerializeToString(),
        )


def main():
    rclpy.init(args=None)

    ocs = OCS()
    rclpy.spin(ocs)

    ocs.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
