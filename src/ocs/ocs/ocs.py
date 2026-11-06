import datetime
import http.server
import json
import math
import os
import re
import socketserver
import threading
import time
from collections import defaultdict
from functools import partial
from pathlib import Path

import paho.mqtt.client as mqtt
import pymavlink.mavutil as mavutil
import rclpy
from geographic_msgs.msg import GeoPoint, GeoPose
from google.protobuf.json_format import MessageToDict
from google.protobuf.message import Message
from mil_msgs.msg import AcousticPingerConfig
from mil_msgs.msg import Heartbeat as RosHeartbeat
from mil_msgs.msg import PipelineSurveyReport as RosPipelineSurveyReport
from paho.mqtt.enums import CallbackAPIVersion
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty

from ocs.proto import (
    DockingReport,
    FlightPhase,
    Heartbeat,
    IncidentAck,
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

TARGET_MAV_TYPES = {
    "HEARTBEAT",
    "GLOBAL_POSITION_INT",
    "ATTITUDE",
    "SYS_STATUS",
    "EXTENDED_SYS_STATE",
    "BATTERY_STATUS",
}

TEAM_CODE = "UFLO"
CAAS_NUMBER = 0


class DashboardHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, status_handler, drone_status_handler, *args, **kwargs):
        self.status_handler = status_handler
        self.drone_status_handler = drone_status_handler
        super().__init__(*args, directory=DASHBOARD_DIR, **kwargs)

    def do_GET(self):
        if self.path.startswith("/api/status"):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()

            status = self.status_handler()
            status_json = json.dumps(status, default=self.json_default)

            self.wfile.write(str.encode(status_json))
        elif self.path.startswith("/api/drone_status"):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()

            drone_status = self.drone_status_handler()
            drone_status_json = json.dumps(drone_status)

            self.wfile.write(str.encode(drone_status_json))
        else:
            super().do_GET()

    def json_default(self, o):
        if isinstance(o, Message):
            return MessageToDict(o)
        raise TypeError(f"{type(o).__name__} is not JSON serializable")

    def send_head(self):
        rng = self.headers.get("Range")
        path = self.translate_path(self.path)
        if not rng or not os.path.isfile(path):
            return super().send_head()

        m = re.match(r"bytes=(\d*)-(\d*)$", rng.strip())
        if not m:
            return super().send_head()

        size = os.path.getsize(path)
        start_s, end_s = m.groups()
        if start_s == "" and end_s == "":
            return super().send_head()
        if start_s == "":  # suffix range: last N bytes
            start = max(size - int(end_s), 0)
            end = size - 1
        else:
            start = int(start_s)
            end = int(end_s) if end_s else size - 1
        end = min(end, size - 1)

        if start >= size or start > end:
            self.send_response(416)
            self.send_header("Content-Range", f"bytes */{size}")
            self.end_headers()
            return None

        f = open(path, "rb")
        f.seek(start)
        self.send_response(206)
        self.send_header("Content-Type", self.guess_type(path))
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.send_header("Content-Length", str(end - start + 1))
        self.end_headers()
        self._remaining = end - start + 1
        return f

    def copyfile(self, source, outputfile):
        remaining = getattr(self, "_remaining", None)
        if remaining is None:
            return super().copyfile(source, outputfile)
        while remaining > 0:
            chunk = source.read(min(64 * 1024, remaining))
            if not chunk:
                break
            outputfile.write(chunk)
            remaining -= len(chunk)
        self._remaining = None

    def end_headers(self):
        self.send_header("Accept-Ranges", "bytes")
        super().end_headers()


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

        self.dock_ocs_command_publisher = self.create_publisher(
            String,
            "/dock/ocs/command",
            10,
        )
        self.create_subscription(
            String,
            "/dock/ocs/result",
            self.on_dock_ocs_result,
            10,
        )

        self.sub_mission_publisher = self.create_publisher(
            String,
            "/mission_sending",
            10,
        )

        self.prop_emergency_publisher = self.create_publisher(
            GeoPose,
            "/latlon_goal/coordinate",
            10,
        )

        self.prop_all_clear_client = self.create_client(Empty, "/latlon_goal/all_clear")

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

        self.client.connect(
            self.get_parameter("host").get_parameter_value().string_value,
        )
        self.client.loop_start()

        self.mavconn = mavutil.mavlink_connection(
            self.get_parameter("mavlink_port").get_parameter_value().string_value,
            baud=57600,
        )

        mav_thread = threading.Thread(target=self.loop_mav)
        mav_thread.start()

        web_thread = threading.Thread(target=self.start_dashboard)
        web_thread.start()

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
    latest_heartbeats = {}

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
        self.latest_heartbeats[vehicle_id] = {
            "sent_at": datetime.datetime.now().isoformat(),
            **kwargs,
        }

    def rx_incident_ack(self, vehicle_id, command_seq):
        rep = RxReport(
            team_id=self.get_parameter("team_id").get_parameter_value().string_value,
            vehicle_id=vehicle_id,
            seq=self.rx_report_seqs[vehicle_id],
            incident_ack=IncidentAck(command_seq=command_seq),
        )
        rep.sent_at.GetCurrentTime()
        self.rx_report_seqs[vehicle_id] += 1
        self.client.publish(
            f"robocommand/robotx/{vehicle_id}/report",
            rep.SerializeToString(),
        )

    def on_course(self, _client, _, msg):
        course = RxCourse()

        try:
            course.ParseFromString(msg.payload)
        except Exception:
            print("failed to parse command protobuf")
            return

        config = AcousticPingerConfig()
        config.frequency = course.pinger_freq_hz
        self.acoustic_frequency_config_publisher.publish(config)

        # Set boundary parameters on sub/boat (does it need it)

        # TODO send geofence via mav, inset slightly

        self.rx_run_declaration(
            vehicle_ids=["UUV", "USV", "UAV"],
            task1_tier=TaskTier.TIER_CORE,
            task2_tier=TaskTier.TIER_CORE,
            task3_tier=TaskTier.TIER_CORE,
            task4_tier=TaskTier.TIER_CORE,
            uav_geofence=course.corners,  # TODO use sent geofence
        )

    current_run_id = None

    def on_command(self, _client, _, msg):
        command = RxCommand()

        try:
            command.ParseFromString(msg.payload)
        except Exception:
            print("failed to parse command payload")
            return

        if command.run_start is not None:
            if command.run_start.declaration_seq != self.rx_run_declaration_seq:
                print("mismatched RunStart")
                return

            self.current_run_id = command.run_start.run_id

            self.sub_mission_publisher.publish("PingerSurveyMission")
            # TODO probably want to start prop on task 1 before 3 here
            self.dock_ocs_command_publisher.publish("START")
            # TODO drone start?

        elif command.assistance_request is not None:
            if command.assistance_request.vehicle_type == VehicleType.TYPE_USV:
                msg = GeoPose()

                point = GeoPoint()
                point.latitude = command.position.latitude
                point.longitude = command.position.longitude

                msg.point = point
                self.prop_emergency_publisher.publish(msg)
                self.rx_incident_ack("USV", command.seq)
            elif command.assistance_request.vehicle_type == VehicleType.TYPE_UAV:
                self.mavconn.mav.command_long_send(
                    self.mavconn.target_system,
                    self.mavconn.target_component,
                    mavutil.mavlink.MAV_CMD_USER_4,
                    0,
                    4,
                    command.position.latitude,
                    command.position.longitude,
                    10,  # 10 meters above? idk
                    0,
                    0,
                    0,
                )
        elif command.keep_out_zone is not None:
            if command.keep_out_zone.vehicle_type == VehicleType.TYPE_USV:
                # TODO prop advanced tier not implemented yet
                self.rx_incident_ack("USV", command.seq)
            elif command.keep_out_zone.vehicle_type == VehicleType.TYPE_UAV:
                self.mavconn.mav.command_long_send(
                    self.mavconn.target_system,
                    self.mavconn.target_component,
                    mavutil.mavlink.MAV_CMD_USER_4,
                    0,
                    8,
                    command.center.latitude,
                    command.center.longitude,
                    command.radius_m,
                    0,
                    0,
                    0,
                )
                self.rx_incident_ack("UAV", command.seq)
        elif command.readiness_confirm is not None:
            if command.readiness_confirm.vehicle_id == "USV":
                req = Empty.Request()
                self.prop_all_clear_client.call_async(req)
            elif command.readiness_confirm.vehicle_id == "UAV":
                self.mavconn.mav.command_long_send(
                    self.mavconn.target_system,
                    self.mavconn.target_component,
                    mavutil.mavlink.MAV_CMD_USER_4,
                    0,
                    5,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
        elif command.all_clear is not None:
            if command.all_clear.vehicle_type == VehicleType.TYPE_USV:
                # TODO prop advanced tier not implemented yet
                self.rx_incident_ack("USV", command.seq)
            elif command.all_clear.vehicle_type == VehicleType.TYPE_UAV:
                self.mavconn.mav.command_long_send(
                    self.mavconn.target_system,
                    self.mavconn.target_component,
                    mavutil.mavlink.MAV_CMD_USER_4,
                    0,
                    9,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                self.rx_incident_ack("UAV", command.seq)

    drone_battery_percentage = 0
    drone_battery_time = 0

    def loop_mav(self):
        latest_messages = {}
        mode_map = mavutil.mode_mapping_bynumber(mavlink.MAV_TYPE_QUADROTOR)

        self.mavconn.wait_heartbeat()
        print("Got initial mavlink heartbeat")

        # TODO retry if no ack
        for TARGET in TARGET_MAV_TYPES:
            self.mavconn.mav.command_long_send(
                self.mavconn.target_system,
                self.mavconn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                getattr(mavlink, f"MAVLINK_MSG_ID_{TARGET}"),
                500_000,  # 2hz in microseconds
                0,
                0,
                0,
                0,
                0,
            )

        while True:
            # TODO dont reset to zero when missing value, just use last value
            # TODO alert when no heartbeats in a while

            # ensure mavlink buffer is cleared
            while True:
                msg = self.mavconn.recv_match(blocking=False)

                if msg is None:
                    break

                msg_type = msg.get_type()
                if msg_type in TARGET_MAV_TYPES:
                    latest_messages[msg_type] = msg

            latest_heartbeat = latest_messages.get("HEARTBEAT")
            if latest_heartbeat is None:
                state = RobotState.STATE_KILLED
            else:
                armed = latest_heartbeat.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED

                if not armed:
                    state = RobotState.STATE_KILLED
                else:
                    mode = mode_map.get(latest_heartbeat.custom_mode)
                    if mode == "GUIDED":
                        state = RobotState.STATE_AUTO
                    elif mode == "LOITER":
                        state = RobotState.STATE_LOITER
                    else:
                        state = RobotState.STATE_UNKNOWN

            latest_e_sys = latest_messages.get("EXTENDED_SYS_STATUS")
            if latest_e_sys is None:
                flight_phase = FlightPhase.FLIGHT_PHASE_GROUNDED
            else:
                match latest_e_sys.landed_state:
                    case mavlink.MAV_LANDED_STATE_ON_GROUND:
                        flight_phase = FlightPhase.FLIGHT_PHASE_GROUNDED
                    case (
                        mavlink.MAV_LANDED_STATE_IN_AIR
                        | mavlink.MAV_LANDED_STATE_TAKEOFF
                        | mavlink.MAV_LANDED_STATE_LANDING
                    ):
                        flight_phase = FlightPhase.FLIGHT_PHASE_AIRBORNE
                    case _:
                        flight_phase = FlightPhase.FLIGHT_PHASE_UNKNOWN

            latest_global_pos = latest_messages.get("GLOBAL_POSITION_INT")
            if latest_global_pos is None:
                altitude = 0
                position = None
                speed = 0
                heading = 0
            else:
                altitude = latest_global_pos.relative_alt / 1000.0  # mm -> m
                position = LatLng(
                    latitude=latest_global_pos.lat / 10**7,
                    longitude=latest_global_pos.lon / 10**7,
                )

                # all in cm/s
                vx = latest_global_pos.vx
                vy = latest_global_pos.vy
                vz = latest_global_pos.vz

                speed_cms = math.sqrt(vx**2 + vy**2 + vz**2)
                speed = speed_cms / 100.0

                heading = latest_global_pos.hdg / 100.0  # cdeg -> deg

            latest_attitude = latest_messages.get("ATTITUDE")
            if latest_attitude is None:
                roll = 0
                pitch = 0
            else:
                roll = latest_attitude.roll * (180 / math.pi)  # rad -> deg
                pitch = latest_attitude.pitch * (180 / math.pi)  # rad -> deg

            latest_battery = latest_messages.get("BATTERY_STATUS")
            if latest_battery is not None:
                self.drone_battery_percentage = latest_battery.battery_remaining
                self.drone_battery_time = latest_battery.time_remaining

            self.rx_heartbeat(
                vehicle_id="UAV",
                state=state,
                current_task=RxTask.TASK_NONE,  # todo
                vehicle_type=VehicleType.TYPE_UAV,
                flight_phase=flight_phase,
                position=position,
                spd_mps=speed,
                heading_deg=heading,
                roll_deg=roll,
                pitch_deg=pitch,
                altitude_hae_m=altitude,
                uav_registration=f"{TEAM_CODE}-{CAAS_NUMBER}",
            )

            time.sleep(0.5)  # 2hz

            # also need
            # alerts for weak link, geofence violation
            # buttons for submitting bad geofence

    def start_dashboard(self):
        if not os.path.exists(DASHBOARD_DIR):
            print(
                f"Error: Directory '{DASHBOARD_DIR}' not found. Run 'npm run build' first.",
            )
            exit(1)

        socketserver.TCPServer.allow_reuse_address = True

        port = self.get_parameter("dashboard_port").get_parameter_value().integer_value

        handler_class = partial(
            DashboardHandler,
            self.get_status,
            self.get_drone_status,
        )
        with socketserver.TCPServer(("", port), handler_class) as httpd:
            httpd.serve_forever()

    def get_status(self):
        return {
            "robocommand": self.client.is_connected(),
            "subjugatorHeartbeat": self.latest_heartbeats.get("UUV"),
            "propagatorHeartbeat": self.latest_heartbeats.get("USV"),
            "investigatorHeartbeat": self.latest_heartbeats.get("UAV"),
        }

    def get_drone_status(self):
        return {
            "battery_percentage": self.drone_battery_percentage,
            "battery_time": self.drone_battery_time,
        }

    def on_heartbeat(self, msg):
        self.rx_heartbeat(
            vehicle_id=msg.type,
            state=RobotState.Value(f"STATE_{msg.state}"),
            current_task=RxTask.Value(f"TASK_{msg.task}"),
            vehicle_type=VehicleType.Value(f"TYPE_{msg.type}"),
        )

    def on_pipeline_survey_report(self, msg):
        rep = RxReport(
            team_id=self.get_parameter("team_id").get_parameter_value().string_value,
            vehicle_id="USV",
            seq=self.rx_report_seqs["UUV"],
            pipeline_survey=PipelineSurveyReport(
                active_buoy_position=LatLng(
                    latitude=msg.active_buoy_position.y,
                    longitude=msg.active_buoy_position.x,
                ),
                segments=[s.status for s in msg.segments],
            ),
        )
        rep.sent_at.GetCurrentTime()
        self.rx_report_seqs["UUV"] += 1
        self.client.publish(
            "robocommand/robotx/UUV/report",
            rep.SerializeToString(),
        )

    def on_dock_ocs_result(self, msg):
        rep = RxReport(
            team_id=self.get_parameter("team_id").get_parameter_value().string_value,
            vehicle_id="USV",
            seq=self.rx_report_seqs["USV"],
            docking=DockingReport(bay_id=json.loads(msg)["bay_number"]),
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
