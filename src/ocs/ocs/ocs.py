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

from ocs.InvestiGator.src.gc_helpers import send_command
from ocs.InvestiGator.src.InvestiGator.mavconnection import MAVConnection
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
    "RADIO_STATUS",
}

TEAM_CODE = "UFLO"
CAAS_NUMBER = "0"
PILOT_NAME = "Ethan"
UAPL_ID = None

CALLSIGN = PILOT_NAME if UAPL_ID is None else f"{PILOT_NAME}-{UAPL_ID}"


class DashboardHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(
        self,
        status_handler,
        drone_status_handler,
        bad_drone_geofence_handler,
        bad_drone_height_handler,
        *args,
        **kwargs,
    ):
        self.status_handler = status_handler
        self.drone_status_handler = drone_status_handler
        self.bad_drone_geofence_handler = bad_drone_geofence_handler
        self.bad_drone_height_handler = bad_drone_height_handler

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

    def do_POST(self):
        if self.path.startswith("/api/bad_drone_geofence"):
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()

            self.bad_drone_geofence_handler()

            self.wfile.write(b"ok")
        elif self.path.startswith("/api/bad_drone_height"):
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()

            self.bad_drone_height_handler()

            self.wfile.write(b"ok")

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

        self.client.message_callback_add("robocommand/robotx/course", self.on_course)
        self.client.message_callback_add(
            f"robocommand/robotx/{self.get_parameter("team_id").get_parameter_value().string_value}/command",
            self.on_command,
        )

        def mqtt_on_connect(client, userdata, flags, reason_code, properties=None):
            if reason_code != 0:
                self.get_logger().error(f"MQTT connect failed: {reason_code}")
                return

            self.client.subscribe("robocommand/robotx/course")
            self.client.subscribe(
                f"robocommand/robotx/{self.get_parameter("team_id").get_parameter_value().string_value}/command",
            )

        self.client.on_connect = mqtt_on_connect

        self.mavconn = MAVConnection(
            self.get_parameter("mavlink_port").get_parameter_value().string_value,
            baud=57600,
            source_system=254,
            source_component=1,
            mav_type=mavlink.MAV_TYPE_GCS,
        )
        self.configure_messages()
        self.mavconn.subscribe(mavlink.MAVLink_heartbeat_message.msgname)(
            self.mav_handle_heartbeat,
        )
        self.mavconn.subscribe(mavlink.MAVLink_extended_sys_state_message.msgname)(
            self.mav_handle_extended_sys_state,
        )
        self.mavconn.subscribe(mavlink.MAVLink_global_position_int_message.msgname)(
            self.mav_handle_global_position,
        )
        self.mavconn.subscribe(mavlink.MAVLink_attitude_message.msgname)(
            self.mav_handle_attitude,
        )
        self.mavconn.subscribe(mavlink.MAVLink_battery_status_message.msgname)(
            self.mav_handle_battery_status,
        )
        self.mavconn.subscribe(mavlink.MAVLink_radio_status_message.msgname)(
            self.mav_handle_radio_status,
        )

        mav_heartbeat_thread = threading.Thread(target=self.mav_heartbeat_loop)
        mav_heartbeat_thread.start()

        web_thread = threading.Thread(target=self.start_dashboard)
        web_thread.start()

        self.client.connect(
            self.get_parameter("host").get_parameter_value().string_value,
        )
        self.client.loop_start()

    rx_run_declaration_seq = 0

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

    geofence = []

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

        self.geofence = (
            course.corners
            if len(course.uav_transit_area) == 0
            else course.uav_transit_area
        )

        cancel_event = threading.Event()
        self.mavconn.upload_fence_polygon(
            self.geofence,
            cancel_event,
            target_component=mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
        )
        print("Transmitted geofence")

        self.rx_run_declaration(
            vehicle_ids=["UUV", "USV", "UAV"],
            task1_tier=TaskTier.TIER_CORE,
            task2_tier=TaskTier.TIER_CORE,
            task3_tier=TaskTier.TIER_CORE,
            task4_tier=TaskTier.TIER_CORE,
            uav_geofence=course.corners,  # TODO use sent geofence,
            uav_geofence_max_altitude_hae_m=40,  # 50m HAE is max, keep it safe at 40.
            uav_pilot_callsign=CALLSIGN,
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
                send_command(
                    self.mavconn,
                    mavutil.mavlink.MAV_CMD_USER_4,
                    param1=4,
                    param2=command.position.latitude,
                    param3=command.position.longitude,
                    param4=10,  # 10 meters above
                )
        elif command.keep_out_zone is not None:
            if command.keep_out_zone.vehicle_type == VehicleType.TYPE_USV:
                # TODO prop advanced tier not implemented yet
                self.rx_incident_ack("USV", command.seq)
            elif command.keep_out_zone.vehicle_type == VehicleType.TYPE_UAV:
                send_command(
                    self.mavconn,
                    mavutil.mavlink.MAV_CMD_USER_4,
                    param1=8,
                    param2=command.center.latitude,
                    param3=command.center.longitude,
                    param4=command.radius_m,
                )

                self.rx_incident_ack("UAV", command.seq)
        elif command.readiness_confirm is not None:
            if command.readiness_confirm.vehicle_id == "USV":
                req = Empty.Request()
                self.prop_all_clear_client.call_async(req)
            elif command.readiness_confirm.vehicle_id == "UAV":
                send_command(self.mavconn, mavutil.mavlink.MAV_CMD_USER_4, param1=5)
        elif command.all_clear is not None:
            if command.all_clear.vehicle_type == VehicleType.TYPE_USV:
                # TODO prop advanced tier not implemented yet
                self.rx_incident_ack("USV", command.seq)
            elif command.all_clear.vehicle_type == VehicleType.TYPE_UAV:
                send_command(self.mavconn, mavutil.mavlink.MAV_CMD_USER_4, param1=9)
                self.rx_incident_ack("UAV", command.seq)

    def configure_messages(self):
        for TARGET in TARGET_MAV_TYPES:
            send_command(
                connection=self.mavconn,
                command=mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                target_component=1,
                param1=getattr(mavlink, f"MAVLINK_MSG_ID_{TARGET}"),
                param2=500_000,
            )

    drone_state = RobotState.STATE_KILLED
    drone_task = RxTask.TASK_NONE

    def mav_handle_heartbeat(self, message):
        if message.base_mode == 0:
            # Heartbeat from RPI - has robot task
            self.drone_task = message.custom_mode
        else:
            # Heartbeat from ArduPilot
            armed = message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if not armed:
                self.drone_state = RobotState.STATE_KILLED
            else:
                mode_map = mavutil.mode_mapping_bynumber(mavlink.MAV_TYPE_QUADROTOR)
                mode = mode_map.get(message.custom_mode)
                if mode == "GUIDED":
                    self.drone_state = RobotState.STATE_AUTO
                elif mode == "LOITER":
                    self.drone_state = RobotState.STATE_MANUAL
                else:
                    self.drone_state = RobotState.STATE_UNKNOWN

    drone_flight_phase = FlightPhase.FLIGHT_PHASE_UNKNOWN

    def mav_handle_extended_sys_state(self, message):
        match message.landed_state:
            case mavlink.MAV_LANDED_STATE_ON_GROUND:
                self.drone_flight_phase = FlightPhase.FLIGHT_PHASE_GROUNDED
            case (
                mavlink.MAV_LANDED_STATE_IN_AIR
                | mavlink.MAV_LANDED_STATE_TAKEOFF
                | mavlink.MAV_LANDED_STATE_LANDING
            ):
                self.drone_flight_phase = FlightPhase.FLIGHT_PHASE_AIRBORNE
            case _:
                self.drone_flight_phase = FlightPhase.FLIGHT_PHASE_UNKNOWN

    drone_altitude = 0
    drone_position = None
    drone_speed = 0
    drone_heading = 0

    def mav_handle_global_position(self, message):
        self.drone_altitude = message.relative_alt / 1000.0  # mm -> m
        self.drone_position = LatLng(
            latitude=message.lat / 10**7,
            longitude=message.lon / 10**7,
        )

        # all in cm/s
        vx = message.vx
        vy = message.vy
        vz = message.vz

        speed_cms = math.sqrt(vx**2 + vy**2 + vz**2)
        self.drone_speed = speed_cms / 100.0

        self.drone_heading = message.hdg / 100.0  # cdeg -> deg

    drone_roll = 0
    drone_pitch = 0

    def mav_handle_attitude(self, message):
        self.drone_roll = message.roll * (180 / math.pi)  # rad -> deg
        self.drone_pitch = message.pitch * (180 / math.pi)  # rad -> deg

    drone_battery_percentage = 0
    drone_battery_time = 0

    def mav_handle_battery_status(self, message):
        self.drone_battery_percentage = message.battery_remaining
        self.drone_battery_time = message.time_remaining

    drone_local_rssi_margin = 0
    drone_remote_rssi_margin = 0

    def mav_handle_radio_status(self, message):
        self.drone_local_rssi_margin = message.rssi - message.noise
        self.drone_remote_rssi_margin = message.remrssi - message.remnoise

    def mav_heartbeat_loop(self):
        while True:
            self.rx_heartbeat(
                vehicle_id="UAV",
                state=self.drone_state,
                current_task=self.drone_task,
                vehicle_type=VehicleType.TYPE_UAV,
                flight_phase=self.drone_flight_phase,
                position=self.drone_position,
                spd_mps=self.drone_speed,
                heading_deg=self.drone_heading,
                roll_deg=self.drone_roll,
                pitch_deg=self.drone_pitch,
                altitude_hae_m=self.drone_altitude,
                uav_registration=f"{TEAM_CODE}-{CAAS_NUMBER}",
            )

            time.sleep(0.5)  # 2hz

            # TODO alerts for geofence violation

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
            self.send_bad_drone_geofence,
            self.send_bad_drone_height,
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
            "local_rssi_margin": self.drone_local_rssi_margin,
            "remote_rssi_margin": self.drone_remote_rssi_margin,
        }

    def send_bad_drone_geofence(self):
        self.rx_run_declaration(
            vehicle_ids=["UAV"],
            task1_tier=TaskTier.TIER_CORE,
            task2_tier=TaskTier.TIER_CORE,
            task3_tier=TaskTier.TIER_CORE,
            task4_tier=TaskTier.TIER_CORE,
            uav_geofence=[
                LatLng(latitude=1, longitude=1),
                LatLng(latitude=2, longitude=1),
                LatLng(latitude=1.5, longitude=2),
                LatLng(latitude=1, longitude=1),
            ],
            uav_geofence_max_altitude_hae_m=40,
            uav_pilot_callsign=CALLSIGN,
        )

    def send_bad_drone_height(self):
        self.rx_run_declaration(
            vehicle_ids=["UAV"],
            task1_tier=TaskTier.TIER_CORE,
            task2_tier=TaskTier.TIER_CORE,
            task3_tier=TaskTier.TIER_CORE,
            task4_tier=TaskTier.TIER_CORE,
            uav_geofence=self.geofence,
            uav_geofence_max_altitude_hae_m=100,
            uav_pilot_callsign=CALLSIGN,
        )

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
