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
from shapely.geometry import Polygon
from std_msgs.msg import String
from std_srvs.srv import Empty

from ocs.InvestiGator.src.gc_helpers import (
    send_command,
    send_mission_message_wait_ack,
)
from ocs.InvestiGator.src.InvestiGator.mavconnection import MAVConnection
from ocs.proto import (
    DockingReport,
    FlightPhase,
    Heartbeat,
    IncidentAck,
    LatLng,
    PipelineSurveyReport,
    ReadinessReport,
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
CAAS_NUMBER = "53581"
PILOT_NAME = "EthanM"
UAPL_ID = None

CALLSIGN = PILOT_NAME if UAPL_ID is None else f"{PILOT_NAME}-{UAPL_ID}"


class DashboardHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(
        self,
        status_handler,
        drone_status_handler,
        bad_drone_geofence_handler,
        bad_drone_height_handler,
        good_drone_handler,
        mission_handler,
        *args,
        **kwargs,
    ):
        self.status_handler = status_handler
        self.drone_status_handler = drone_status_handler
        self.bad_drone_geofence_handler = bad_drone_geofence_handler
        self.bad_drone_height_handler = bad_drone_height_handler
        self.good_drone_handler = good_drone_handler
        self.mission_handler = mission_handler

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
            drone_status_json = json.dumps(drone_status, default=self.json_default)

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
        elif self.path.startswith("/api/good_drone"):
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()

            self.good_drone_handler()

            self.wfile.write(b"ok")
        elif self.path.startswith("/api/mission"):
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length)
            try:
                data = json.loads(raw.decode("utf-8"))
            except (json.JSONDecodeError, UnicodeDecodeError):
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(b'{"error": "invalid JSON"}')
                return

            print(data)
            self.mission_handler(data["mission"])

            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
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
        self.declare_parameter("port", 1883)
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
            source_component=0,
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
        self.mavconn.subscribe(mavlink.MAVLink_statustext_message.msgname)(
            self.mav_handle_statustext,
        )

        mav_heartbeat_thread = threading.Thread(target=self.mav_heartbeat_loop)
        mav_heartbeat_thread.start()

        web_thread = threading.Thread(target=self.start_dashboard)
        web_thread.start()

        self.client.connect(
            self.get_parameter("host").get_parameter_value().string_value,
            self.get_parameter("port").get_parameter_value().integer_value,
        )
        self.client.loop_start()

    rx_run_declaration_seq = -1

    def rx_run_declaration(self, **kwargs):
        self.rx_run_declaration_seq += 1

        req = RxRequest(
            team_id=self.get_parameter("team_id").get_parameter_value().string_value,
            seq=self.rx_run_declaration_seq,
            run_declaration=RunDeclaration(**kwargs),
        )
        req.sent_at.GetCurrentTime()

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
            f"robocommand/robotx/{TEAM_CODE}/{vehicle_id}/report",
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

    def rx_readiness_report(self, vehicle_id, assistance_seq):
        rep = RxReport(
            team_id=self.get_parameter("team_id").get_parameter_value().string_value,
            vehicle_id=vehicle_id,
            seq=self.rx_report_seqs[vehicle_id],
            readiness_report=ReadinessReport(command_seq=assistance_seq),
        )
        rep.sent_at.GetCurrentTime()
        self.rx_report_seqs[vehicle_id] += 1
        self.client.publish(
            f"robocommand/robotx/{vehicle_id}/report",
            rep.SerializeToString(),
        )

    def to_polygon(self, coords):
        return Polygon([(p.longitude, p.latitude) for p in coords])

    course_geofence = []
    geofence = []
    max_height = 0
    course_received = False

    def on_course(self, _client, _, msg):
        print("Got RxCourse from RoboCommand")
        course = RxCourse()

        try:
            course.ParseFromString(msg.payload)
        except Exception:
            print("failed to parse command protobuf")
            return

        self.course_received = True

        config = AcousticPingerConfig()
        config.frequency = course.pinger_freq_hz
        self.acoustic_frequency_config_publisher.publish(config)

        if len(course.uav_transit_area) == 0:
            self.course_geofence = course.corners
        else:
            corners_poly = self.to_polygon(course.corners)
            uav_poly = self.to_polygon(course.uav_transit_area)
            union = corners_poly.union(uav_poly)
            self.course_geofence = [
                LatLng(latitude=lat, longitude=lon)
                for lon, lat in union.exterior.coords
            ]

        self.drone_geofence_and_run_decl(self.course_geofence, 40)

    def drone_geofence_and_run_decl(self, geofence, max_height_hae_m):
        cancel_event = threading.Event()
        geofence_transmitted = self.mavconn.upload_fence_polygon(
            geofence,
            cancel_event,
            target_component=mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
        )
        send_command(
            self.mavconn,
            mavutil.mavlink.MAV_CMD_USER_4,
            param1=11,
            param2=max_height_hae_m,
        )

        if not geofence_transmitted:
            return

        print("Transmitted geofence")
        time.sleep(2)  # give time for fence to go live

        responses = self.mavconn.read_cube_fence(
            target_component=mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
        )

        if responses is None:
            print("failed to get geofence")
            return

        self.geofence = []

        for r in responses:
            l_x = r.x / 10**7
            l_y = r.y / 10**7
            self.geofence.append(LatLng(latitude=l_x, longitude=l_y))
            self.max_height = r.z

            print(f"adding geofence corner {l_x}, {l_y}")

        self.geofence.append(self.geofence[0])

        print("Got geofence")

        self.rx_run_declaration(
            # vehicle_ids=["UUV", "USV", "UAV"],
            vehicle_ids=["UAV"],
            task1_tier=TaskTier.TIER_CORE,
            task2_tier=TaskTier.TIER_CORE,
            task3_tier=TaskTier.TIER_CORE,
            task4_tier=TaskTier.TIER_CORE,
            uav_geofence=self.geofence,
            uav_geofence_max_altitude_hae_m=self.max_height,
            uav_pilot_callsign=CALLSIGN,
        )

    current_run_id = None
    assistance_request_id = None

    def on_command(self, _client, _, msg):
        command = RxCommand()

        try:
            command.ParseFromString(msg.payload)
        except Exception:
            print("failed to parse command payload")
            return

        print(command)

        if command.HasField("run_start"):
            if command.run_start.declaration_seq != self.rx_run_declaration_seq:
                print("mismatched RunStart")
                return

            print("Got RunStart")
            self.current_run_id = command.run_start.run_id

            sub_mission = String()
            sub_mission.data = "PingerSurveyMission"
            self.sub_mission_publisher.publish(sub_mission)

            # TODO probably want to start prop on task 1 before 3 here

            dock_ocs_command = String()
            dock_ocs_command.data = "START"
            self.dock_ocs_command_publisher.publish(dock_ocs_command)

            self.send_drone_mission(11)  # drone loiters at 5m for 120s

        elif command.HasField("assistance_request"):
            if command.assistance_request.vehicle_type == VehicleType.TYPE_USV:
                print("got assistance request for usv")
                msg = GeoPose()

                point = GeoPoint()
                point.latitude = command.assistance_request.position.latitude
                point.longitude = command.assistance_request.position.longitude

                msg.point = point
                self.prop_emergency_publisher.publish(msg)
                self.rx_incident_ack("USV", command.seq)
            elif command.assistance_request.vehicle_type == VehicleType.TYPE_UAV:
                print("got assistance request for uav")
                start_pos = self.drone_position
                target = command.assistance_request.position
                if start_pos is not None:
                    print(
                        f"UAV move from:\n\tlat {start_pos.latitude:.7f}\n\tlon {start_pos.longitude:.7f}\n"
                        f"to:\n\tlat {target.latitude:.7f}\n\tlon {target.longitude:.7f}",
                    )
                self.drone_at_incident = False
                self.assistance_request_id = command.seq
                send_command(
                    self.mavconn,
                    mavutil.mavlink.MAV_CMD_USER_4,
                    param1=4,
                    param2=command.assistance_request.position.latitude,
                    param3=command.assistance_request.position.longitude,
                    param4=10,  # 10 meters above
                )
                self.rx_incident_ack("UAV", command.seq)
        elif command.HasField("keep_out_zone"):
            if command.keep_out_zone.vehicle_type == VehicleType.TYPE_USV:
                # TODO prop advanced tier not implemented yet
                self.rx_incident_ack("USV", command.seq)
            elif command.keep_out_zone.vehicle_type == VehicleType.TYPE_UAV:
                send_command(
                    self.mavconn,
                    mavutil.mavlink.MAV_CMD_USER_4,
                    param1=8,
                    param2=command.keep_out_zone.center.latitude,
                    param3=command.keep_out_zone.center.longitude,
                    param4=command.keep_out_zone.radius_m,
                )

                self.rx_incident_ack("UAV", command.seq)
        elif command.HasField("readiness_confirm"):
            if command.readiness_confirm.vehicle_id == "USV":
                req = Empty.Request()
                self.prop_all_clear_client.call_async(req)
            elif command.readiness_confirm.vehicle_id == "UAV":
                send_command(self.mavconn, mavutil.mavlink.MAV_CMD_USER_4, param1=5)
        elif command.HasField("all_clear"):
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
    drone_last_heartbeat_seconds = 0
    drone_at_incident = False
    drone_armed = False

    def mav_handle_heartbeat(self, message):
        self.drone_last_heartbeat_seconds = time.time()

        if message.base_mode == 0:
            # Heartbeat from RPI - has robot task
            self.drone_task = message.custom_mode

            # 15 == MIL_STATE_AT_INCIDENT
            if not self.drone_at_incident and message.system_status == 15:
                print("uav at incident")
                self.rx_readiness_report("UAV", self.assistance_request_id)

        else:
            # Heartbeat from ArduPilot
            armed = message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            start_pos = self.drone_position
            if armed and not self.drone_armed and start_pos is not None:
                print(
                    f"UAV armed at:\n\tlat {start_pos.latitude:.7f}\n\tlon {start_pos.longitude:.7f}",
                )
            self.drone_armed = bool(armed)
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

    drone_status_text = None

    def mav_handle_statustext(self, message):
        self.drone_status_text = message.text

    def mav_heartbeat_loop(self):
        while True:
            if time.time() - self.drone_last_heartbeat_seconds < 1:
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
            self.send_good_drone,
            self.send_drone_mission,
        )
        with socketserver.TCPServer(("", port), handler_class) as httpd:
            httpd.serve_forever()

    def get_status(self):
        return {
            "robocommand": {
                "connected": self.client.is_connected(),
                "course_received": self.course_received,
                "run_start_received": self.current_run_id is not None,
            },
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
            "geofence": list(self.geofence),
            "status_text": self.drone_status_text,
        }

    def send_bad_drone_geofence(self):
        self.drone_geofence_and_run_decl(
            [
                LatLng(latitude=1, longitude=1),
                LatLng(latitude=2, longitude=1),
                LatLng(latitude=1.5, longitude=2),
                LatLng(latitude=1, longitude=1),
            ],
            40,
        )

    def send_bad_drone_height(self):
        self.drone_geofence_and_run_decl(self.course_geofence, 100)

    def send_good_drone(self):
        self.drone_geofence_and_run_decl(self.course_geofence, 40)

    def send_drone_mission(self, mission):
        send_mission_message_wait_ack(self.mavconn, mission)

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
