import datetime
import http.server
import json
import math
import os
import re
import socketserver
import threading
import time
import uuid
from collections import defaultdict
from functools import partial
from pathlib import Path

import paho.mqtt.client as mqtt
import pymavlink.mavutil as mavutil
import rclpy
from geographic_msgs.msg import GeoPath, GeoPoint, GeoPose, GeoPoseStamped
from google.protobuf.json_format import MessageToDict
from google.protobuf.message import Message
from mil_msgs.msg import AcousticPingerConfig
from mil_msgs.msg import Heartbeat as RosHeartbeat
from mil_msgs.msg import PipelineSurveyReport as RosPipelineSurveyReport
from paho.mqtt.enums import CallbackAPIVersion
from prop_missions.ocs_protocol import decode
from prop_ocs_interfaces.msg import SafePassageReport as RosSafePassageReport
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from shapely.geometry import Polygon
from std_msgs.msg import String, UInt32
from std_srvs.srv import Empty

from ocs.InvestiGator.src.gc_helpers import (
    send_command,
    send_mission_message_wait_ack,
)
from ocs.InvestiGator.src.InvestiGator.mavconnection import MAVConnection
from ocs.proto import (
    BeaconState,
    BuoyDetection,
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
    SafePassageReport,
    ReadinessReport,
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

        # RoboCommand MQTT broker. Defaults are for the RobotX 2026 TEST course
        # over Wi-Fi (network RN-TeamTest): UFLO's assigned port is 1907.
        #   Test course, cable at registration booth: host rc-test.local, port 1883
        #   If rc-test.local doesn't resolve on Wi-Fi: host 192.168.70.74, port 1907
        #   Main competition course: override with -p host:=... -p port:=...
        self.declare_parameter("team_id", "UFLO")
        self.declare_parameter("port", 1907)
        self.declare_parameter("host", "rc-test.local")
        self.declare_parameter(
            "mavlink_port",
            "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00HS6A-if00-port0",
        )
        self.declare_parameter("dashboard_port", 3000)
        # What this run declares. RoboCommand starts a run only once every
        # declared vehicle reports AUTO, so a boat-only test declares only USV.
        # The drone's radio is opened only when UAV is declared.
        self.declare_parameter("vehicles", ["UUV", "USV", "UAV"])
        for task in range(1, 5):
            self.declare_parameter(f"task{task}_tier", "CORE")

        self.create_subscription(RosHeartbeat, "/heartbeats", self.on_heartbeat, 10)
        self.create_subscription(
            RosSafePassageReport,
            "/prop/safe_passage_report",
            self.on_safe_passage_report,
            10,
        )
        # Not latched: a mission started later must not take an old run's
        # start for its own, so the boat's mission waits before the run.
        self.prop_run_start_publisher = self.create_publisher(
            UInt32,
            "/prop/run_start",
            10,
        )
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

        # The docking mission's exchange (prop_missions/ocs_protocol.py): it
        # says it is waiting on status, a START bound to its session starts
        # it, and it repeats its result until a receipt comes back.
        self.create_subscription(
            String,
            "/dock/ocs/result",
            self.on_dock_ocs_result,
            10,
        )
        self.create_subscription(String, "/dock/ocs/status", self.on_dock_status, 10)
        self.dock_command_publisher = self.create_publisher(
            String,
            "/dock/ocs/command",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.dock_receipt_publisher = self.create_publisher(
            String,
            "/dock/ocs/receipt",
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

        self.create_subscription(UInt32, "/prop/readiness_report", self.on_readiness, 10)
        self.prop_all_clear_client = self.create_client(Empty, "/latlon_goal/all_clear")

        # The course boundary, for the boat's survey; latched so a boat that
        # comes up after the course arrives still gets it.
        self.course_boundary_publisher = self.create_publisher(
            GeoPath,
            "/prop/course_boundary",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

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

        self.mavconn = None
        if "UAV" in self.vehicles():
            self.connect_drone()

        web_thread = threading.Thread(target=self.start_dashboard)
        web_thread.start()

        # connect_async and the loop thread keep retrying until RoboCommand is
        # reachable, and reconnect after a drop, as the handbook asks. An error
        # in a handler is logged instead of ending the loop thread, which would
        # silently stop every heartbeat.
        self.client.suppress_exceptions = True
        self.client.enable_logger()
        self.client.reconnect_delay_set(min_delay=1, max_delay=5)
        self.client.connect_async(
            self.get_parameter("host").get_parameter_value().string_value,
            self.get_parameter("port").get_parameter_value().integer_value,
        )
        self.client.loop_start()

    def team_id(self):
        return self.get_parameter("team_id").get_parameter_value().string_value

    def vehicles(self):
        return list(self.get_parameter("vehicles").value)

    def tier(self, task):
        name = self.get_parameter(f"task{task}_tier").value
        return TaskTier.Value(f"TIER_{name}")

    def on_readiness(self, msg):
        self.rx_readiness_report("USV", self.usv_command_seq)

    def connect_drone(self):
        """Open the drone's radio. Without it, the OCS runs on without the drone."""
        mavlink_port = (
            self.get_parameter("mavlink_port").get_parameter_value().string_value
        )
        if not mavlink_port:
            self.get_logger().info("mavlink_port is empty; running without drone")
            return
        try:
            self.mavconn = MAVConnection(
                mavlink_port,
                baud=57600,
                source_system=254,
                source_component=0,
                mav_type=mavlink.MAV_TYPE_GCS,
            )
        except Exception as e:
            self.get_logger().warning(
                f"Could not open drone MAVLink port {mavlink_port!r} ({e}); running without drone",
            )
            return
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

    def rx_report(self, vehicle_id, **body):
        """One RxReport about vehicle_id, on its own topic and sequence."""
        rep = RxReport(
            team_id=self.team_id(),
            vehicle_id=vehicle_id,
            seq=self.rx_report_seqs[vehicle_id],
            **body,
        )
        rep.sent_at.GetCurrentTime()
        self.rx_report_seqs[vehicle_id] += 1
        self.client.publish(
            f"robocommand/robotx/{self.team_id()}/{vehicle_id}/report",
            rep.SerializeToString(),
        )

    def rx_heartbeat(self, *, vehicle_id, **kwargs):
        self.rx_report(vehicle_id, heartbeat=Heartbeat(**kwargs))
        self.latest_heartbeats[vehicle_id] = {
            "sent_at": datetime.datetime.now().isoformat(),
            **kwargs,
        }

    def rx_incident_ack(self, vehicle_id, command_seq):
        self.rx_report(vehicle_id, incident_ack=IncidentAck(command_seq=command_seq))

    def rx_readiness_report(self, vehicle_id, command_seq):
        self.rx_report(vehicle_id, readiness=ReadinessReport(command_seq=command_seq))
    
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
        print("Course corners")
        for c in course.corners: 
            print(c.latitude, " ", c.longitude)

        self.course_boundary_publisher.publish(course_boundary(course))

        config = AcousticPingerConfig()
        config.frequency = course.pinger_freq_hz
        self.acoustic_frequency_config_publisher.publish(config)

        if "UAV" not in self.vehicles():
            self.rx_run_declaration(
                vehicle_ids=self.vehicles(),
                **{f"task{task}_tier": self.tier(task) for task in range(1, 5)},
            )
            return

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
        if self.mavconn is None:
            print("No drone connected; declaring the run without UAV")
            self.rx_run_declaration(
                vehicle_ids=[v for v in self.vehicles() if v != "UAV"],
                **{f"task{task}_tier": self.tier(task) for task in range(1, 5)},
            )
            return

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
            self.geofence.append(LatLng(latitude=r.x / 10**7, longitude=r.y / 10**7))
            self.max_height = r.z

        self.geofence.append(self.geofence[0])

        print("Got geofence")

        self.rx_run_declaration(
            vehicle_ids=self.vehicles(),
            **{f"task{task}_tier": self.tier(task) for task in range(1, 5)},
            uav_geofence=self.geofence,
            uav_geofence_max_altitude_hae_m=self.max_height,
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

        # A protobuf sub-message is never None, so the body that was sent is
        # the one WhichOneof names.
        body = command.WhichOneof("body")
        if body == "run_start":
            if command.run_start.declaration_seq != self.rx_run_declaration_seq:
                print("mismatched RunStart")
                return

            self.current_run_id = command.run_start.run_id
            self.dock_wanted = self.tier(3) != TaskTier.TIER_NONE
            self.dock_command = None
            self.docked_events = set()

            # The boat's tasks go in order: Safe Passage starts here, and
            # docking once the boat says Safe Passage is over, or now when
            # Safe Passage is not being attempted.
            self.prop_run_start_publisher.publish(UInt32(data=self.current_run_id))
            self.sub_mission_publisher.publish(String(data="PingerSurveyMission"))
            if self.tier(1) == TaskTier.TIER_NONE:
                self.start_docking()

        elif body == "assistance_request":
            request = command.assistance_request
            if request.vehicle_type == VehicleType.TYPE_USV:
                msg = GeoPose()

                point = GeoPoint()
                point.latitude = request.position.latitude
                point.longitude = request.position.longitude

                msg.position = point
                self.prop_emergency_publisher.publish(msg)
                self.rx_incident_ack("USV", command.seq)
                self.usv_command_seq = command.seq
            elif request.vehicle_type == VehicleType.TYPE_UAV:
                self.drone_send_command(
                    mavutil.mavlink.MAV_CMD_USER_4,
                    param1=4,
                    param2=request.position.latitude,
                    param3=request.position.longitude,
                    param4=10,  # 10 meters above
                )
        elif body == "keep_out_zone":
            zone = command.keep_out_zone
            if zone.vehicle_type == VehicleType.TYPE_USV:
                # TODO prop advanced tier not implemented yet
                self.rx_incident_ack("USV", command.seq)
            elif zone.vehicle_type == VehicleType.TYPE_UAV:
                self.drone_send_command(
                    mavutil.mavlink.MAV_CMD_USER_4,
                    param1=8,
                    param2=zone.center.latitude,
                    param3=zone.center.longitude,
                    param4=zone.radius_m,
                )

                self.rx_incident_ack("UAV", command.seq)
        elif body == "readiness_confirm":
            if command.readiness_confirm.vehicle_id == "USV":
                req = Empty.Request()
                self.prop_all_clear_client.call_async(req)
            elif command.readiness_confirm.vehicle_id == "UAV":
                self.drone_send_command(mavutil.mavlink.MAV_CMD_USER_4, param1=5)
        elif body == "all_clear":
            if command.all_clear.vehicle_type == VehicleType.TYPE_USV:
                # TODO prop advanced tier not implemented yet
                self.rx_incident_ack("USV", command.seq)
            elif command.all_clear.vehicle_type == VehicleType.TYPE_UAV:
                self.drone_send_command(mavutil.mavlink.MAV_CMD_USER_4, param1=9)
                self.rx_incident_ack("UAV", command.seq)

    def drone_send_command(self, *args, **kwargs):
        if self.mavconn is None:
            print("Ignoring drone command: no drone connected")
            return
        send_command(self.mavconn, *args, **kwargs)

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

    def mav_handle_heartbeat(self, message):
        self.drone_last_heartbeat_seconds = time.time()

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
        if self.mavconn is None:
            print("Ignoring drone mission: no drone connected")
            return
        send_mission_message_wait_ack(self.mavconn, mission)

    def on_heartbeat(self, msg):
        fields = {}
        if msg.type == "USV":
            fields = usv_heartbeat_fields(msg)
            if self.boat_task == "SAFE_PASSAGE" and msg.task == "NONE":
                self.start_docking()
            self.boat_task = msg.task
        else:
            fields = {
                "position": LatLng(latitude=msg.position.y, longitude=msg.position.x),
                "spd_mps": msg.speed,
                "heading_deg": msg.heading,
                "roll_deg": msg.roll,
                "pitch_deg": msg.pitch,
                "depth_m": msg.depth,
            }
        self.rx_heartbeat(
            vehicle_id=msg.type,
            state=RobotState.Value(f"STATE_{msg.state}"),
            current_task=RxTask.Value(f"TASK_{msg.task}"),
            vehicle_type=VehicleType.Value(f"TYPE_{msg.type}"),
            **fields,
        )

    def on_safe_passage_report(self, msg):
        self.rx_report("USV", safe_passage=safe_passage_report(msg))

    def on_pipeline_survey_report(self, msg):
        self.rx_report(
            "UUV",
            pipeline_survey=PipelineSurveyReport(
                active_buoy_position=LatLng(
                    latitude=msg.active_buoy_position.y,
                    longitude=msg.active_buoy_position.x,
                ),
                segments=[s.status for s in msg.segments],
            ),
        )

    boat_task = "NONE"
    dock_wanted = False
    dock_status = None
    dock_command = None
    docked_events = set()

    def on_dock_status(self, msg):
        status = decode(msg.data)
        if status is None or status["kind"] != "status":
            return
        self.dock_status = status
        if self.dock_command == "PENDING":
            self.start_docking()

    def start_docking(self):
        """START the docking mission waiting on the boat, for this run."""
        if not self.dock_wanted or self.current_run_id is None:
            return
        status = self.dock_status
        if status is None or status.get("state") != "WAIT_START":
            self.dock_command = "PENDING"
            print("docking: waiting for the docking mission to say it is ready")
            return
        self.dock_command = {
            "version": status["version"],
            "kind": "command",
            "vehicle_id": status["vehicle_id"],
            "session_id": status["session_id"],
            "run_id": str(self.current_run_id),
            "command_id": uuid.uuid4().hex,
            "action": "START",
        }
        self.dock_command_publisher.publish(String(data=json.dumps(self.dock_command)))
        print(f"docking: started session {status['session_id']}")

    def on_dock_ocs_result(self, msg):
        result = decode(msg.data)
        command = self.dock_command
        if (
            result is None
            or result["kind"] != "docking_result"
            or not isinstance(command, dict)
            or any(result.get(k) != command[k] for k in ("session_id", "run_id"))
            or result.get("bay_number") not in (1, 2, 3)
        ):
            return
        # The mission repeats a result until it hears back; report it once.
        if result["event_id"] not in self.docked_events:
            self.docked_events.add(result["event_id"])
            self.rx_report("USV", docking=DockingReport(bay_id=result["bay_number"]))
        receipt = {
            k: result[k]
            for k in ("version", "vehicle_id", "session_id", "run_id", "event_id")
        }
        receipt.update(kind="receipt", status="RECEIVED")
        self.dock_receipt_publisher.publish(String(data=json.dumps(receipt)))


def usv_heartbeat_fields(msg):
    """
    The rest of RoboCommand's heartbeat, from prop_heartbeat's: position is
    longitude and latitude in x and y, angles are radians, and heading is ENU
    yaw. A NaN means the boat does not know it yet, so it is left out.
    """
    fields = {}
    if math.isfinite(msg.position.x) and math.isfinite(msg.position.y):
        fields["position"] = LatLng(latitude=msg.position.y, longitude=msg.position.x)
    if math.isfinite(msg.heading):
        fields["heading_deg"] = (90.0 - math.degrees(msg.heading)) % 360.0
    if math.isfinite(msg.roll):
        fields["roll_deg"] = math.degrees(msg.roll)
    if math.isfinite(msg.pitch):
        fields["pitch_deg"] = math.degrees(msg.pitch)
    if math.isfinite(msg.speed):
        fields["spd_mps"] = msg.speed
    return fields


def course_boundary(course):
    """RxCourse's corners, the course boundary, as a GeoPath."""
    path = GeoPath()
    path.header.frame_id = "wgs84"
    for corner in course.corners:
        pose = GeoPoseStamped()
        pose.pose.position = GeoPoint(
            latitude=corner.latitude,
            longitude=corner.longitude,
        )
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def safe_passage_report(msg):
    """RoboCommand's SafePassageReport from the boat's. NaN entry or exit: not found."""

    def latlng(point):
        return LatLng(latitude=point.latitude, longitude=point.longitude)

    report = SafePassageReport(
        buoys=[
            BuoyDetection(
                position=latlng(buoy.position),
                state=BeaconState.Value(f"BEACON_STATE_{buoy.state}"),
            )
            for buoy in msg.buoys
        ],
    )
    if math.isfinite(msg.entry_position.latitude):
        report.entry_position.CopyFrom(latlng(msg.entry_position))
    if math.isfinite(msg.exit_position.latitude):
        report.exit_position.CopyFrom(latlng(msg.exit_position))
    return report


def main():
    rclpy.init(args=None)

    ocs = OCS()
    rclpy.spin(ocs)

    ocs.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
