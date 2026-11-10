#!/usr/bin/env python3
"""Play RoboCommand against the OCS, to test a run without the course network.

Needs an MQTT broker the OCS also uses (mosquitto, or amqtt), paho-mqtt, and
src/ocs on PYTHONPATH for the protos. It publishes a course, waits for the
RunDeclaration, prints every report, and sends RunStart once every declared
vehicle has reported AUTO, as RoboCommand does.

    PYTHONPATH=src/ocs python3 scripts/robocommand_standin.py [--host H] [--port 1883]

--corners gives the course boundary as lat,lon pairs in order, for example the
practice area's, instead of the default square in Marina Bay:

    ... robocommand_standin.py --corners 29.6481,-82.3438 29.6481,-82.3430 ...
"""

import argparse
import time

import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion

from ocs.proto import (
    LatLng,
    RobotState,
    RunStart,
    RxCommand,
    RxCourse,
    RxReport,
    RxRequest,
)

# Marina Bay, Singapore
CORNERS = [
    (1.2895, 103.8590),
    (1.2895, 103.8610),
    (1.2875, 103.8610),
    (1.2875, 103.8590),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--team", default="UFLO")
    parser.add_argument("--pinger", type=int, default=30000)
    parser.add_argument(
        "--corners",
        nargs="+",
        metavar="LAT,LON",
        type=lambda pair: tuple(float(v) for v in pair.split(",")),
        default=CORNERS,
    )
    args = parser.parse_args()

    declaration = None
    auto = set()
    started = False
    seq = 0

    def on_request(client, _, msg):
        nonlocal declaration
        req = RxRequest()
        req.ParseFromString(msg.payload)
        declaration = req
        d = req.run_declaration
        print(f"RunDeclaration seq {req.seq}: vehicles {list(d.vehicle_ids)}")

    def on_report(client, _, msg):
        nonlocal started, seq
        rep = RxReport()
        rep.ParseFromString(msg.payload)
        body = rep.WhichOneof("body")
        if body == "heartbeat":
            h = rep.heartbeat
            print(
                f"{rep.vehicle_id} heartbeat {RobotState.Name(h.state)}"
                f" task {h.current_task} at ({h.position.latitude:.6f},"
                f" {h.position.longitude:.6f}) heading {h.heading_deg:.0f}",
            )
            if h.state == RobotState.STATE_AUTO:
                auto.add(rep.vehicle_id)
        else:
            print(f"{rep.vehicle_id} {body}: {getattr(rep, body)}".replace("\n", " "))
        if declaration is None or started:
            return
        if set(declaration.run_declaration.vehicle_ids) <= auto:
            started = True
            command = RxCommand(
                team_id=args.team,
                seq=seq,
                run_start=RunStart(declaration_seq=declaration.seq, run_id=7),
            )
            command.sent_at.GetCurrentTime()
            seq += 1
            topic = f"robocommand/robotx/{args.team}/command"
            client.publish(topic, command.SerializeToString())
            print("every declared vehicle is AUTO: sent RunStart for run 7")

    client = mqtt.Client(callback_api_version=CallbackAPIVersion.VERSION2)
    client.message_callback_add(f"robocommand/robotx/{args.team}/request", on_request)
    client.message_callback_add(f"robocommand/robotx/{args.team}/+/report", on_report)
    client.connect(args.host, args.port)
    client.subscribe(f"robocommand/robotx/{args.team}/#")

    course = RxCourse(
        course_id="standin",
        pinger_freq_hz=args.pinger,
        corners=[
            LatLng(latitude=a, longitude=b) for a, b in args.corners + args.corners[:1]
        ],
    )
    course.sent_at.GetCurrentTime()
    client.publish("robocommand/robotx/course", course.SerializeToString(), retain=True)
    print("published the course; waiting for the OCS")
    client.loop_start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    client.loop_stop()


if __name__ == "__main__":
    main()
