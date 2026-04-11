### i told chat-gpt to remove ros from the old driver I yoinked from MIL mono-repo
# this script is untested

import json
import socket
import time
import traceback

import numpy as np

# ---- CONFIG ----
host = "127.0.0.1"
port = 1234
child_frame_id = "base_link"
decimation = 1
force_z_to_zero = False


def build_absodom(d, t):
    ecef_cov = np.array(
        d["X_position_relative_position_orientation_ecef_covariance"]
    )

    pose_cov = np.vstack(
        (
            np.hstack((ecef_cov[0:3, 0:3], ecef_cov[0:3, 6:9])),
            np.hstack((ecef_cov[6:9, 0:3], ecef_cov[6:9, 6:9])),
        )
    ).flatten()

    twist_cov = np.vstack(
        (
            np.hstack((d["X_velocity_body_covariance"], np.zeros((3, 3)))),
            np.hstack((np.zeros((3, 3)), d["X_angular_velocity_body_covariance"])),
        )
    ).flatten()

    return {
        "type": "absodom",
        "timestamp": t,
        "frame_id": "ecef",
        "child_frame_id": child_frame_id,
        "pose": {
            "position": d["position_ecef"],
            "orientation": d["orientation_ecef"],
            "covariance": pose_cov.tolist(),
        },
        "twist": {
            "linear": d["velocity_body"],
            "angular": d["angular_velocity_body"],
            "covariance": twist_cov.tolist(),
        },
    }


def build_odom(d, t):
    pose_cov = np.array(
        d["X_relative_position_orientation_enu_covariance"]
    ).flatten()

    twist_cov = np.vstack(
        (
            np.hstack((d["X_velocity_body_covariance"], np.zeros((3, 3)))),
            np.hstack((np.zeros((3, 3)), d["X_angular_velocity_body_covariance"])),
        )
    ).flatten()

    return {
        "type": "odom",
        "timestamp": t,
        "frame_id": "enu",
        "child_frame_id": child_frame_id,
        "pose": {
            "position": d["relative_position_enu"],
            "orientation": d["orientation_enu"],
            "covariance": pose_cov.tolist(),
        },
        "twist": {
            "linear": d["velocity_body"],
            "angular": d["angular_velocity_body"],
            "covariance": twist_cov.tolist(),
        },
    }


def build_acceleration(d, t):
    return {
        "type": "acceleration",
        "timestamp": t,
        "frame_id": child_frame_id,
        "vector": d["X_acceleration_body"],
    }


def go():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))

    first = True
    buf = b""
    count = 0

    while True:
        data = s.recv(2**12)
        t = time.time()

        if not data:
            break

        buf += data
        lines = buf.split(b"\n")
        buf = lines[-1]

        for line in lines[:-1]:
            if first:
                first = False
                continue
            #print(json.loads(line.decode()))
            #continue

            if count % decimation == 0:
                d = json.loads(line.decode())

                if not d["running"]:
                    print(f"[WARN] GPS not running: {d['running_reason']}")
                    continue

                if force_z_to_zero:
                    d["relative_position_enu"][2] = 0

                absodom = build_absodom(d, t)
                odom = build_odom(d, t)
                accel = build_acceleration(d, t)

                # ---- PRINT INSTEAD OF PUBLISH ----
                print(json.dumps(absodom, indent=2))
                print(json.dumps(odom, indent=2))
                print(json.dumps(accel, indent=2))
                print("-" * 60)

            count += 1


while True:
    try:
        go()
    except Exception:
        traceback.print_exc()
        time.sleep(1)
