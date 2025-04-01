#!/usr/bin/env python3

import random
import sys

import matplotlib.pyplot as plt
import numpy as np
import rclpy
import scipy.linalg
import yaml
from geometry_msgs.msg import Vector3
from mpl_toolkits.mplot3d import Axes3D
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from tf_transformations import random_rotation_matrix, unit_vector


def normalized_matrix(m):
    assert np.linalg.det(m) > 0
    return m / np.linalg.det(m) ** (1 / m.shape[0])


def calculate_error(points):
    radii = list(map(np.linalg.norm, points))
    error = np.std(radii) / np.mean(radii)
    return error


def fit_ellipsoid(points):
    points = np.array(points)

    A = np.zeros((points.shape[0], 9))
    A[:, 0] = points[:, 0] ** 2
    A[:, 1] = points[:, 1] ** 2
    A[:, 2] = points[:, 2] ** 2
    A[:, 3] = 2 * points[:, 0] * points[:, 1]
    A[:, 4] = 2 * points[:, 0] * points[:, 2]
    A[:, 5] = 2 * points[:, 1] * points[:, 2]
    A[:, 6] = -2 * points[:, 0]
    A[:, 7] = -2 * points[:, 1]
    A[:, 8] = -2 * points[:, 2]

    B = np.ones((points.shape[0], 1))

    X = np.linalg.lstsq(A, B, rcond=None)[0].flatten()
    if X[0] < 0:
        X = -X

    ka = np.linalg.inv(
        np.array(
            [
                [X[0], X[3], X[4]],
                [X[3], X[1], X[5]],
                [X[4], X[5], X[2]],
            ],
        ),
    )
    shift = ka.dot(X[6:9])

    scale = scipy.linalg.sqrtm(ka)
    assert np.isreal(scale).all(), scale
    scale = np.real(scale)
    scale = normalized_matrix(scale)

    scale_inv = np.linalg.inv(scale)
    compensated = [scale_inv.dot(p - shift) for p in points]

    for axis in range(3):
        assert min(p[axis] for p in compensated) < 0 < max(p[axis] for p in compensated)

    return scale, shift, compensated


def axisEqual3D(ax):
    ax.axis("tight")
    extents = np.array([getattr(ax, f"get_{dim}lim")() for dim in "xyz"])
    r = np.max(np.abs(extents))
    for dim in "xyz":
        getattr(ax, f"set_{dim}lim")(-r, r)


def test():
    rot = random_rotation_matrix()[:3, :3]
    s = normalized_matrix(np.diag([random.expovariate(1) for _ in range(3)]))
    scale = rot @ s @ rot.T
    shift = np.random.randn(3)

    points = [scale @ unit_vector(np.random.randn(3)) + shift for _ in range(1000)]
    scale2, shift2, compensated = fit_ellipsoid(points)
    error = calculate_error(compensated)

    assert np.allclose(scale2, scale), "Self-test failed (scale)"
    assert np.allclose(shift2, shift), "Self-test failed (shift)"
    assert error < 1e-5, f"Self-test failed, error: {error}"


def _vector3_to_numpy(v: Vector3):
    return np.array([v.x, v.y, v.z])


def read_ros2_bag(bag_path, topic_name):
    storage_options = StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_type = None
    for topic in reader.get_all_topics_and_types():
        if topic.name == topic_name:
            topic_type = topic.type
            break

    if topic_type is None:
        raise ValueError(f"Topic '{topic_name}' not found in bag.")

    msg_type = get_message(topic_type)
    points = []

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(data, msg_type)
            vec = (
                _vector3_to_numpy(msg.magnetic_field)
                if hasattr(msg, "magnetic_field")
                else _vector3_to_numpy(msg.vector)
            )
            points.append(vec)

    return np.array(points)


def main():
    for _ in range(100):
        test()

    if len(sys.argv) < 2:
        print("Usage: ros2 run <package> <script> <bag_file>")
        sys.exit(1)

    bag_file = sys.argv[1]
    topic = "/vectornav/magnetic"

    points = read_ros2_bag(bag_file, topic)

    if len(points) == 0:
        print("No data in bag file.")
        sys.exit(1)

    print(f"original error: {100 * calculate_error(points):.4f}%")

    if plt:
        fig = plt.figure(figsize=(10, 10))
        ax = Axes3D(fig)
        ax.scatter([0], [0], [0], s=100, c="r")
        ax.scatter(*zip(*points[::10, :]))
        axisEqual3D(ax)
        plt.show()

    scale, shift, compensated = fit_ellipsoid(points)
    compensated = np.array(compensated)
    print(f"error: {100 * calculate_error(compensated):.4f}%")

    if plt:
        fig = plt.figure(figsize=(10, 10))
        ax = Axes3D(fig)
        ax.scatter([0], [0], [0], s=100, c="r")
        ax.scatter(*zip(*points[::10, :]))
        ax.scatter(*zip(*compensated[::10, :]), c="g")
        axisEqual3D(ax)
        plt.show()

    print(
        yaml.dump(
            {
                "scale": scale.tolist(),
                "shift": shift.tolist(),
            },
        ),
    )


if __name__ == "__main__":
    rclpy.init()
    main()
    rclpy.shutdown()
