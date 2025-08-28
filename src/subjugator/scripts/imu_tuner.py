# fig 1 is pre-rotate
# fig 2 is post-rotate

import subprocess

import matplotlib.pyplot as plt
import numpy as np

"""
Ok so this sets up the bs graph globally
"""
# Set interactive mode on
plt.ion()

# Setup plot 1
fig, ax = plt.subplots()
x_data = []
y1_data = []
y2_data = []
y3_data = []

(line1,) = ax.plot([], [], label="x stuff")
(line2,) = ax.plot([], [], label="y stuff")
(line3,) = ax.plot([], [], label="z stuff")

ax.set_xlim(0, 100)
ax.set_ylim(-10, 10)
ax.legend()

# Setup plot 2
fig2, ax2 = plt.subplots()
x_data2 = []
y1_data2 = []
y2_data2 = []
y3_data2 = []

(line12,) = ax2.plot([], [], label="x stuff")
(line22,) = ax2.plot([], [], label="y stuff")
(line32,) = ax2.plot([], [], label="z stuff")

ax2.set_xlim(0, 100)
ax2.set_ylim(-10, 10)
ax2.legend()


def vector_to_rpy(v):
    v = np.array(v, dtype=float)
    v_norm = v / np.linalg.norm(v)

    # Target vector is straight down -Z
    t = np.array([0, 0, -1])

    # Compute rotation axis and angle
    axis = np.cross(v_norm, t)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-8:
        # v is parallel or anti-parallel to t
        if np.dot(v_norm, t) > 0:
            # Already aligned, no rotation
            return 0.0, 0.0, 0.0
        else:
            # 180 degree rotation around any axis orthogonal to v
            # Find orthogonal axis
            orth = np.array([1, 0, 0])
            if np.abs(v_norm[0]) > 0.9:
                orth = np.array([0, 1, 0])
            axis = np.cross(v_norm, orth)
            axis /= np.linalg.norm(axis)
            theta = np.pi
    else:
        axis /= axis_norm
        theta = np.arccos(np.clip(np.dot(v_norm, t), -1.0, 1.0))

    # Rodrigues' rotation formula to build rotation matrix
    K = np.array(
        [[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]],
    )

    R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)

    # Extract RPY (ZYX order)
    pitch = np.arcsin(-R[2, 0])
    if np.cos(pitch) > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock
        roll = 0
        yaw = np.arctan2(-R[0, 1], R[1, 1])

    return roll, pitch, yaw


def rotate_about(ax, ay, az, roll, pitch, yaw):
    a_imu = np.array([ax, ay, az])

    # Rotation matrices
    Rx = np.array(
        [[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]],
    )

    Ry = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ],
    )

    Rz = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]],
    )

    # Combined rotation matrix (ZYX order)
    R = Rz @ Ry @ Rx

    # Rotate acceleration to robot frame
    a_robot = R @ a_imu

    return a_robot


# this function just nvm i don't wanna document
def echo_imu_topic():
    try:
        process = subprocess.Popen(
            ["ros2", "topic", "echo", "/imu/data", "--field", "linear_acceleration"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,  # decode stdout as text (str instead of bytes)
        )

        print("Listening to /imu/data topic (Ctrl+C to stop):")

        buffer = []
        i = 0

        for line in process.stdout:
            buffer.append(line)

            if len(buffer) == 4:
                x = float(buffer[0][3:].strip())
                y = float(buffer[1][3:].strip())
                z = float(buffer[2][3:].strip())

                roll, pitch, yaw = vector_to_rpy([x, y, z])
                print(f"Roll: {roll:.5f}, Pitch: {pitch:.5f}, Yaw: {yaw:.5f} (radians)")

                x2, y2, z2 = rotate_about(x, y, z, roll, pitch, yaw)

                x_data.append(i)
                y1_data.append(x)
                y2_data.append(y)
                y3_data.append(z)

                # Keep the last 100 points
                x_data_trim = x_data[-100:]
                y1_data_trim = y1_data[-100:]
                y2_data_trim = y2_data[-100:]
                y3_data_trim = y3_data[-100:]

                line1.set_data(x_data_trim, y1_data_trim)
                line2.set_data(x_data_trim, y2_data_trim)
                line3.set_data(x_data_trim, y3_data_trim)

                ax.set_xlim(max(0, i - 100), i + 10)

                x_data2.append(i)
                y1_data2.append(x2)
                y2_data2.append(y2)
                y3_data2.append(z2)

                # Keep the last 100 points
                x_data_trim2 = x_data2[-100:]
                y1_data_trim2 = y1_data2[-100:]
                y2_data_trim2 = y2_data2[-100:]
                y3_data_trim2 = y3_data2[-100:]

                line12.set_data(x_data_trim2, y1_data_trim2)
                line22.set_data(x_data_trim2, y2_data_trim2)
                line32.set_data(x_data_trim2, y3_data_trim2)

                ax2.set_xlim(max(0, i - 100), i + 10)

                # Redraw plot
                plt.pause(0.01)

                buffer = []
                i += 1

    except KeyboardInterrupt:
        print("\nStopping...")
        process.terminate()
        process.wait()


def main():
    echo_imu_topic()


if __name__ == "__main__":
    main()
