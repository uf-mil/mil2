import threading, time, os, subprocess, pathlib
import rclpy
from rclpy.executors import MultiThreadedExecutor

def clean_ros_env() -> dict:
    env = os.environ.copy()
    for v in ("QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH", "QML2_IMPORT_PATH"):
        env.pop(v, None)
    if "LD_LIBRARY_PATH" in env:
        env["LD_LIBRARY_PATH"] = ":".join(
            p for p in env["LD_LIBRARY_PATH"].split(":") if "/cv2/" not in p
        )
    env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    pathlib.Path(env["XDG_RUNTIME_DIR"]).mkdir(parents=True, exist_ok=True)
    return env

subprocess.Popen(
    [
        "gnome-terminal", "--",
        "bash", "-c",
        "source /opt/ros/jazzy/setup.bash && "
        "source ~/mil2/install/setup.bash && "
        "ros2 launch subjugator_bringup gazebo.launch.py; exec bash"
    ],
    env=clean_ros_env(),
)


import imu_subscriber      # should expose ImuSubscriber or similar
import cam_subscriber      # should expose CamSubscriber or similar

#rclpy.init()

#IMU always goes first: it has the rclpy.init
imu_node = imu_subscriber.ImuSubscriber()
cam_node = cam_subscriber.CamSubscriber()

exec_ = MultiThreadedExecutor(num_threads=2)   # or 4 – up to you
exec_.add_node(imu_node)
exec_.add_node(cam_node)

spin_thread = threading.Thread(target=exec_.spin, daemon=True)
spin_thread.start()

try:
    while rclpy.ok():
        if imu_node.imu_data is not None:
            print(imu_node.imu_data)
        else:
            print("Waiting for IMU data…")

        if cam_node.cam_data is not None:
            print(cam_node.cam_data)
        else:
            print("Waiting for CAM data…")

        time.sleep(0.5)
finally:
    exec_.shutdown()
    rclpy.shutdown()
