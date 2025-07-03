import os
import pathlib
import subprocess
import threading
import time

import gymnasium as gym
import numpy as np
import rclpy
from gymnasium import spaces
from rclpy.executors import MultiThreadedExecutor

from subjugator_RL import GymNode
from subjugator_RL.locks import cam_lock, imu_lock

# Shape of the image. L, W, # of channels
SHAPE = [50, 80, 3]


# ROS environment crashes in spawned terminal without using this - caused by OpenCv depednencies
def clean_ros_env() -> dict:
    env = os.environ.copy()

    # 1) Remove only the variables that the opencv wheel polluted
    for v in ("QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH", "QML2_IMPORT_PATH"):
        env.pop(v, None)

    # 2) Strip every LD_LIBRARY_PATH component that contains "/cv2/"
    if "LD_LIBRARY_PATH" in env:
        env["LD_LIBRARY_PATH"] = ":".join(
            p for p in env["LD_LIBRARY_PATH"].split(":") if "/cv2/" not in p
        )

    env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    pathlib.Path(env["XDG_RUNTIME_DIR"]).mkdir(parents=True, exist_ok=True)

    return env


class SubEnv(gym.Env):
    proc = None
    image_gets = 0
    imu_gets = 0
    get_attempts = 0

    def __init__(self, render_mode="rgb_array"):

        # Initialize ROS2 if not already done
        if not rclpy.ok():
            rclpy.init()

        # Store callback functions from the GymNode
        self.gymNode = GymNode.GymNode()
        exec_ = MultiThreadedExecutor(num_threads=4)
        exec_.add_node(self.gymNode)  # for the gym env
        spin_thread = threading.Thread(target=exec_.spin, daemon=True)
        spin_thread.start()

        # Variables to store ros subscriber data
        self.imu_data = 0.0
        self.cam_data = None

        self.random_pt = None
        self.previousDistance = None

        # self.localizationProc = subprocess.Popen(
        #     [
        #         "gnome-terminal",
        #         "--",
        #         "bash",
        #         "-c",
        #         "source /opt/ros/jazzy/setup.bash && "
        #         "source ~/mil2/install/setup.bash && "
        #         "ros2 launch subjugator_localization subjugator_localization.launch.py; exec bash",
        #     ],
        #     env=clean_ros_env(),
        # )

        # self.start_ekf_node()

        # self.proc.kill()
        # Wait for 10 seconds for gazebo to open
        time.sleep(10)

        # camera rgb space, Orientation+position, Linear/angular velocity comes from odometery/filtered
        self.observation_space = spaces.Dict(
            {
                "position": spaces.Box(low=-np.inf, high=np.inf, shape=(3,)),
                "orientation": spaces.Box(low=-1.0, high=1.0, shape=(4,)),
                "Linear_velocity": spaces.Box(low=-np.inf, high=np.inf, shape=(3,)),
                "angular_velocity": spaces.Box(low=-np.inf, high=np.inf, shape=(3,)),
                "object_distance": spaces.Box(
                    low=-100,
                    high=100,
                    shape=(3,),
                    dtype=np.float32,
                ),
            },
        )

        # First 3 are force, second 3 are torque
        self.action_space = spaces.Box(low=-50, high=50, shape=(6,), dtype=np.float32)

    def seed(self, seed=None):

        self.np_random, seed = gym.utils.seeding.np_random(seed)

        return [seed]

    def _reset_sub_pose(self):
        try:
            cmd = [
                "gz",
                "service",
                "-s",
                "/world/robosub_2024/set_pose",
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--req",
                'name: "sub9", position: {x: 0, y: 0, z: -0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}',
            ]

            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                print("Successfully reset pose")
                return True
            else:
                print(f"Failed to reset submarine: {result.stderr}")
                return False

        except subprocess.TimeoutExpired:
            print("Timeout while removing submarine")
            return False
        except Exception as e:
            print(f"Error removing submarine: {e}")
            return False

    def unpause_gazebo(self):
        try:
            cmd = [
                "gz",
                "service",
                "-s",
                "/world/robosub_2024/control",
                "--reqtype",
                "gz.msgs.WorldControl",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "3000",
                "--req",
                "pause: false",
            ]

            subprocess.run(cmd, timeout=5)

        except Exception as e:
            print(f"Could not unpause Gazebo: {e}")

    def pause_gazebo(self):
        try:
            cmd = [
                "gz",
                "service",
                "-s",
                "/world/robosub_2024/control",
                "--reqtype",
                "gz.msgs.WorldControl",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "3000",
                "--req",
                "pause: true",
            ]

            subprocess.run(cmd, timeout=5)

        except Exception as e:
            print(f"Could not pause Gazebo: {e}")

    def reset_localization(self):
        try:
            cmd = [
                "ros2",
                "service",
                "call",
                "/subjugator_localization/reset",
                "std_srvs/srv/Empty",
                "{ }",
            ]

            subprocess.run(cmd, timeout=5)

        except Exception as e:
            print(f"Could not start filter: {e}")

    def start_ekf_node(self):
        try:
            cmd = [
                "ros2",
                "service",
                "call",
                "/subjugator_localization/enable",
                "std_srvs/srv/Empty",
                "{ }",
            ]

            subprocess.run(cmd, timeout=5)

        except Exception as e:
            print(f"Could not start filter: {e}")

    def reset_ros_time(self):
        try:
            cmd = [
                "ros2",
                "topic",
                "pub",
                "/reset_time",
                "std_msgs/msg/Empty",
                "{}",
                "--once",
            ]
            subprocess.run(cmd, timeout=5)
            print("Reset ROS time")
        except Exception as e:
            print(f"Could not reset ROS time: {e}")

    def generate_random_pt(self):
        x = np.random.uniform(-5, 5)
        y = np.random.uniform(-5, 5)

        # can't go out of the water
        z = np.random.uniform(-5, 0)

        return np.array([x, y, z], dtype=np.float32)

    def calculate_red_amount(self, image):
        if image is None:
            return 0, 0, 0

        # Extract red channel (first channel in RGB)
        red_channel = image[:, :, 0]

        # Calculate metrics
        total_red = np.sum(red_channel)
        avg_red = np.mean(red_channel)

        # Calculate percentage (255 is max value for each pixel)
        max_possible_red = 255 * image.shape[0] * image.shape[1]
        red_percentage = (total_red / max_possible_red) * 100

        return total_red, avg_red, red_percentage

    def calculate_reward(self, observation):

        distance = np.linalg.norm(observation["object_distance"])

        # sub has reached target
        if distance < 1.0:
            return 1000

        if self.previousDistance is not None:
            progress = self.previousDistance - distance

            # rewards based on if sub is moving in the direction of the target
            progress_reward = progress * 10

        else:
            progress_reward = 0.0

        self.previousDistance = distance

        distance_reward = 1 / distance

        total_reward = progress_reward + distance_reward

        return total_reward

    def _get_obs(self):
        self.get_attempts += 1
        # Get image from RL_subscriber through thread - MUST CHECK FOR LOCK HERE

        # if cam_lock.acquire(False):
        #     self.cam_data = self.gymNode.cam_data  # get data from gymnode
        #     cam_lock.release()

        # imu version of above based on imu_node -- MUST CHECK FOR LOCK HERE
        if imu_lock.acquire(False):
            self.imu_gets += 1
            self.imu_data = self.gymNode.imu_data  # get data from gymnode
            imu_lock.release()

        # capture data
        pos = self.imu_data.pose.pose.position
        ori = self.imu_data.pose.pose.orientation
        linVel = self.imu_data.twist.twist.linear
        angVel = self.imu_data.twist.twist.angular

        # make it an array
        position = np.array([pos.x, pos.y, pos.z], dtype=np.float32)
        orientation = np.array([ori.x, ori.y, ori.z, ori.w], dtype=np.float32)
        linear_vel = np.array([linVel.x, linVel.y, linVel.z], dtype=np.float32)
        angular_vel = np.array([angVel.x, angVel.y, angVel.z], dtype=np.float32)

        if self.random_pt is not None:
            object_distance = self.random_pt - position
        else:
            object_distance = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        return {
            "position": position,
            "orientation": orientation,
            "Linear_velocity": linear_vel,
            "angular_velocity": angular_vel,
            "object_distance": object_distance,
        }

    def _get_info(self):
        print("Getting info")
        return {}

    def step(self, action):
        # Send action to the environment
        self.gymNode.publish_action_as_wrench(action)

        # Get observation from the environment (especially object_distance)
        observation = self._get_obs()

        # Get extra info (mostly for debugging)
        info = self._get_info()

        reward = self.calculate_reward(observation)

        # Define termination
        terminated = False

        # gets eucladian distance
        object_distance = np.linalg.norm(
            observation.get("object_distance", np.array([float("inf")] * 3)),
        )

        if object_distance < 2:
            terminated = True

        print("Reward: ", reward)
        print("Object distance: ", object_distance)
        print(f"Sub's Position={observation['position']}")
        print(f"Action value={action}")
        return observation, reward, terminated, False, info

    def reset(self, seed=None, options=None):
        print("RESET SUB ENV")

        super().reset(seed=seed)

        self.pause_gazebo()
        time.sleep(1)

        self.reset_localization()
        time.sleep(5)

        success = self._reset_sub_pose()
        time.sleep(5)
        # need to reset Rostime here by publishing to the  /reset_time topic and sending an empty msg to get rid of EKF error

        self.start_ekf_node()
        time.sleep(5)

        self.unpause_gazebo()
        time.sleep(5)

        self.random_pt = self.generate_random_pt()
        print(f"Generated target: {self.random_pt}")

        if not success:
            print("Gazebo service reset failed!")
            # You could raise an exception here if you want the environment to fail
            # raise RuntimeError("Failed to reset submarine using Gazebo services")

        # Small delay to let physics settle
        time.sleep(2)

        observation = self._get_obs()
        info = self._get_info()

        return observation, info

    def close(self):
        """Clean up resources"""
        if self.gazeboProc:
            self.gazeboProc.terminate()
            self.gazeboProc.wait()

        if self.localizationProc:
            self.localizationProc.kill()
            self.localizationProc.wait()

        if hasattr(self, "node"):
            self.node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
