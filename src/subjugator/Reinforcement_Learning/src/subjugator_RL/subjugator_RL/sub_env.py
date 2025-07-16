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
from subjugator_RL.dict_helpers import dicts_equal, nan_to_zero_in_dict

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
        self.previousObservation = None

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
        # Wait for 5 seconds for gazebo to open
        time.sleep(5)

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
        self.action_space = spaces.Box(low=-100, high=100, shape=(6,), dtype=np.float32)

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

    def _spawn_buoy(self, x: float, y: float, z: float) -> bool:
        """
        Calls /world/robosub_2024/create to spawn buoy_2024 at (x, y, z)
        with a default unit-quaternion orientation.
        """
        # Build the EntityFactory request string
        req = (
            f'name: "TARGET_BUOY"; '
            f'sdf_filename: "package://subjugator_description/models/buoy_2024/buoy.sdf"; '
            f'pose: {{ position: {{ x: {x}, y: {y}, z: {z} }}, '
            f'orientation: {{ x: 0.0, y: 0.0, z: 0.0, w: 1.0 }} }}'
        )

        cmd = [
            "gz", "service",
            "--service", "/world/robosub_2024/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", req
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                print(f"Successfully spawned buoy at x={x}, y={y}, z={z}")
                return True
            else:
                print(f"Failed to spawn buoy: {result.stderr.strip()}")
                return False

        except subprocess.TimeoutExpired:
            print("Create service call timed out")
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


    def reset_simulation(self):
        try:
            cmd = "gz service -s /world/robosub_2024/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'reset: {all: true}'"
            subprocess.run(cmd, shell=True, timeout=5, check=True)
            print("Reset sim successfully")
            return True
        except Exception as e:
            print(f"Could not reset sim: {e}")
            return False

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
        # Generate random point greater than min_x or y, but less than max_x or y
        min_x, max_x = 3, 6
        min_y, max_y = 3, 6
        x = np.where(np.random.rand() < 0.5, np.random.uniform(-min_x, -max_x), np.random.uniform(min_x, max_x))
        y = np.where(np.random.rand() < 0.5, np.random.uniform(-min_y, -max_y), np.random.uniform(min_y, max_y))

        # sub is bad at changing its depth, -0.2 is default
        z = -0.12 #np.random.uniform(-1.5, 0)

        # also put the point as a buoy into gazebo
        self._spawn_buoy(x, y, z)

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
        if distance < 0.5:
            return 10000

        if self.previousDistance is not None:
            progress = self.previousDistance - distance
            # rewards based on if sub is moving in the direction of the target
            print(f"PROGESS: {progress}")
            # higher negative penalty for negative progress
            progress *= 2 if progress < 0 else 1
            # clip progress in case of sudden jerk, increase progress reward to meaningful amount
            progress_reward = 100 * np.clip(progress, -0.05, 0.05)

        else:
            progress_reward = 0

        self.previousDistance = distance

        distance_reward = ((5/distance) ** 2) - 1

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

        # print(position)
        # print(orientation)
        # print(linear_vel)
        # print(angular_vel)
        # print(object_distance)

        if self.random_pt is not None:
            object_distance = self.random_pt - position
        else:
            object_distance = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        return nan_to_zero_in_dict({
            "position": position,
            "orientation": orientation,
            "Linear_velocity": linear_vel,
            "angular_velocity": angular_vel,
            "object_distance": object_distance,
        })

    def _get_info(self):
        print("Getting info")
        return {}

    def step(self, action):
        # Send action to the environment
        self.gymNode.publish_action_as_wrench(action)

        # Get observation logic
        observation = None
        # How many steps env does env calculate per second?
        # hertz_div determines that: env_hertz = gazebo simulation updates per second / hertz_div 
        hertz_div = 10
        for i in range(hertz_div):
            # Get observation, retry until actually new observation is returned
            observation = self._get_obs()
            while(observation is not None and self.previousObservation is not None and dicts_equal(observation, self.previousObservation)): 
                observation = self._get_obs()
                time.sleep(0.005)
            self.previousObservation = observation

        # Get extra info (mostly for debugging)
        info = self._get_info()

        reward = self.calculate_reward(observation)

        # gets eucladian distance
        object_distance = np.linalg.norm(
            observation.get("object_distance", np.array([float("inf")] * 3)),
        )

        # -- Termination conditions --
        terminated = False # Define termination
        position = observation["position"]
        x, y = position[0], position[1]
        # Terminate on success
        if object_distance < 0.5:
            print(f"SUB ENV SUCCESS at x={x:.2f}, y={y:.2f}")
            terminated = True
        # Terminate if sub veers out too far or glitches off map
        if not (-10.5 < x < 10.5 and -24 < y < 24):
            print(f"Terminated: Out of bounds at x={x:.2f}, y={y:.2f}")
            terminated = True

        print("Reward: ", reward)
        print(object_distance, " - Object distance")
        print(f"Sub's Position={observation['position']}")
        print(f"Action value={action}")
        return observation, reward, terminated, False, info

    def reset(self, seed=None, options=None):
        print("RESET SUB ENV")

        super().reset(seed=seed)

        print("Attempting to pause sim")
        # Gazebo automatically pauses on reset
        success = self.pause_gazebo()
        print("Attempting to reset sim")
        success = self.reset_simulation()
        time.sleep(0.5)
        print("Attempting to reset localization")
        self.reset_localization()
        self.start_ekf_node()
        time.sleep(0.5)
        self.unpause_gazebo()
        # need to reset Rostime here by publishing to the  /reset_time topic and sending an empty msg to get rid of EKF error

        # Reset class variables
        self.random_pt = self.generate_random_pt()
        print(f"Generated target: {self.random_pt}")
        self.previousDistance = 0
        self.previousObservation = None

        if not success:
            print("Gazebo service reset failed!")
            # You could raise an exception here if you want the environment to fail
            # raise RuntimeError("Failed to reset submarine using Gazebo services")

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
