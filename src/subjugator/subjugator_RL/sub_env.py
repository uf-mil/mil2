import gym
from gym import spaces
import numpy as np
from geometry_msgs.msg import Wrench, Vector3
from sensor_msgs.msg import Imu
import random
import time
import os
import threading
import imu_subscriber
import cam_subscriber
import subprocess
import os, subprocess, pathlib
from rclpy.executors import SingleThreadedExecutor

# Shape of the image. L, W, # of channels
SHAPE = [50,80,3]

# ROS environment crashes in spanwed terminal without using this
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

    # 3) Ensure XDG_RUNTIME_DIR is sane (needed by rclcpp logging)
    env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    pathlib.Path(env["XDG_RUNTIME_DIR"]).mkdir(parents=True, exist_ok=True)

    return env

#to be finished
class SubEnv(gym.Env):   
    proc = None

    # Example of getting data: data = imu_subscriber.imu_node.imu_data

    def __init__(self, render_mode = "rgb_array"):
        
        # Run the launch file to reset the gazebo
        self.proc = subprocess.Popen(
            [
                "gnome-terminal", "--",
                "bash", "-c",
                "source /opt/ros/jazzy/setup.bash && "
                "source ~/mil2/install/setup.bash && "
                "ros2 launch subjugator_bringup gazebo.launch.py; exec bash"
            ],
            env=clean_ros_env(),
        )

        cam_subscriber.run()
        imu_subscriber.run()

        # Wait for 15 seconds for gazebo to open
        time.sleep(15)

        #camera rgb space
        self.observation_space = spaces.Dict({
                'image'                 : spaces.Box(0, 255, shape=(SHAPE[0], SHAPE[1], SHAPE[2]), dtype=np.uint8),
                'orientation': spaces.Box(low=-1.0, high=1.0, shape=(4,)),
                'angular_velocity': spaces.Box(low=-np.inf, high=np.inf, shape=(3,)),
                'linear_acceleration': spaces.Box(low=-np.inf, high=np.inf, shape=(3,)),
                'object_distance'       : spaces.Box(low = 0, high = 100, shape=(1,), dtype=np.float32),
            })


        #Force_x
        self.action_space = spaces.Dict( 
            {'force'  : spaces.Box(low=-50, high=50, shape=(3,), dtype=np.float32),
             'torque' : spaces.Box(low=-50, high=50, shape=(3,), dtype=np.float32)
            })



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
        image = observation['image']

        total_red, avg_red, red_percentage = self.calculate_red_amount(image)

        reward = red_percentage #trying to mamximize the red percentage so that it goes to the red bouy

        return reward
        

    def _get_obs(self):
        # Get image from RL_subscriber through thread
        image = cam_subscriber.cam_node.cam_data
        
        # imu version of above based on imu_node --
        imu = imu_subscriber.imu_node.imu_data
        
        # Get object distance via the buoy_finder (either through subscriber thread, or pipe) --       
        
        # object_distance has yet to be implemented --
        return {"image": image, "imu": imu, "object_distance": object_distance }
    
    # Unneeded, can be used for debugging
    def _get_info(self):
        print("Getting info")


    def step(self, action):
        # Send action to the environment
        self._publish_action_as_wrench(self, action)

        # Get observation from the environment (especially object_distance)
        observation = self._get_obs(self)

        # Get extra info (mostly for debugging)
        info = self._get_info(self)

        # Define rules for rewarding - must be within range of 2 and 5 meters of object
        # if (self.object_distance > 2 or self.object_distance < 5):
        #     reward = 1
        # else:
        #     reward = 0

        reward = self.calculate_reward(observation)

        # Also define termination (when to end)
        terminated = False
        if (self.object_distance < 2):
            terminated = True
    
        return observation, reward, terminated, False, info
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

            # Kill the previous gazebo process if it exists
        if self.proc:
            self.proc.terminate()
            self.proc.wait()  # Wait for it to close properly

        # Run the launch file to reset the gazebo
        command = ["ros2", "launch", "subjugator_bringup", "gazebo.launch.py"]
        subprocess.run(command, capture_output=True, text=True, check=True)

        # Wait for 15 seconds for gazebo to open
        time.sleep(15)

        observation = self._get_obs(self)
        info = self._get_info(self)

        return observation, info


    def _publish_action_as_wrench(self, action):
        force_action  = action['force']
        torque_action  = action['torque']
        wrench_msg = Wrench()

        wrench_msg.force = Vector3(
             
            x=float(force_action[0]),
            y=float(force_action[1]),
            z=float(force_action[2])
        )

        wrench_msg.torque = Vector3(
             torque_action[0],
             torque_action[1],
             torque_action[2]
        )

        with open("wrench_msg_pipe", 'wb') as pipe:
            print("Writing to wrench publisher")
            pipe.write(wrench_msg.tobytes())

# subscriber for IMU
# Register the environment in the gym
# register(
#     id="SubjugatorAgent-v0",
#     entry_point=SubEnv
# )


if __name__ == '__main__':
    env = SubEnv()
    env.reset()
