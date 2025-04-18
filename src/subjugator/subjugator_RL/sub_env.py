import gym
from gym import spaces
import numpy as np
from geometry_msgs.msg import Wrench, Vector3
from sensor_msgs.msg import Imu
import random
import time
import os
import threading
from imu_subscriber import ImuSubscriber
from imu_subscriber import main as imu_main

# Shape of the image. L, W, # of channels
SHAPE = [50,80,3]

#to be finished
class SubEnv(gym.Env):

    def __init__(self, render_mode = "rgb_array"):

        #IMU field
        imu_thread = threading.Thread(target=imu_main)
        imu_thread.daemon = True  # The thread will exit when the main program exits
        imu_thread.start()

        #camera rgb space
        self.observation_space = spaces.Dict({
                'image'                 : spaces.Box(0, 255, shape=(SHAPE[0], SHAPE[1], SHAPE[2]), dtype=np.uint8),
                'orientation': spaces.Box(low=-1.0, high=1.0, shape=(4,)),
                'angular_velocity': spaces.Box(low=-np.inf, high=np.inf, shape=(3,)),
                'linear_acceleration': spaces.Box(low=-np.inf, high=np.inf, shape=(3,))
        })


        #Force_x
        self.action_space = spaces.Dict( 
            {'force'  : spaces.Box(low=-50, high=50, shape=(3,), dtype=np.float32),
             'torque' : spaces.Box(low=-50, high=50, shape=(3,), dtype=np.float32)
            })

        self.imu_node = ImuSubscriber()

       

        # self.sketch = None
        # self.setup = True



    def _get_obs(self):
        # Get image from RL_subscriber through pipe
        try:
            with open("image_pipe", 'rb') as pipe:
                img_data = pipe.read()
                
        except:
            image = None
        
        imu_data = self.imu_node.get_latest_data()

        if img_data:
            image = np.frombuffer(img_data, dtype=np.uint8).reshape((SHAPE[0], SHAPE[1], SHAPE[2]))
        else:
            image = np.zeros(img_data, dtype=np.uint8).reshape((SHAPE[0], SHAPE[1], SHAPE[2]))
        
        if imu_data:
            return {
                "image": image,
                "orientation": np.array(imu_data['orientation']),  # This is  quaternion
                "angular_velocity": np.array(imu_data['angular_velocity']),
                "linear_acceleration": np.array(imu_data['linear_acceleration'])
            }
        else:
        # Default vals if no IMU data is available
            return {
                "image": image,
                "orientation": np.array([0.0, 0.0, 0.0, 1.0]),  # quaternion [x,y,z,w]
                "angular_velocity": np.zeros(3),
                "linear_acceleration": np.zeros(3)
            }
        

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

    def calculate_reward(self, observation):
        image = observation['image']

        total_red, avg_red, red_percentage = self.calculate_red_amount(image)

        reward = red_percentage #trying to mamximize the red percentage so that it goes to the red bouy




    def step(self, action):
        self._publish_action_as_wrench(self, action)


        #if()
        #reward
        observation = self._get_obs(self)
# subscriber for IMU
# Register the environment in the gym

register(
    id="SubjugatorAgent-v0",
    entry_point=SubEnv
)
