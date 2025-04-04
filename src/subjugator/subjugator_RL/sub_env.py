import gym
from gym import spaces
import numpy as np
from geometry_msgs.msg import Wrench, Vector3
import random
import time
import os

# Shape of the image. L, W, # of channels
SHAPE = [50,80,3]

#to be finished
class SubEnv(gym.Env):
    def __init__(self, render_mode = "rgb_array"):

        #camera rgb space
        self.observation_space = spaces.Dict({
                'image'                 : spaces.Box(0, 255, shape=(SHAPE[0], SHAPE[1], SHAPE[2]), dtype=np.uint8),
                'orientation'           : spaces.Box(low = -np.inf, high = np.inf, shape=(4,))

            })


        #Force_x
        self.action_space = spaces.Dict( 
            {'force'  : spaces.Box(low=-50, high=50, shape=(3,), dtype=np.float32),
             'torque' : spaces.Box(low=-50, high=50, shape=(3,), dtype=np.float32)
            })


        # self.sketch = None
        # self.setup = True



    def _get_obs(self):
        # Get image from RL_subscriber through pipe
        
        with open("image_pipe", 'rb') as pipe:
            img_data = pipe.read(SHAPE[0]*SHAPE[1]*SHAPE[2])
            if img_data:
                image = np.frombuffer(img_data, dtype=np.uint8).reshape((SHAPE[0], SHAPE[1], SHAPE[2]))
        
        return {"image": image, }
    




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
