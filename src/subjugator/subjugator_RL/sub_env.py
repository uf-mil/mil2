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

# Shape of the image. L, W, # of channels
SHAPE = [50,80,3]

#to be finished
class SubEnv(gym.Env):

    # Is threading needed? Might add unnecessary computation that could slow down training --
    imu_subscriber.run()
    cam_subscriber.run()
    # Example of getting data: data = imu_subscriber.imu_node.imu_data

    def __init__(self, render_mode = "rgb_array"):

        #camera rgb space
        self.observation_space = spaces.Dict({
                'image'                 : spaces.Box(0, 255, shape=(SHAPE[0], SHAPE[1], SHAPE[2]), dtype=np.uint8),
                'orientation'           : spaces.Box(low = -np.inf, high = np.inf, shape=(4,))
                'object_distance'       : spaces.Box(low = 0, high = 100, shape=(1,), dtype=np.float32),
            })


        #Force_x
        self.action_space = spaces.Dict( 
            {'force'  : spaces.Box(low=-50, high=50, shape=(3,), dtype=np.float32),
             'torque' : spaces.Box(low=-50, high=50, shape=(3,), dtype=np.float32)
            })

        self.imu_node = ImuSubscriber()



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
        if (self.object_distance < 2 or self.object_distance > 5):
            reward = 1
        else:
            reward = 0

        # Also define termination (when to end)
        terminated = False
        if (self.object_distance < 2){
            terminated = True
        }
    
        return observation, reward, terminated, False, info
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # Find initial position of sub and place it there (if possible within the ROS environment, or simply send it to initial position) --

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
register(
    id="SubjugatorAgent-v0",
    entry_point=SubEnv
)
