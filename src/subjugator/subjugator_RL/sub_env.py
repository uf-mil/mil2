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

<<<<<<< HEAD
=======
    # Is threading needed? Might add unnecessary computation that could slow down training --
    imu_subscriber.run()
    cam_subscriber.run()
    # Example of getting data: data = imu_subscriber.imu_node.imu_data

>>>>>>> ceb3dc351a9e992f5700afb762676984a7c2a3b2
    def __init__(self, render_mode = "rgb_array"):

        #IMU field
        imu_thread = threading.Thread(target=imu_main)
        imu_thread.daemon = True  # The thread will exit when the main program exits
        imu_thread.start()

        #camera rgb space
        self.observation_space = spaces.Dict({
                'image'                 : spaces.Box(0, 255, shape=(SHAPE[0], SHAPE[1], SHAPE[2]), dtype=np.uint8),
<<<<<<< HEAD
                'orientation': spaces.Box(low=-1.0, high=1.0, shape=(4,)),
                'angular_velocity': spaces.Box(low=-np.inf, high=np.inf, shape=(3,)),
                'linear_acceleration': spaces.Box(low=-np.inf, high=np.inf, shape=(3,))
        })
=======
                'orientation'           : spaces.Box(low = -np.inf, high = np.inf, shape=(4,))
                'object_distance'       : spaces.Box(low = 0, high = 100, shape=(1,), dtype=np.float32),
            })
>>>>>>> ceb3dc351a9e992f5700afb762676984a7c2a3b2


        #Force_x
        self.action_space = spaces.Dict( 
            {'force'  : spaces.Box(low=-50, high=50, shape=(3,), dtype=np.float32),
             'torque' : spaces.Box(low=-50, high=50, shape=(3,), dtype=np.float32)
            })

        self.imu_node = ImuSubscriber()
<<<<<<< HEAD

       

        # self.sketch = None
        # self.setup = True
=======
>>>>>>> ceb3dc351a9e992f5700afb762676984a7c2a3b2



    def _get_obs(self):
<<<<<<< HEAD
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
=======
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
>>>>>>> ceb3dc351a9e992f5700afb762676984a7c2a3b2


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

<<<<<<< HEAD
    def calculate_reward(self, observation):
        image = observation['image']

        total_red, avg_red, red_percentage = self.calculate_red_amount(image)

        reward = red_percentage #trying to mamximize the red percentage so that it goes to the red bouy




    def step(self, action):
        self._publish_action_as_wrench(self, action)


        #if()
        #reward
        observation = self._get_obs(self)
=======
>>>>>>> ceb3dc351a9e992f5700afb762676984a7c2a3b2
# subscriber for IMU
# Register the environment in the gym
register(
    id="SubjugatorAgent-v0",
    entry_point=SubEnv
)
