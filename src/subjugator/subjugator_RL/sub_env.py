import gym
from gym import spaces
import numpy as np
import random
import time
import os

# Shape of the image. L, W, # of channels
SHAPE = [50,80,3]

#to be finished
class SubEnv(gym.Env):
    def __init__(self, render_mode = "rgb_array"):

	self.observation_space = spaces.Dict({
            'image': spaces.Box(0, 255, shape=(SHAPE[0], SHAPE[1], SHAPE[2]), dtype=np.uint8),
        })

	self.sketch = None
	self.setup = True

	# self._action_to_direction = ?

    def _get_obs(self):
        # Get image from RL_subscriber through pipe
        with open("image_pipe", 'rb') as pipe:
            img_data = pipe.read(SHAPE[0]*SHAPE[1]*SHAPE[2])
            if img_data:
                image = np.frombuffer(img_data, dtype=np.uint8).reshape((SHAPE[0], SHAPE[1], SHAPE[2]))
	return {"image": image}
		    


# Register the environment in the gym
register(
    id="SubjugatorAgent-v0",
    entry_point=SubEnv
)
