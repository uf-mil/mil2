import gym
from gym import spaces
import numpy as np
import random
import time


class subEnv(gym.Env):
	def _init_(self, render_mode = "rgb_array"):
		self.observation_space = spaces.Box(0, 255, shape(200, 266), dtype=np.uint8)
		
