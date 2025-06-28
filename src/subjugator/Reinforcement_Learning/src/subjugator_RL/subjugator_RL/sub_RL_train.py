# import gym
# import subjugator_RL
# from stable_baselines3 import PPO
# from stable_baselines3.common.vec_env import SubprocVecEnv
# from stable_baselines3.common.utils import set_random_seed



# def make_env(env_id, rank, seed=0):
    
#     def _init():
#         env = subjugator_RL.SubEnv()
#         env.seed(seed + rank)
#         return env

#     return _init


# if __name__ == "__main__":

#     num_cpu = 1
    
#     env = SubprocVecEnv([make_env("subRL", i) for i in range(num_cpu)])
    
#     model = PPO("MlpPolicy", env, verbose=1)

#     print("model is training...")

#     model.learn(total_timesteps=50000)

#     print("training compelete")

#     model.save("subRL model")

#     print("model saved!")
    

import time
import numpy as np
from multiprocessing import Process
import rclpy
from subjugator_RL.GymNode import GymNode

def main():
    # Initialize the ROS node via GymNode
    gym_node = GymNode()
    
    def train():
        try:
            print("Environment reset successfully!")

            # Loop with random actions
            for i in range(1000000):
                time.sleep(0.5)
                action = {
                    "force": np.random.uniform(0, 10, 3),
                    "torque": np.random.uniform(0, 10, 3),
                }
                obs, reward, terminated, truncated, info = gym_node.subEnv.step(action)
                print(f"Step {i}: Reward = {reward}")
                if obs.get("imu") is not None:
                    imu_lin_x = obs["imu"].twist.twist.linear.x
                    print(f"Step {i}: Filtered odom = {imu_lin_x}")

                if terminated:
                    obs, info = gym_node.subEnv.reset()
                    print("Episode terminated, reset environment")
        finally:
            gym_node.subEnv.close()

    # Run the manual loop in a separate process
    p = Process(target=train)
    p.start()

    # Spin the ROS node to handle subscriptions/publishers
    rclpy.spin(gym_node)


if __name__ == "__main__":
    rclpy.init()
    main()
