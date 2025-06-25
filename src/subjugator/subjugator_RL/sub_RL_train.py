import gym
import sub_env
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.utils import set_random_seed



def make_env(env_id, rank, seed=0):
    
    def _init():
        env = sub_env.SubEnv()
        env.seed(seed + rank)
        return env

    return _init


if __name__ == "__main__":

    num_cpu = 1
    
    env = SubprocVecEnv([make_env("subRL", i) for i in range(num_cpu)])
    
    model = PPO("MlpPolicy", env, verbose=1)

    print("model is training...")

    model.learn(total_timesteps=50000)

    print("training compelete")

    model.save("subRL model")

    print("model saved!")
    