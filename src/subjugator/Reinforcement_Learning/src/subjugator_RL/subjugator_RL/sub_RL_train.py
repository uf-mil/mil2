from subjugator_RL import sub_env
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv


def make_env(env_id, rank, seed=0):

    def _init():
        env = sub_env.SubEnv()
        env.seed(seed + rank)
        return env

    return _init


def main():

    num_cpu = 1

    env = SubprocVecEnv([make_env("subRL", i) for i in range(num_cpu)])

    model = PPO("MultiInputPolicy", env, verbose=1, ent_coef=0.1,learning_rate=0.0001, batch_size=512)

    print("model is training...")

    model.learn(total_timesteps=500000,progress_bar=True)

    print("training compelete")

    model.save("subRL model")

    print("model saved!")
