from subjugator_RL import sub_env
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv

import matplotlib.pyplot as plt
plt.show(block=True)

def make_env(env_id, rank, seed=0):

    def _init():
        env = sub_env.SubEnv()
        env.seed(seed + rank)
        return env

    return _init


def main():
    # plt.ion()
    # plt.plot(1, 2, label='entropy')
    # plt.show(block=True)

    num_cpu = 1

    env = SubprocVecEnv([make_env("subRL", i) for i in range(num_cpu)])

    model = PPO("MultiInputPolicy", env, verbose=1, ent_coef=0.1,learning_rate=0.01, batch_size=16, n_steps=16)

    print("model is training...")

    model.learn(total_timesteps=512,progress_bar=True)
    
    
    plt.plot(model.xData, model.y1Data, '-ro', label='entropy_loss', linewidth=2)
    plt.plot(model.xData, model.y2Data, '-bo', label='gradient_loss', linewidth=2)
    plt.plot(model.xData, model.y3Data, '-go', label='value_loss', linewidth=2)
    plt.plot(model.xData, model.y4Data, '-yo', label='kl', linewidth=2)

    plt.legend()

    print(plt.savefig("trainingPlot.png"))
    
    print(plt.show(block=True))

    print("training compelete")

    model.save("subRL model")

    print("model saved!")
