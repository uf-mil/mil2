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

    #Retrian and add rewards onto the plaotting for the graph
    #add a pentaly for taking a lot of timesteps. 
    num_cpu = 1

    env = SubprocVecEnv([make_env("subRL", i) for i in range(num_cpu)])

    #model = PPO("MultiInputPolicy", env, verbose=1, ent_coef=0.01,learning_rate=0.0001, batch_size=128, n_steps=2048, clip_range=0.1, max_grad_norm=0.5)

    model = PPO.load("/home/mohana/mil2/subRL model.zip")

    model.set_env(env)

    print("model is training...")

    #model.learn(total_timesteps=1000000,progress_bar=True)
    model.learn(total_timesteps=50000,progress_bar=True)
    
    print("training compelete")

    model.save("subRL model")

    print("model saved!")
    
    plt.plot(model.xData, model.y1Data, '-ro', label='entropy_loss', linewidth=2)
    plt.plot(model.xData, model.y2Data, '-bo', label='gradient_loss', linewidth=2)
    plt.plot(model.xData, model.y3Data, '-go', label='value_loss', linewidth=2)
    plt.plot(model.xData, model.y4Data, '-yo', label='kl', linewidth=2)

    plt.legend()

    print(plt.savefig("trainingPlot.png"))
    
    print(plt.show(block=True))

    
