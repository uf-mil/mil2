from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from subjugator_RL import sub_env
import numpy as np


def make_env(env_id, rank, seed=0):

    def _init():
        env = sub_env.SubEnv()
        env.seed(seed + rank)
        return env

    return _init


def main():

    
    num_cpu = 1

    env = SubprocVecEnv([make_env("subRL", i) for i in range(num_cpu)])

    model = PPO.load("/home/mohana/mil2/subRL model.zip")

    num_episodes = 10 
    successes = 0
    total_steps = 0
    total_rewards = []

    for episode in range(num_episodes):

        obs = env.reset()

        episode_reward  = 0 

        episode_steps = 0

        done = False

        while not done:
            action, states = model.predict(obs, deterministic=True)

            obs, reward, done, info = env.step(action) ## taking action in env

            episode_steps+=1
            episode_reward += reward

            if episode_steps > 20000:
                print(f"Episode timeout after {episode_steps} steps")
                break

        total_steps += episode_steps
        total_rewards.append(episode_reward)

        distance = np.linalg.norm(obs["object_distance"])
        print(f"Test obj distance: {distance}")
        
        if distance < 0.5:
            successes += 1
            print(f"Episode {episode}: SUCCESS in {episode_steps} steps, reward: {episode_reward}")
        else:
            print(f"Episode {episode}: FAILED in {episode_steps} steps, reward: {episode_reward}")
            

    success_rate = successes / num_episodes
    avg_steps = total_steps / num_episodes
    avg_reward = sum(total_rewards) / num_episodes

    print(f"\n=== RESULTS ===")
    print(f"Success Rate: {success_rate:.1%} ({successes}/{num_episodes})")
    print(f"Average Steps: {avg_steps:.1f}")
    print(f"Average Reward: {avg_reward:.1f}")










 