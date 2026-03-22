def register_env() -> None:
    try:
        if _GYMNASIUM:
            from gymnasium.envs.registration import register
        else:
            from gym.envs.registration import register

        register(
            id=ENV_ID,
            entry_point=Vairl3DEnv,
            max_episode_steps=MAX_STEP_COUNT,
        )
    except Exception:
        # Already registered or gym registry unavailable; ignore
        pass


def _maybe_add_noise(action: np.ndarray, noise_std: float) -> np.ndarray:
    if noise_std <= 0:
        return action
    return action + np.random.normal(scale=noise_std, size=action.shape)


# TODO: Have the CSV writer handle returning a list of imitations
def trajectories_to_imitation(
    trajectories: List[List[Tuple[np.ndarray, np.ndarray, np.ndarray]]],
):
    trajs = []
    for traj in trajectories:
        obs = [step[0] for step in traj]
        obs.append(traj[-1][2])
        acts = [step[1] for step in traj]
        obs_arr = np.asarray(obs, dtype=np.float32)
        acts_arr = np.asarray(acts, dtype=np.float32)
        trajs.append(
            imitation_types.Trajectory(
                obs=obs_arr,
                acts=acts_arr,
                infos=None,
                terminal=True,
            ),
        )
    return trajs


def total_kl(stats: Tuple[torch.Tensor, ...]) -> torch.Tensor:
    mu_g, logvar_g, mu_h, logvar_h, mu_hn, logvar_hn = stats
    return (
        kl_divergence(mu_g, logvar_g)
        + kl_divergence(mu_h, logvar_h)
        + kl_divergence(mu_hn, logvar_hn)
    ).mean()


def trajectories_to_batch(trajs: List[List[Tuple[np.ndarray, np.ndarray, np.ndarray]]]):
    obs_list = []
    acts_list = []
    next_obs_list = []
    dones_list = []

    for traj in trajs:
        for i, (s, a, s_next) in enumerate(traj):
            obs_list.append(s)
            acts_list.append(a)
            next_obs_list.append(s_next)
            dones_list.append(1.0 if i == len(traj) - 1 else 0.0)

    return {
        "obs": np.asarray(obs_list, dtype=np.float32),
        "acts": np.asarray(acts_list, dtype=np.float32),
        "next_obs": np.asarray(next_obs_list, dtype=np.float32),
        "dones": np.asarray(dones_list, dtype=np.float32),
    }


# TODO: Place this function inside the trainer
def generate_generator_trajectories(
    generator: TRPO,
    reward_net: VAIRLRewardNet,
    env: Vairl3DEnv,
    n_trajs: int = 20,
):
    trajectories = []
    rewards = []

    for _ in range(n_trajs):
        if _GYMNASIUM:
            state, _ = env.reset()
        else:
            state = env.reset()

        traj = []
        reward_accumulated = 0.0

        for _ in range(MAX_STEP_COUNT):
            action, _ = generator.predict(state, deterministic=False)
            action = np.asarray(action, dtype=np.float32)

            if _GYMNASIUM:
                next_state, _env_reward, terminated, truncated, _ = env.step(action)
                done = terminated or truncated
            else:
                next_state, _env_reward, done, _ = env.step(action)

            with torch.no_grad():
                s_t = torch.tensor(state, dtype=torch.float32).unsqueeze(0)
                a_t = torch.tensor(action, dtype=torch.float32).unsqueeze(0)
                s_next_t = torch.tensor(next_state, dtype=torch.float32).unsqueeze(0)
                d_t = torch.tensor([float(done)], dtype=torch.float32).unsqueeze(1)
                reward = reward_net(s_t, a_t, s_next_t, d_t).item()

            dist = np.linalg.norm(next_state - GOAL)
            if done and dist <= 0.5:
                reward += 1.0

            reward_accumulated += reward
            traj.append((state.copy(), action.copy(), next_state.copy()))

            state = next_state

            if done:
                if _GYMNASIUM:
                    state, _ = env.reset()
                else:
                    state = env.reset()

        rewards.append(reward_accumulated)
        trajectories.append(traj)

    print(f"Reward mean  : {np.mean(rewards):.3f}")
    print(f"Reward std   : {np.std(rewards):.3f}")

    return trajectories, float(np.mean(rewards)), float(np.std(rewards))
