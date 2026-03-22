import numpy as np

try:
    import gymnasium as gym

    _GYMNASIUM = True
except ImportError:  # fallback for older gym
    import gym

    _GYMNASIUM = False


SEED = 42
MAX_STEP_COUNT = 40  # TODO: Dynamically determine this value based on the average number of steps per demo.


class Environment(gym.Env):
    """
    Training environment for VAIRL algorithm with Gazebo.
    """

    def __init__(self):
        super().__init__()

        self.state = None
        self.t = 0
        self.rng = np.random.default_rng(SEED)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        """
        Place sub in random position based on the random spawn space of the project.
        """
        if seed is not None:
            self.rng = np.random.default_rng(seed)

        self.state = np.array(
            [
                # TODO: Get random position
            ],
            dtype=np.float32,
        )

        self.t = 0

        if _GYMNASIUM:
            return self.state.copy(), {}

        return self.state.copy()

    def step(self, action):
        """
        Move sub by sending a goal pose.
        """
        self.t += 1

        action = np.asarray(action, dtype=np.float32)
        next_state = self.state + action

        env_reward = 0.0

        # TODO: Publish a goal pose and wait for action to be completed

        # TODO: Determine if it hits an obstacle and deduct points

        self.state = next_state

        # TODO: Maybe determine condition for termination

        terminated = False
        truncated = self.t >= MAX_STEP_COUNT

        if _GYMNASIUM:
            return (
                next_state.copy(),
                float(env_reward),
                bool(terminated),
                bool(truncated),
                {},
            )

        done = bool(terminated or truncated)
        return next_state.copy(), float(env_reward), done, {}
