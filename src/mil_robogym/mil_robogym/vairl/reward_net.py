import torch
from imitation.util.networks import RunningNorm

try:
    import gymnasium as gym
except ImportError:
    import gym

GAMMA = 0.99
Z_SIZE = 6


class VAIRLRewardNet(RewardNet):
    """
    VAIRL reward network with variational encoders on g and h.
    f(s,a,s') = g(z_g(s,a)) + gamma*h(z_h(s')) - h(z_h(s))
    """

    def __init__(
        self,
        observation_space: gym.Space,  # TODO: Figure out how to set this up with the bounds of gazebo.
        action_space: gym.Space,
        normalize_input_layer=RunningNorm,
        gamma: float = GAMMA,
    ):
        try:
            super().__init__(observation_space, action_space)
        except TypeError:
            super().__init__()

        self.gamma = gamma
        self.obs_dim = int(np.prod(observation_space.shape))
        self.act_dim = int(np.prod(action_space.shape))

        self.obs_norm = (
            normalize_input_layer(observation_space.shape)
            if normalize_input_layer
            else None
        )
        self.act_norm = (
            normalize_input_layer(action_space.shape) if normalize_input_layer else None
        )

        self.enc_g = Encoder(self.obs_dim + self.act_dim)
        self.enc_h = Encoder(self.obs_dim)
        self.g = GNet(Z_SIZE)
        self.h = HNet(Z_SIZE)

    def _normalize(self, obs: torch.Tensor, acts: torch.Tensor):
        if self.obs_norm is not None:
            obs = self.obs_norm(obs)
        if self.act_norm is not None:
            acts = self.act_norm(acts)
        return obs, acts

    def forward_with_stats(
        self,
        obs: torch.Tensor,
        acts: torch.Tensor,
        next_obs: torch.Tensor,
        dones: torch.Tensor,
    ):
        obs, acts = self._normalize(obs, acts)
        next_obs = self.obs_norm(next_obs) if self.obs_norm is not None else next_obs

        sa = torch.cat([obs, acts], dim=1)
        z_g, mu_g, logvar_g = self.enc_g(sa)
        z_h, mu_h, logvar_h = self.enc_h(obs)
        z_hn, mu_hn, logvar_hn = self.enc_h(next_obs)

        f = self.g(z_g) + self.gamma * self.h(z_hn) - self.h(z_h)
        stats = (mu_g, logvar_g, mu_h, logvar_h, mu_hn, logvar_hn)
        return f, stats

    def forward(self, obs, acts, next_obs, dones):
        f, _stats = self.forward_with_stats(obs, acts, next_obs, dones)
        return f.squeeze(-1)
