import torch.optim as optim

from .reward_net import VAIRLRewardNet

I_C = 0.5
BETA_STEP_SIZE = 1e-3
DISCRIMINATOR_LEARNING_RATE = 3e-3


class VAIRL(AIRL):
    """
    VAIRL algorithm.
    """

    def __init__(
        self,
        *,
        demonstrations,
        demo_batch_size: int,
        venv,
        gen_algo,
        reward_net: VAIRLRewardNet,
        beta: float = 0.0,
        i_c: float = I_C,
        beta_step_size: float = BETA_STEP_SIZE,
        disc_lr: float = DISCRIMINATOR_LEARNING_RATE,
        **kwargs,
    ):
        super().__init__(
            demonstrations=demonstrations,
            demo_batch_size=demo_batch_size,
            venv=venv,
            gen_algo=gen_algo,
            reward_net=reward_net,
            **kwargs,
        )

        self.vairl_reward_net = reward_net
        self.beta = float(beta)
        self.i_c = float(i_c)
        self.beta_step_size = float(beta_step_size)

        self.device = getattr(gen_algo, "device", torch.device("cpu"))
        self.vairl_reward_net.to(self.device)

        self._vairl_disc_opt = optim.Adam(
            self.vairl_reward_net.parameters(),
            lr=disc_lr,
        )

    def train_disc(
        self,
        *,
        expert_samples: dict[str, np.ndarray] | None = None,
        gen_samples: dict[str, np.ndarray] | None = None,
    ) -> dict[str, float]:

        if expert_samples is None or gen_samples is None:
            raise ValueError(
                "VAIRL.train_disc requires explicit expert and gen samples",
            )

        def to_torch(x):
            return torch.as_tensor(x, dtype=torch.float32, device=self.device)

        obs_e = to_torch(expert_samples["obs"])
        acts_e = to_torch(expert_samples["acts"])
        next_obs_e = to_torch(expert_samples["next_obs"])
        dones_e = to_torch(expert_samples["dones"]).unsqueeze(1)

        obs_g = to_torch(gen_samples["obs"])
        acts_g = to_torch(gen_samples["acts"])
        next_obs_g = to_torch(gen_samples["next_obs"])
        dones_g = to_torch(gen_samples["dones"]).unsqueeze(1)

        f_e, stats_e = self.vairl_reward_net.forward_with_stats(
            obs_e,
            acts_e,
            next_obs_e,
            dones_e,
        )
        f_g, stats_g = self.vairl_reward_net.forward_with_stats(
            obs_g,
            acts_g,
            next_obs_g,
            dones_g,
        )

        with torch.no_grad():
            _, log_prob_e, _ = self.gen_algo.policy.evaluate_actions(obs_e, acts_e)
            _, log_prob_g, _ = self.gen_algo.policy.evaluate_actions(obs_g, acts_g)

        if log_prob_e.ndim == 1:
            log_prob_e = log_prob_e.unsqueeze(1)
        if log_prob_g.ndim == 1:
            log_prob_g = log_prob_g.unsqueeze(1)

        logits_e = f_e - log_prob_e
        logits_g = f_g - log_prob_g

        labels_e = torch.ones_like(logits_e)
        labels_g = torch.zeros_like(logits_g)

        loss_disc = F.binary_cross_entropy_with_logits(logits_e, labels_e)
        loss_disc = loss_disc + F.binary_cross_entropy_with_logits(logits_g, labels_g)

        kl = 0.5 * (total_kl(stats_e) + total_kl(stats_g))
        bottleneck = kl - self.i_c

        loss = loss_disc + self.beta * bottleneck

        self._vairl_disc_opt.zero_grad()
        loss.backward()
        self._vairl_disc_opt.step()

        self.beta = max(0.0, self.beta + self.beta_step_size * float(bottleneck.item()))

        return {
            "loss": float(loss.item()),
            "kl": float(kl.item()),
            "beta": float(self.beta),
        }
