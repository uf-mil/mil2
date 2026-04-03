import torch
import torch.nn as nn


class Encoder(nn.Module):
    """
    Encoder that maps states into latent space.
    """

    def __init__(self, input_size: int, z_size: int, e_hidden_size: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_size, e_hidden_size),
            nn.ReLU(),
            nn.Linear(e_hidden_size, e_hidden_size),
            nn.ReLU(),
        )
        self.mu = nn.Linear(e_hidden_size, z_size)
        self.logvar = nn.Linear(e_hidden_size, z_size)

    def forward(self, x: torch.Tensor):
        h = self.net(x)
        mu = self.mu(h)
        logvar = torch.clamp(self.logvar(h), min=-10.0, max=10.0)

        std = torch.exp(0.5 * logvar)
        eps = torch.randn_like(std)
        z = mu + eps * std
        return z, mu, logvar


class GNet(nn.Module):
    """
    Encoder that maps state and action wise inputs into latent space.
    """

    def __init__(self, z_size: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(z_size, 64),
            nn.ReLU(),
            nn.Linear(64, 1),
        )

    def forward(self, z):
        return self.net(z)


class HNet(nn.Module):
    """
    Encoder that mpas state wise inputs into latent space.
    """

    def __init__(self, z_size: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(z_size, 64),
            nn.ReLU(),
            nn.Linear(64, 1),
        )

    def forward(self, z):
        return self.net(z)
