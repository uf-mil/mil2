from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F


class CNNEncoder(nn.Module):
    """
    Encode image tensors of arbitrary spatial size into a fixed-size latent vector.

    Expected input layouts:
      - ``(C, H, W)``
      - ``(B, C, H, W)``

    Expected value ranges:
      - ``uint8`` inputs in ``[0, 255]`` are scaled into ``[0, 1]``
      - floating point inputs are expected to already be in ``[0, 1]``

    Single-channel inputs are promoted to 3 channels by repetition so the
    encoder can be used with either grayscale or RGB tensors.
    """

    def __init__(
        self,
        latent_dim: int = 128,
        resize_shape: tuple[int, int] = (224, 224),
        mean: tuple[float, float, float] = (0.485, 0.456, 0.406),
        std: tuple[float, float, float] = (0.229, 0.224, 0.225),
    ):
        super().__init__()

        if latent_dim <= 0:
            raise ValueError("latent_dim must be positive.")
        if len(resize_shape) != 2 or any(size <= 0 for size in resize_shape):
            raise ValueError("resize_shape must contain exactly two positive ints.")
        if len(mean) != 3 or len(std) != 3:
            raise ValueError("mean and std must each contain exactly 3 values.")
        if any(value <= 0 for value in std):
            raise ValueError("std values must be positive.")

        self.latent_dim = latent_dim
        self.resize_shape = tuple(int(size) for size in resize_shape)

        self.register_buffer(
            "mean",
            torch.tensor(mean, dtype=torch.float32).view(1, 3, 1, 1),
        )
        self.register_buffer(
            "std",
            torch.tensor(std, dtype=torch.float32).view(1, 3, 1, 1),
        )

        self.backbone = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(128, 256, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),
        )
        self.pool = nn.AdaptiveAvgPool2d((1, 1))
        self.projection = nn.Linear(256, latent_dim)

    def _prepare_input(self, image: torch.Tensor) -> torch.Tensor:
        if not isinstance(image, torch.Tensor):
            raise TypeError("image must be a torch.Tensor.")

        if image.ndim == 3:
            if image.shape[0] not in (1, 3) and image.shape[-1] in (1, 3):
                raise ValueError(
                    "image must be in CHW/BCHW layout, not HWC/BHWC layout.",
                )
            image = image.unsqueeze(0)
        elif image.ndim == 4:
            if image.shape[1] not in (1, 3) and image.shape[-1] in (1, 3):
                raise ValueError(
                    "image must be in CHW/BCHW layout, not HWC/BHWC layout.",
                )
        else:
            raise ValueError(
                "image must have shape (C, H, W) or (B, C, H, W).",
            )

        if image.shape[1] == 1:
            image = image.repeat(1, 3, 1, 1)
        elif image.shape[1] != 3:
            raise ValueError(
                "image channel dimension must be 1 or 3 in CHW/BCHW layout.",
            )

        if image.dtype == torch.uint8:
            image = image.to(dtype=torch.float32) / 255.0
        else:
            image = image.to(dtype=torch.float32)

        image = F.interpolate(
            image,
            size=self.resize_shape,
            mode="bilinear",
            align_corners=False,
        )
        image = (image - self.mean) / self.std
        return image

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        image = self._prepare_input(image)
        features = self.backbone(image)
        pooled = self.pool(features).flatten(start_dim=1)
        return self.projection(pooled)
