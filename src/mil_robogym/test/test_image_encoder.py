"""Tests for the standalone CNN image encoder."""

import pytest

from mil_robogym.vairl.image_encoder import CNNEncoder

torch = pytest.importorskip("torch")


def test_cnn_encoder_outputs_fixed_latent_dim_for_single_rgb_image():
    """Encodes a single RGB image into one fixed-size latent vector."""
    encoder = CNNEncoder()

    image = torch.randint(0, 256, (3, 600, 960), dtype=torch.uint8)
    latent = encoder(image)

    assert latent.shape == (1, 128)
    assert torch.isfinite(latent).all()


def test_cnn_encoder_outputs_fixed_latent_dim_for_batched_rgb_images():
    """Encodes a batch of RGB images into batch-aligned latent vectors."""
    encoder = CNNEncoder(latent_dim=64)

    image = torch.rand(4, 3, 128, 192, dtype=torch.float32)
    latent = encoder(image)

    assert latent.shape == (4, 64)
    assert torch.isfinite(latent).all()


def test_cnn_encoder_supports_single_channel_images():
    """Promotes grayscale images to 3 channels before encoding."""
    encoder = CNNEncoder()

    image = torch.rand(2, 1, 73, 111, dtype=torch.float32)
    latent = encoder(image)

    assert latent.shape == (2, 128)
    assert torch.isfinite(latent).all()


def test_cnn_encoder_rejects_non_chw_layout():
    """Rejects tensors that are not provided in CHW or BCHW layout."""
    encoder = CNNEncoder()

    image = torch.rand(224, 224, 3, dtype=torch.float32)

    with pytest.raises(ValueError, match="CHW/BCHW layout"):
        encoder(image)


def test_cnn_encoder_rejects_unsupported_channel_count():
    """Rejects tensors whose channel dimension is neither 1 nor 3."""
    encoder = CNNEncoder()

    image = torch.rand(2, 4, 224, 224, dtype=torch.float32)

    with pytest.raises(ValueError, match="channel dimension must be 1 or 3"):
        encoder(image)
