"""Tests for DeepSet network behavior."""

import pytest

from mil_robogym.vairl.deep_set import DeepSet

torch = pytest.importorskip("torch")


def test_deep_set_forward_shapes_and_gradients():
    """Produces expected output shapes and supports backward pass."""
    model = DeepSet(obj={"a": [1.0, 2.0], "b": (3.0, 4.0)}, output_shape=5)

    n = model.input_dim
    m = 7
    b = 3

    assert model.input_dim == 4

    x_single = torch.randn(m, n)
    y_single = model(x_single)
    assert y_single.shape == (5,)

    x_batch = torch.randn(b, m, n)
    y_batch = model(x_batch)
    assert y_batch.shape == (b, 5)

    loss = y_single.pow(2).mean() + y_batch.pow(2).mean()
    loss.backward()

    assert any(param.grad is not None for param in model.parameters())


def test_deep_set_raises_on_wrong_feature_dimension():
    """Rejects inputs whose feature dimension does not match model input_dim."""
    model = DeepSet(obj={"a": [1.0, 2.0], "b": (3.0, 4.0)}, output_shape=5)

    m = 7
    bad = torch.randn(m, model.input_dim + 1)

    with pytest.raises(ValueError, match="does not match expected input_dim"):
        model(bad)


def test_deep_set_rejects_non_int_output_shape():
    """Rejects non-integer output_shape."""
    with pytest.raises(TypeError, match="output_shape must be an int"):
        DeepSet(obj={"x": 1.0}, output_shape=(5,))


def test_deep_set_rejects_empty_object():
    """Rejects objects that flatten to an empty tensor."""
    with pytest.raises(ValueError, match="empty tensor"):
        DeepSet(obj=[], output_shape=3)


def test_deep_set_raises_on_invalid_input_rank():
    """Rejects inputs that are not [m, n] or [b, m, n]."""
    model = DeepSet(obj={"x": 1.0}, output_shape=2)
    bad = torch.randn(model.input_dim)

    with pytest.raises(ValueError, match=r"shape \[m, n\] or \[b, m, n\]"):
        model(bad)


def test_deep_set_handles_boolean_values():
    """Supports bool values when flattening objects."""
    model = DeepSet(obj={"a": True, "b": False, "c": 1.0}, output_shape=4)
    x = torch.randn(5, model.input_dim)
    y = model(x)
    assert y.shape == (4,)


def test_deep_set_rejects_string_in_object():
    """Rejects string values during object flattening."""
    model = DeepSet(obj={"x": 1.0}, output_shape=2)

    with pytest.raises(TypeError, match="String values are not supported"):
        model.convert_object_to_tensor({"bad": "text"})


def test_deep_set_set_order_is_stable():
    """Produces stable tensor conversion for set elements regardless of insertion order."""
    model = DeepSet(obj={"s": {3, 1, 2}}, output_shape=2)
    t1 = model.convert_object_to_tensor({"s": {1, 2, 3}})
    t2 = model.convert_object_to_tensor({"s": {3, 2, 1}})

    assert torch.equal(t1, t2)


def test_deep_set_deterministic_in_eval_mode():
    """Produces deterministic outputs for the same input in eval mode."""
    torch.manual_seed(0)
    model = DeepSet(obj={"a": [1.0, 2.0]}, output_shape=3).eval()
    x = torch.randn(6, model.input_dim)
    y1 = model(x)
    y2 = model(x)

    assert torch.allclose(y1, y2)
