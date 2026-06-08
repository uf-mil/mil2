"""Unit tests for the magnetometer hard/soft-iron ellipsoid fit.

These are pure-math tests: they exercise the calibration math in
``magnetic_compensation.ellipsoid`` with synthetic data and need only
numpy/scipy (no ROS, no hardware).

The model under test: a clean magnetometer traces a unit sphere; the sub's
hard-iron (a constant offset) and soft-iron (a linear distortion) warp it into
an off-center, tilted ellipsoid. ``fit_ellipsoid`` must recover the offset
(``shift``) and the distortion (``scale``) from points on that ellipsoid.
"""

import numpy as np
import pytest
from magnetic_compensation.ellipsoid import (
    calculate_error,
    fit_ellipsoid,
    normalized_matrix,
)

# --- helpers ---------------------------------------------------------------


def _fib_sphere(n):
    """n evenly distributed unit vectors (deterministic, full coverage)."""
    i = np.arange(n)
    phi = np.pi * (3.0 - np.sqrt(5.0))  # golden angle
    y = 1.0 - 2.0 * i / (n - 1)
    r = np.sqrt(np.clip(1.0 - y * y, 0.0, 1.0))
    theta = phi * i
    return np.column_stack((np.cos(theta) * r, y, np.sin(theta) * r))


def _rotation(rx, ry, rz):
    """A proper rotation matrix from fixed roll/pitch/yaw angles."""
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    rot_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    rot_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rot_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rot_z @ rot_y @ rot_x


def _ellipsoid_points(scale, shift, n=400):
    """Points on the ellipsoid: p_i = scale @ u_i + shift."""
    u = _fib_sphere(n)
    return u @ np.asarray(scale).T + np.asarray(shift)


# --- normalized_matrix -----------------------------------------------------


def test_normalized_matrix_has_unit_determinant():
    m = np.diag([1.0, 2.0, 3.0])
    assert np.isclose(np.linalg.det(normalized_matrix(m)), 1.0)


def test_normalized_matrix_is_scale_invariant():
    m = np.diag([1.0, 2.0, 3.0])
    assert np.allclose(normalized_matrix(5.0 * m), normalized_matrix(m))


def test_normalized_matrix_identity_unchanged():
    assert np.allclose(normalized_matrix(np.eye(3)), np.eye(3))


def test_normalized_matrix_rejects_nonpositive_determinant():
    with pytest.raises(AssertionError):
        normalized_matrix(np.diag([-1.0, 1.0, 1.0]))  # det = -1


# --- calculate_error -------------------------------------------------------


def test_calculate_error_zero_on_perfect_sphere():
    assert calculate_error(_fib_sphere(200)) == pytest.approx(0.0, abs=1e-9)


def test_calculate_error_known_value():
    pts = np.array([[1.0, 0, 0], [2.0, 0, 0], [3.0, 0, 0]])
    # radii = [1, 2, 3]; std/mean = 0.81650 / 2
    assert calculate_error(pts) == pytest.approx(0.40825, abs=1e-4)


# --- fit_ellipsoid: recovery (the "ellipsoids are proper" requirement) -----


def test_fit_identity_sphere():
    scale, shift = np.eye(3), np.zeros(3)
    s2, sh2, _ = fit_ellipsoid(_ellipsoid_points(scale, shift))
    assert np.allclose(s2, scale, atol=1e-6)
    assert np.allclose(sh2, shift, atol=1e-6)


def test_fit_recovers_pure_hard_iron():
    scale, shift = np.eye(3), np.array([1.0, -2.0, 0.5])
    s2, sh2, _ = fit_ellipsoid(_ellipsoid_points(scale, shift))
    assert np.allclose(s2, scale, atol=1e-6)
    assert np.allclose(sh2, shift, atol=1e-6)


def test_fit_recovers_pure_soft_iron():
    scale, shift = np.diag([2.0, 1.0, 0.5]), np.zeros(3)  # det(scale) = 1
    s2, sh2, _ = fit_ellipsoid(_ellipsoid_points(scale, shift))
    assert np.allclose(s2, scale, atol=1e-6)
    assert np.allclose(sh2, shift, atol=1e-6)


def test_fit_recovers_full_affine():
    rot = _rotation(0.3, -0.2, 0.5)
    scale = rot @ np.diag([2.0, 1.0, 0.5]) @ rot.T
    shift = np.array([0.3, -0.4, 0.2])
    s2, sh2, _ = fit_ellipsoid(_ellipsoid_points(scale, shift))
    assert np.allclose(s2, scale, atol=1e-6)
    assert np.allclose(sh2, shift, atol=1e-6)


def test_fit_output_scale_is_symmetric_spd_unit_det():
    rot = _rotation(0.3, -0.2, 0.5)
    scale = rot @ np.diag([2.0, 1.0, 0.5]) @ rot.T
    shift = np.array([0.3, -0.4, 0.2])
    s2, _, _ = fit_ellipsoid(_ellipsoid_points(scale, shift))
    assert np.allclose(s2, s2.T, atol=1e-9)  # symmetric
    assert np.isclose(np.linalg.det(s2), 1.0, atol=1e-6)  # normalized
    assert (np.linalg.eigvalsh(s2) > 0).all()  # positive-definite


def test_fit_compensated_points_lie_on_unit_sphere():
    rot = _rotation(0.3, -0.2, 0.5)
    scale = rot @ np.diag([2.0, 1.0, 0.5]) @ rot.T
    shift = np.array([0.3, -0.4, 0.2])
    _, _, compensated = fit_ellipsoid(_ellipsoid_points(scale, shift))
    assert calculate_error(compensated) < 1e-6


# --- fit_ellipsoid: edge cases (must fail loudly, not return garbage) ------


def test_fit_too_few_points_raises():
    pts = _fib_sphere(5)  # fewer than the 9 unknowns
    with pytest.raises(ValueError):
        fit_ellipsoid(pts)


def test_fit_coplanar_points_raises():
    u = _fib_sphere(200)
    u[:, 2] = 0.0  # collapse onto the z = 0 plane
    with pytest.raises(ValueError):
        fit_ellipsoid(u)


def test_fit_clustered_points_raises():
    # all directions bunched near +z -> compensated cloud never straddles 0
    a = np.linspace(0.0, 0.2, 60)
    b = np.linspace(0.0, 2 * np.pi, 60)
    pts = np.column_stack(
        (np.sin(a) * np.cos(b), np.sin(a) * np.sin(b), np.cos(a)),
    )
    with pytest.raises(AssertionError):
        fit_ellipsoid(pts)
