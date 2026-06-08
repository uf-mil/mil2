"""Pure-math hard/soft-iron ellipsoid fitting for magnetometer calibration.

A clean magnetometer, swept through all orientations, traces a sphere centered
at the origin. The sub's hard-iron (a constant offset) and soft-iron (a linear
distortion) warp that sphere into an off-center, tilted ellipsoid.

``fit_ellipsoid`` recovers the offset (``shift``) and the distortion
(``scale``) from points on that ellipsoid so the live node can undo them with
``scale^-1 @ (raw - shift)``.

This module has no ROS or plotting dependencies so the math can be unit tested
in isolation (numpy + scipy only).
"""

import numpy as np
import scipy.linalg


def normalized_matrix(m):
    assert np.linalg.det(m) > 0
    return m / np.linalg.det(m) ** (1 / m.shape[0])


def calculate_error(points):
    radii = list(map(np.linalg.norm, points))
    error = np.std(radii) / np.mean(radii)
    return error


def fit_ellipsoid(points):
    points = np.array(points)

    # The fit solves for 9 coefficients; fewer points leaves it underdetermined
    # and lstsq returns a confident-looking but meaningless calibration. Reject
    # that loudly rather than silently emitting a bogus scale/shift.
    if points.shape[0] < 9:
        raise ValueError(
            f"fit_ellipsoid needs at least 9 points, got {points.shape[0]}",
        )

    # Reject (near-)coplanar/collinear data. If the points don't span all three
    # axes the ellipsoid is underdetermined along an axis, and whether the fit
    # then errors out is numerically fragile (it depends on the BLAS backend).
    # The singular values of the mean-centered points measure spread per axis; a
    # vanishing smallest one means the calibration sweep was degenerate (e.g. the
    # vehicle only rotated about one axis), so fail loudly here instead.
    singular_values = np.linalg.svd(points - points.mean(axis=0), compute_uv=False)
    if singular_values[-1] < 1e-6 * singular_values[0]:
        raise ValueError(
            "fit_ellipsoid: points are (near-)coplanar; calibration data must "
            "cover all three axes",
        )

    A = np.zeros((points.shape[0], 9))
    A[:, 0] = points[:, 0] ** 2
    A[:, 1] = points[:, 1] ** 2
    A[:, 2] = points[:, 2] ** 2
    A[:, 3] = 2 * points[:, 0] * points[:, 1]
    A[:, 4] = 2 * points[:, 0] * points[:, 2]
    A[:, 5] = 2 * points[:, 1] * points[:, 2]
    A[:, 6] = -2 * points[:, 0]
    A[:, 7] = -2 * points[:, 1]
    A[:, 8] = -2 * points[:, 2]

    B = np.ones((points.shape[0], 1))

    X = np.linalg.lstsq(A, B, rcond=None)[0].flatten()
    if X[0] < 0:
        X = -X

    ka = np.linalg.inv(
        np.array(
            [
                [X[0], X[3], X[4]],
                [X[3], X[1], X[5]],
                [X[4], X[5], X[2]],
            ],
        ),
    )
    shift = ka.dot(X[6:9])

    scale = scipy.linalg.sqrtm(ka)
    assert np.isreal(scale).all(), scale
    scale = np.real(scale)
    scale = normalized_matrix(scale)

    scale_inv = np.linalg.inv(scale)
    compensated = [scale_inv.dot(p - shift) for p in points]

    for axis in range(3):
        assert min(p[axis] for p in compensated) < 0 < max(p[axis] for p in compensated)

    return scale, shift, compensated
