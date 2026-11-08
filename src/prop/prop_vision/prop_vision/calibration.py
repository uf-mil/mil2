"""The camera's calibration for the frames it actually streams."""

import json

import numpy as np


def load(path, width, height, rotate=0):
    """
    Camera matrix and distortion coefficients for width x height frames that
    camera_node turned rotate degrees (0 or 180).

    Turning frames upside down mirrors the principal point through the image
    centre and flips the sign of the tangential terms.
    """
    with open(path) as f:
        params = json.load(f)
    matrix = np.array(params["camera_matrix"], dtype=float)
    dist = np.array(params["dist_coeffs"], dtype=float)
    calibrated_width = params["calibrated_width"]
    calibrated_height = params["calibrated_height"]
    if rotate == 180:
        matrix[0, 2] = calibrated_width - 1 - matrix[0, 2]
        matrix[1, 2] = calibrated_height - 1 - matrix[1, 2]
        dist.flat[2:4] *= -1
    elif rotate != 0:
        raise ValueError(f"rotate must be 0 or 180, not {rotate}")
    matrix[0] *= width / calibrated_width
    matrix[1] *= height / calibrated_height
    return matrix, dist
