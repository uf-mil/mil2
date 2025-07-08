import numpy as np

# helper function for comparing observation spaces, or any other dicts
def dicts_equal(a, b):
    if a.keys() != b.keys():
        return False
    for k in a:
        v1, v2 = a[k], b[k]
        if isinstance(v1, np.ndarray) and isinstance(v2, np.ndarray):
            if not np.array_equal(v1, v2):
                return False
        else:
            if v1 != v2:
                return False
    return True    

# helper function for turning nan values in dicts from /dvl/odom into 0 values
def nan_to_zero_in_dict(obs_dict):
    """
    Given a dict whose values may include numpy arrays,
    return a new dict where all NaNs in those arrays have been replaced by 0.
    """
    clean = {}
    for key, val in obs_dict.items():
        if isinstance(val, np.ndarray):
            # nan_to_num will turn NaN → 0, +inf → large finite, -inf → large negative finite
            clean[key] = np.nan_to_num(val, nan=0.0)
        else:
            clean[key] = val
    return clean