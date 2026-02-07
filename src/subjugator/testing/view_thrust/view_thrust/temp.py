def binary_search(target, effort_to_pwm_lut):
    low = 0
    high = len(effort_to_pwm_lut) - 1
    
    while low <= high:
        mid = low + (high - low) // 2
        if effort_to_pwm_lut[mid] < target:
            low = mid + 1
        elif effort_to_pwm_lut[mid] > target:
            high = mid - 1
        else:
            return mid
    
    # Return closest match
    if high < 0:
        return 0
    if low >= len(effort_to_pwm_lut):
        return len(effort_to_pwm_lut) - 1
    
    return high if (target - effort_to_pwm_lut[high] < effort_to_pwm_lut[low] - target) else low


def thruster_effort_to_duty(effort, effort_to_pwm_lut):
    # first, if the effort is zero, return early with a zero-duty
    if effort == 0:
        return 1500  # TODO use the STOP_VAL constant instead
    
    # Bound it just in case
    effort = max(-1, min(1, effort))
    
    min_val = effort_to_pwm_lut[0]
    max_val = effort_to_pwm_lut[-1]
    
    # convert the given percentage to a target value in the list
    # this value might not exist in the data sample, we will pick the closest match
    if effort < 0:
        target = -effort * min_val
    else:
        target = effort * max_val
    
    closest_index = binary_search(target, effort_to_pwm_lut)
    
    # convert index to duty and return
    duty = 1100 + closest_index * 4
    return duty

effort_to_pwm_lut = [
    -4.07,
    -4.05,
    -4.02,
    -3.96,
    -3.90,
    -3.87,
    -3.82,
    -3.80,
    -3.75,
    -3.71,
    -3.66,
    -3.59,
    -3.52,
    -3.45,
    -3.40,
    -3.31,
    -3.25,
    -3.20,
    -3.11,
    -3.04,
    -2.99,
    -2.94,
    -2.86,
    -2.82,
    -2.77,
    -2.71,
    -2.66,
    -2.58,
    -2.55,
    -2.51,
    -2.45,
    -2.38,
    -2.35,
    -2.28,
    -2.24,
    -2.20,
    -2.12,
    -2.08,
    -2.02,
    -1.98,
    -1.94,
    -1.86,
    -1.81,
    -1.76,
    -1.70,
    -1.67,
    -1.61,
    -1.56,
    -1.51,
    -1.49,
    -1.44,
    -1.40,
    -1.35,
    -1.30,
    -1.26,
    -1.20,
    -1.16,
    -1.12,
    -1.10,
    -1.05,
    -1.02,
    -0.98,
    -0.94,
    -0.90,
    -0.87,
    -0.82,
    -0.78,
    -0.74,
    -0.72,
    -0.68,
    -0.65,
    -0.62,
    -0.58,
    -0.54,
    -0.51,
    -0.48,
    -0.44,
    -0.42,
    -0.39,
    -0.35,
    -0.32,
    -0.29,
    -0.26,
    -0.24,
    -0.21,
    -0.18,
    -0.15,
    -0.13,
    -0.10,
    -0.09,
    -0.07,
    -0.05,
    -0.04,
    0.00,
    0.00,
    0.00,
    0.00,
    0.00,
    0.00,
    0.00,
    0.00,
    0.00,
    0.00,
    0.00,
    0.00,
    0.00,
    0.00,
    0.00,
    0.04,
    0.05,
    0.08,
    0.10,
    0.13,
    0.15,
    0.18,
    0.22,
    0.25,
    0.29,
    0.32,
    0.36,
    0.40,
    0.44,
    0.47,
    0.51,
    0.56,
    0.60,
    0.64,
    0.68,
    0.72,
    0.78,
    0.82,
    0.87,
    0.91,
    0.95,
    0.99,
    1.03,
    1.10,
    1.14,
    1.18,
    1.24,
    1.28,
    1.33,
    1.39,
    1.44,
    1.48,
    1.54,
    1.59,
    1.65,
    1.69,
    1.76,
    1.82,
    1.88,
    1.93,
    1.99,
    2.05,
    2.12,
    2.18,
    2.22,
    2.28,
    2.38,
    2.43,
    2.52,
    2.58,
    2.65,
    2.73,
    2.76,
    2.84,
    2.89,
    2.98,
    3.05,
    3.11,
    3.17,
    3.22,
    3.30,
    3.37,
    3.42,
    3.51,
    3.61,
    3.68,
    3.74,
    3.82,
    3.89,
    3.96,
    4.06,
    4.15,
    4.25,
    4.30,
    4.38,
    4.51,
    4.53,
    4.65,
    4.71,
    4.79,
    4.84,
    4.93,
    5.01,
    5.08,
    5.14,
    5.18,
    5.22,
    5.25 ]

while True:
    effort = float(input())
    duty = thruster_effort_to_duty(effort, effort_to_pwm_lut)
    index = (duty-1100)/4
    print("duty ", duty)
    print("index ", index)
    print("actual ", effort_to_pwm_lut[int(index)])
