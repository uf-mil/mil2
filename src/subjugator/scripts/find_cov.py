import numpy as np
import pandas as pd

# === CONFIGURATION ===
csv_file = "some_data.txt"  # Replace with your file
columns = ["accel_x", "accel_y", "accel_z"]

# === LOAD DATA ===
df = pd.read_csv(csv_file)

# Ensure the data has the required columns
if not all(col in df.columns for col in columns):
    raise ValueError(f"CSV must contain columns: {columns}")

# Extract acceleration values
accel_data = df[columns].to_numpy()

# === COMPUTE COVARIANCE ===
cov_matrix = np.cov(accel_data, rowvar=False)
variances = np.diag(cov_matrix)

# === PRINT RESULTS ===
print("Covariance Matrix (linear acceleration):")
print(cov_matrix)

print("\nRecommended diagonal values for robot_localization:")
print(f"X: {variances[0]:.6f}")
print(f"Y: {variances[1]:.6f}")
print(f"Z: {variances[2]:.6f}")

print("\nrobot_localization format:")
print(f"[{variances[0]:.6f}, 0, 0, 0, {variances[1]:.6f}, 0, 0, 0, {variances[2]:.6f}]")
