"""
Work In Progress
Drag Compensator

This script implements an experimental drag compensator for the subjugator. It
does this by estimating the drag forces, multiplying them by a gain, and publishing
the result for controller.cpp.
"""

from pathlib import Path

import pandas as pd

LOG_RATE_HZ = 10.0

THRUSTER_FIELDS = [
    "thrust_flh",
    "thrust_frh",
    "thrust_blh",
    "thrust_brh",
    "thrust_flv",
    "thrust_frv",
    "thrust_blv",
    "thrust_brv",
]

# Collect logs from record_fwd.py output
logs_dir = Path.home() / "record-fwd-movement" / "logs"
logs_paths = list(logs_dir.glob("*.csv"))
logs_dfs = [pd.read_csv(log) for log in logs_paths]

cleaned_logs = [
    log.dropna() for log in logs_dfs if not log[["x", "y", "z"]].isnull().all().all()
]

if not cleaned_logs:
    raise RuntimeError(
        f"No logs with valid odometry found in {logs_dir}. "
        "Was the EKF enabled during the runs?",
    )

combined = pd.concat(cleaned_logs, ignore_index=True)

grouped = combined.groupby("t_sec", as_index=False)
averaged = grouped.mean().sort_values("t_sec").reset_index(drop=True)

run_counts = grouped.size().sort_values("t_sec").reset_index(drop=True)
averaged["n_runs"] = run_counts["size"]

print(f"Averaged {len(cleaned_logs)} runs onto {len(averaged)} timestamps.")
print(f"Runs per timestamp: {averaged['n_runs'].min()}-{averaged['n_runs'].max()}")
