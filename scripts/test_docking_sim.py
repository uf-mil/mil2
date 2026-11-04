#!/usr/bin/env python3
"""Run perception docking regressions in isolated ROS/Gazebo environments."""

import os
import signal
import subprocess
import tempfile
import uuid
from pathlib import Path


def main():
    repo = Path(__file__).resolve().parents[1]
    logs = Path(tempfile.mkdtemp(prefix="mil2_docking_suite_"))
    cases = [
        ("green_seed_1", 1, "none"),
        ("green_seed_3", 3, "none"),
        ("hidden_green", 4, "green"),
    ]
    success = True
    for index, (name, seed, hidden) in enumerate(cases):
        env = os.environ.copy()
        env["DOCKING_ROS_DOMAIN_ID"] = str(
            int(os.environ.get("DOCKING_TEST_DOMAIN_BASE", "100")) + index,
        )
        env["DOCKING_GZ_PARTITION"] = "mil2_test_" + uuid.uuid4().hex
        env["ROS_LOG_DIR"] = str(logs / "ros")
        path = logs / (name + ".log")
        command = [
            "bash",
            str(repo / "scripts/run_docking_sim.bash"),
            f"seed:={seed}",
            f"hide_color:={hidden}",
            "exit_on_result:=true",
            "ocs_drop_receipts:=2",
            "gz_args:=-s --headless-rendering",
        ]
        if hidden != "none":
            command.append("timeout:=30.0")
        print(f"Running {name}; log: {path}", flush=True)
        with path.open("w") as output:
            process = subprocess.Popen(
                command,
                cwd=repo,
                env=env,
                stdout=output,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                process.wait(timeout=580)
            except (subprocess.TimeoutExpired, KeyboardInterrupt):
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                raise
        text = path.read_text()
        if hidden == "none":
            passed = (
                "[docking_evaluator]: PASS:" in text
                and "ENTER: aligned and camera/LiDAR target revalidated" in text
                and "WAIT_START: waiting for OCS" in text
                and "STARTED: accepted OCS command" in text
                and "MOCK_DROP:" in text
                and "RECEIVED: OCS acknowledged" in text
            )
        else:
            passed = (
                "FAIL: mission timeout" in text
                and "ALIGN: detected" not in text
                and "MOCK_RECEIVED:" not in text
                and "[docking_evaluator]: PASS:" not in text
            )
        print(f'{name}: {"PASS" if passed else "FAIL"}', flush=True)
        success = success and passed
    print(f"Logs: {logs}")
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
