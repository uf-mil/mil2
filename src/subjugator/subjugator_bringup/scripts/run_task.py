#!/usr/bin/env python3
"""ros2 run entrypoint: `ros2 run subjugator_bringup run_task 5`."""

import sys

from subjugator_bringup.task_runner.cli import main

if __name__ == "__main__":
    sys.exit(main())
