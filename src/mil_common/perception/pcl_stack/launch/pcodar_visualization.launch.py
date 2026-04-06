"""Backward-compatible alias for ``velodyne_visualization.launch.py``."""

import importlib.util
from pathlib import Path


def generate_launch_description():
    here = Path(__file__).resolve().parent
    path = here / "velodyne_visualization.launch.py"
    spec = importlib.util.spec_from_file_location(
        "pcl_stack_velodyne_visualization",
        str(path),
    )
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod.generate_launch_description()
