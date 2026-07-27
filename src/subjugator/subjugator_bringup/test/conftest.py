"""Make the package importable straight from source, without a colcon build."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
