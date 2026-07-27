"""The one definition of run_task's exit codes.

Exit codes are load-bearing and split "the robot failed" from "the rig failed":
  0  mission SUCCESS
  1  mission ran and FAILED, or consumed its sim budget
  2  harness error (unknown task, sim never came up, gate timed out, clock stalled)

Anything that maps an outcome to a process exit status imports from here, so the
contract has exactly one home: the CLI, the report renderer, and the run_task
wrapper script all agree by construction.
"""

from __future__ import annotations

EXIT_OK = 0
EXIT_MISSION_FAILED = 1
EXIT_HARNESS = 2
