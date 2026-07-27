from subjugator_bringup.task_runner import btlog

# Real shape, including the ANSI colouring BT.CPP's StdCoutLogger emits.
R = "\x1b[33mRUNNING\x1b[0m"
S = "\x1b[32mSUCCESS\x1b[0m"
F = "\x1b[31mFAILURE\x1b[0m"
IDLE = "\x1b[36mIDLE\x1b[0m"

LOG = f"""[INFO] [1783913878.030259313] [mission_planner]: Waiting for odometry...
[INFO] [1783913879.431511252] [mission_planner]: Odometry received. Starting mission!
[1783913880.000]: AcquireTable              {IDLE} -> {R}
[1783913918.000]: AcquireTable              {R} -> {S}
[1783913918.100]: CollectOneObject          {IDLE} -> {R}
[1783913960.100]: CollectOneObject          {R} -> {S}
[1783913960.200]: CollectOneObject          {IDLE} -> {R}
[1783914000.200]: CollectOneObject          {R} -> {F}
[INFO] [1783914000.300000000] [mission_planner]: Mission finished with status: FAILURE. Shutting down.
"""

TREE_IDS = {"AcquireTable", "CollectOneObject", "OctagonMission"}


def test_outcome_is_extracted():
    assert btlog.parse(LOG, TREE_IDS).outcome == "FAILURE"


def test_missing_status_line_means_no_outcome():
    truncated = "\n".join(LOG.splitlines()[:4])
    assert btlog.parse(truncated, TREE_IDS).outcome is None


def test_stage_durations():
    stages = btlog.parse(LOG, TREE_IDS).stages
    assert stages[0].name == "AcquireTable"
    assert stages[0].result == "SUCCESS"
    assert stages[0].duration == 38.0


def test_repeated_subtree_gets_numbered():
    stages = btlog.parse(LOG, TREE_IDS).stages
    repeats = [s for s in stages if s.name == "CollectOneObject"]
    assert len(repeats) == 2
    assert repeats[0].occurrence == 1
    assert repeats[1].occurrence == 2
    assert repeats[1].result == "FAILURE"


def test_only_known_tree_ids_become_stages():
    stages = btlog.parse(LOG, {"AcquireTable"}).stages
    assert [s.name for s in stages] == ["AcquireTable"]


def test_last_node_is_identified():
    assert btlog.parse(LOG, TREE_IDS).last_node == "CollectOneObject"


def test_start_and_end_times():
    result = btlog.parse(LOG, TREE_IDS)
    assert result.start_wall == 1783913880.0
    assert result.end_wall == 1783914000.2
