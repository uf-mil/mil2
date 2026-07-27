from pathlib import Path

import pytest
from subjugator_bringup.task_runner import btbudget, tasks

# Resolved from this file, not the process CWD: ctest runs these from the
# package source dir while `pytest src/...` runs them from the repo root.
XML_DIR = str(
    Path(__file__).resolve().parents[2] / "mission_planner/subjugator_missions/xml",
)

# Mission XML that BT.CPP's tinyxml2 accepts but ElementTree rejects, so its
# trees are invisible here. Owned by mission_planner, deliberately not fixed
# from this package -- pinned so a NEW malformed file fails loudly instead of
# quietly shrinking some stage's budget.
KNOWN_UNPARSABLE = {"lock_target_xy_test_mission.xml", "roll_style.xml"}

# Durations in a stage tree that genuinely cannot resolve statically. Pinned so
# that a new unresolvable port shows up as a failure rather than as a silently
# smaller bound.
KNOWN_UNRESOLVED = {
    "OctagonMission": {
        # <Repeat num_cycles="-1"> in the S1 pinger sweep: unbounded by design.
        "Repeat.num_cycles=forever",
        # S7's turn count is set by a <Script> at runtime.
        "Repeat.count={turn_ticks}",
    },
}


@pytest.fixture(scope="module")
def trees():
    loaded = btbudget.load_trees(XML_DIR)
    assert loaded, f"no mission XML at {XML_DIR}"
    return loaded


def _trees(tmp_path, xml):
    """Build a tree table from one synthetic XML string."""
    (tmp_path / "t.xml").write_text(xml)
    return btbudget.load_trees(str(tmp_path))


# --- the real mission corpus ------------------------------------------------


def test_single_timeout_tree(trees):
    assert btbudget.declared_bound(trees, "CenterCameraTest").seconds == 30.0


def test_sequence_of_two_bounded_steps(trees):
    assert btbudget.declared_bound(trees, "HoneOverTableSelect").seconds == 45.0


def test_retry_multiplies_its_child(trees):
    # 2 attempts x (20 center + 30 descend + 1.5 settle + 30 lift)
    assert btbudget.declared_bound(trees, "OctagonGraspMission").seconds == 163.0


def test_unresolved_ports_are_reported(trees):
    bound = btbudget.declared_bound(trees, "OctagonMission")
    # turn_ticks is set by a <Script> at runtime, so it cannot resolve statically.
    assert any("turn_ticks" in u for u in bound.unresolved)
    assert not bound.complete


def test_pinger_sweep_is_flagged_as_unbounded(trees):
    # HomeOnOctagonPinger sweeps on <Repeat num_cycles="-1"> with NO enclosing
    # timeout, so its 89 s is one pass, not a ceiling. Counting -1 as a factor
    # would make the bound negative and silently disable the budget guard.
    bound = btbudget.declared_bound(trees, "HomeOnOctagonPinger")
    assert bound.seconds > 0
    assert "Repeat.num_cycles=forever" in bound.unresolved


def test_unparsable_mission_xml_is_the_known_set():
    assert set(btbudget.unparsable(XML_DIR)) == KNOWN_UNPARSABLE


def test_stage_unresolved_values_are_pinned(trees):
    for spec in tasks.TASKS.values():
        for name, stage in spec.stages.items():
            bound = btbudget.declared_bound(trees, stage.mission)
            expected = KNOWN_UNRESOLVED.get(stage.mission, set())
            assert (
                set(bound.unresolved) == expected
            ), f"stage {name} ({stage.mission}) changed which durations resolve"


def test_every_budget_covers_its_tree(trees):
    for num, spec in tasks.TASKS.items():
        for name, stage in spec.stages.items():
            bound = btbudget.declared_bound(trees, stage.mission)
            assert stage.sim_budget >= bound.seconds, (
                f"task {num} stage {name}: budget {stage.sim_budget} s is below "
                f"the tree's declared {bound.seconds} s"
            )
            missing = [u for u in bound.unresolved if u.startswith("SubTree.ID=")]
            assert not missing, f"stage {name} references missing subtrees: {missing}"


# --- walker semantics, on synthetic trees -----------------------------------


def test_wrapping_timeout_hides_its_childs_own_timeouts(tmp_path):
    trees = _trees(
        tmp_path,
        """<root BTCPP_format="4"><BehaviorTree ID="T">
        <RosTimeout msec="30000"><Action ID="X" move_timeout_msec="99000"/></RosTimeout>
      </BehaviorTree></root>""",
    )
    assert btbudget.declared_bound(trees, "T").seconds == 30.0  # not 129.0


def test_missing_subtree_id_is_reported(tmp_path):
    trees = _trees(
        tmp_path,
        """<root BTCPP_format="4"><BehaviorTree ID="T"><Sequence>
        <Action ID="X" timeout_msec="5000"/><SubTree ID="NotDefinedAnywhere"/>
      </Sequence></BehaviorTree></root>""",
    )
    bound = btbudget.declared_bound(trees, "T")
    assert bound.seconds == 5.0
    assert "SubTree.ID=NotDefinedAnywhere" in bound.unresolved


def test_same_subtree_twice_in_one_branch_counts_twice(tmp_path):
    trees = _trees(
        tmp_path,
        """<root BTCPP_format="4">
      <BehaviorTree ID="T"><Sequence>
        <SubTree ID="Leg"/><SubTree ID="Leg"/></Sequence></BehaviorTree>
      <BehaviorTree ID="Leg"><Action ID="X" timeout_msec="5000"/></BehaviorTree></root>""",
    )
    assert btbudget.declared_bound(trees, "T").seconds == 10.0


def test_recursive_subtree_terminates(tmp_path):
    trees = _trees(
        tmp_path,
        """<root BTCPP_format="4">
      <BehaviorTree ID="A"><Sequence>
        <Action ID="X" timeout_msec="3000"/><SubTree ID="B"/></Sequence></BehaviorTree>
      <BehaviorTree ID="B"><SubTree ID="A"/></BehaviorTree></root>""",
    )
    bound = btbudget.declared_bound(trees, "A")
    assert bound.seconds == 3.0
    # A cycle is not a missing subtree; it must not be reported as one.
    assert not any(u.startswith("SubTree.ID=") for u in bound.unresolved)


_AUTOREMAP_XML = """<root BTCPP_format="4">
  <BehaviorTree ID="Outer"><SubTree ID="Mid" cap_msec="7000"/></BehaviorTree>
  <BehaviorTree ID="Mid"><SubTree ID="Leg" {autoremap}/></BehaviorTree>
  <BehaviorTree ID="Leg">
    <RosTimeout msec="{{cap_msec}}"><AlwaysSuccess/></RosTimeout>
  </BehaviorTree></root>"""


def test_autoremap_forwards_the_callers_env(tmp_path):
    trees = _trees(tmp_path, _AUTOREMAP_XML.format(autoremap='_autoremap="true"'))
    assert btbudget.declared_bound(trees, "Outer").seconds == 7.0


def test_without_autoremap_the_child_env_is_empty(tmp_path):
    trees = _trees(tmp_path, _AUTOREMAP_XML.format(autoremap=""))
    bound = btbudget.declared_bound(trees, "Outer")
    assert bound.seconds == 0.0
    assert "RosTimeout.msec={cap_msec}" in bound.unresolved


@pytest.mark.parametrize(
    ("node", "marker"),
    [
        ('<Repeat num_cycles="-1">{child}</Repeat>', "Repeat.num_cycles=forever"),
        (
            "<KeepRunningUntilFailure>{child}</KeepRunningUntilFailure>",
            "KeepRunningUntilFailure=forever",
        ),
    ],
)
def test_infinite_loops_count_one_pass_and_are_flagged(tmp_path, node, marker):
    body = node.format(child='<Action ID="X" timeout_msec="5000"/>')
    trees = _trees(
        tmp_path,
        f'<root BTCPP_format="4"><BehaviorTree ID="T">{body}</BehaviorTree></root>',
    )
    bound = btbudget.declared_bound(trees, "T")
    assert bound.seconds == 5.0  # one pass, never negative
    assert marker in bound.unresolved
    assert not bound.complete


def test_zero_cycles_contributes_nothing(tmp_path):
    trees = _trees(
        tmp_path,
        """<root BTCPP_format="4"><BehaviorTree ID="T">
        <Repeat num_cycles="0"><Action ID="X" timeout_msec="5000"/></Repeat>
      </BehaviorTree></root>""",
    )
    assert btbudget.declared_bound(trees, "T").seconds == 0.0


def test_unresolved_is_deduped(tmp_path):
    trees = _trees(
        tmp_path,
        """<root BTCPP_format="4">
      <BehaviorTree ID="T"><Sequence>
        <SubTree ID="Leg"/><SubTree ID="Leg"/></Sequence></BehaviorTree>
      <BehaviorTree ID="Leg">
        <RosTimeout msec="{cap_msec}"><AlwaysSuccess/></RosTimeout>
      </BehaviorTree></root>""",
    )
    assert btbudget.declared_bound(trees, "T").unresolved == [
        "RosTimeout.msec={cap_msec}",
    ]


def test_a_subtree_walked_alone_is_incomplete_not_zero_length(trees):
    # The footgun Bound.complete exists for: every duration in ApproachAndGrasp
    # comes from a caller-supplied port, so on its own it bounds to 0.0 s.
    bound = btbudget.declared_bound(trees, "ApproachAndGrasp")
    assert bound.seconds == 0.0
    assert not bound.complete
