from types import SimpleNamespace
from unittest.mock import Mock

from prop_missions.docking_mission import DockingMission
from prop_missions.incident import Incident
from prop_missions.safe_passage import SafePassage
from std_msgs.msg import String


def clock_node(seconds):
    node = Mock()
    node.get_clock.return_value.now.side_effect = lambda: SimpleNamespace(
        nanoseconds=int(seconds.pop(0) * 1e9),
    )
    return node


def test_incident_reports_time_lost_once_back():
    ended = []
    incident = Incident(clock_node([10.0, 70.0]), ended.append)
    incident.on_state(String(data="IDLE"))
    assert not incident.active
    incident.on_state(String(data="NAVIGATING"))
    incident.on_state(String(data="HOLDING"))
    incident.on_state(String(data="RETURNING"))
    assert incident.active and ended == []
    incident.on_state(String(data="IDLE"))
    assert not incident.active and ended == [60.0]


def test_docking_does_nothing_while_task_4_has_the_boat():
    node = Mock()
    node.state = "ALIGN"
    node.incident.active = True
    DockingMission.step(node)
    node.seconds.assert_called_once()
    node.pub.publish.assert_not_called()
    node.plan.assert_not_called()


def test_docking_takes_the_boat_back():
    node = Mock()
    node.start = 100.0
    node.state = "ALIGN"
    node.odom = [object()]
    DockingMission.after_incident(node, 60.0)
    assert node.start == 160.0
    assert node.planner.call_async.call_args.args[0].data is False
    node.plan.assert_called_once_with(node.bay.stage)


def test_docking_waiting_for_start_is_left_alone():
    node = Mock()
    node.start = None
    node.state = "WAIT_START"
    DockingMission.after_incident(node, 60.0)
    node.planner.call_async.assert_not_called()
    node.plan.assert_not_called()


def test_safe_passage_pushes_its_clocks_on_and_resends_its_goal():
    node = SimpleNamespace(started=100.0, since=130.0, goal_sent=140.0)
    SafePassage.after_incident(node, 60.0)
    assert (node.started, node.since, node.goal_sent) == (160.0, 190.0, None)
