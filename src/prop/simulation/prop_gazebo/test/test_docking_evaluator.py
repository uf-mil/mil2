"""Independent scorer regressions, without launching ROS or Gazebo."""

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

from prop_missions.ocs_protocol import Exchange
from std_msgs.msg import String

spec = importlib.util.spec_from_file_location(
    "docking_evaluator",
    Path(__file__).resolve().parents[1] / "scripts" / "docking_evaluator.py",
)
evaluator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evaluator)


def test_default_vehicle_exchange_is_verified():
    exchange = Exchange()
    node = SimpleNamespace(
        vehicle_id=exchange.vehicle_id,
        wait_session=None,
        ocs_state=None,
        colors=["red", "green", "red"],
    )
    status = dict(exchange.envelope("status"), state="WAIT_START")
    evaluator.Evaluator.exchange_status(node, String(data=json.dumps(status)))
    assert node.wait_session == exchange.session_id
    command = dict(
        exchange.envelope("command"),
        action="START",
        run_id="42",
        command_id="start-1",
    )
    exchange.command(command)
    exchange.complete(2, 10.0)
    node.start_command = command
    node.docking_result = exchange.result
    node.ocs_receipt = dict(exchange.result, kind="receipt", status="RECEIVED")
    status = dict(
        exchange.envelope("status"),
        state="DOCKED",
        receipt="RECEIVED",
        event_id=exchange.result["event_id"],
    )
    evaluator.Evaluator.exchange_status(node, String(data=json.dumps(status)))
    assert evaluator.Evaluator.exchange_complete(node)
    node.ocs_receipt = dict(node.ocs_receipt, run_id="another-run")
    assert not evaluator.Evaluator.exchange_complete(node)


def test_other_vehicle_cannot_satisfy_start_gate():
    node = SimpleNamespace(vehicle_id="USV", wait_session=None, ocs_state=None)
    status = dict(Exchange("prop").envelope("status"), state="WAIT_START")
    evaluator.Evaluator.exchange_status(node, String(data=json.dumps(status)))
    assert node.wait_session is None
    assert node.ocs_state is None
