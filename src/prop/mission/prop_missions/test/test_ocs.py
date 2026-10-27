"""Run isolation, retries, cancellation, and start-gate regressions."""

import json
from unittest.mock import Mock

import pytest
from prop_missions.ocs_protocol import Exchange, decode
from prop_missions.docking_mission import DockingMission
from prop_missions.mock_ocs import MockOCS
from std_msgs.msg import String


def start(exchange, **changes):
    return dict(exchange.envelope("command"), **dict(run_id="run-1", command_id="start-1", action="START", **changes))


def receipt(exchange, **changes):
    msg = dict(exchange.envelope("receipt"), event_id=exchange.result["event_id"], status="RECEIVED")
    msg.update(changes)
    return msg


def test_wait_start_never_samples_or_commands_navigation():
    node = Mock(state="WAIT_START")
    DockingMission.step(node)
    node.sample.assert_not_called()
    node.plan.assert_not_called()
    node.pub.publish.assert_not_called()


def test_start_once_and_reject_old_session_or_other_vehicle():
    e = Exchange()
    command = start(e)
    for key, value in [("session_id", "old-session"), ("vehicle_id", "other")]:
        assert e.command(dict(command, **{key: value})) == "REJECTED"
    assert e.command(command) == "STARTED"
    assert e.command(command) == "DUPLICATE"
    assert e.command(dict(command, run_id="another-run")) == "REJECTED"
    assert e.command(dict(command, command_id="another-command")) == "REJECTED"
    assert Exchange().command(command) == "REJECTED"


def test_retry_identical_event_and_validate_receipt():
    e = Exchange()
    e.command(start(e))
    e.complete(2, 10)
    event = e.outgoing(10)
    assert event["bay_number"] == 2
    assert e.outgoing(10.5) is None
    assert e.outgoing(11) == event
    for changes in [{"run_id": "old"}, {"session_id": "old"}, {"event_id": "wrong"}, {"vehicle_id": "other"}, {"status": "SENT"}]:
        assert not e.acknowledge(receipt(e, **changes), 11)
    assert e.acknowledge(receipt(e), 12)
    assert e.outgoing(13) is None
    assert e.receipt == "RECEIVED"
    e.complete(1, 14)
    assert e.result == event


def test_missing_or_late_receipt_is_not_success():
    e = Exchange(ack_timeout=3)
    e.command(start(e))
    e.complete(1, 10)
    assert not e.acknowledge(receipt(e), 13)
    assert e.outgoing(13) is None
    assert e.receipt == "TIMED_OUT"


def test_cancel_stops_retries_and_prevents_restart():
    e = Exchange()
    cmd = start(e)
    e.command(cmd)
    e.complete(3, 1)
    assert e.command(dict(cmd, action="CANCEL", command_id="cancel-1")) == "CANCELLED"
    assert e.outgoing(2) is None
    assert not e.acknowledge(receipt(e), 2)
    assert e.command(cmd) == "REJECTED"


def test_cancel_before_start_closes_only_addressed_session():
    e = Exchange()
    cmd = start(e)
    cancel = dict(cmd, action="CANCEL", command_id="cancel-1")
    assert e.command(dict(cancel, session_id="old")) == "REJECTED"
    assert e.command(cancel) == "CANCELLED"
    assert e.run_id == "run-1"
    assert e.command(cmd) == "REJECTED"
    assert e.command(dict(cancel, run_id="different")) == "REJECTED"


@pytest.mark.parametrize("raw", ["bad", "[]", "null", "{}", '{"version":true}', '{"version":2}'])
def test_malformed_messages(raw):
    assert decode(raw) is None
    assert Exchange().command(decode(raw)) == "REJECTED"


def test_no_result_before_start_or_with_invalid_bay():
    e = Exchange()
    e.complete(1, 1)
    assert e.result is None
    e.command(start(e))
    for bay in [0, 4, True, "1"]:
        with pytest.raises(ValueError):
            e.complete(bay, 1)


def test_mock_deduplicates_and_reacks_retries():
    e = Exchange()
    cmd = start(e)
    e.command(cmd)
    e.complete(2, 1)
    node = Mock(command=cmd, accepted=None, drop_receipts=1)
    message = String(data=json.dumps(e.result))
    MockOCS.result(node, message)
    node.receipt_pub.publish.assert_not_called()
    MockOCS.result(node, message)
    ack = decode(node.receipt_pub.publish.call_args.args[0].data)
    assert e.acknowledge(ack, 2)
    node.receipt_pub.publish.reset_mock()
    MockOCS.result(node, String(data=json.dumps(dict(e.result, bay_number=3))))
    node.receipt_pub.publish.assert_not_called()


def test_mock_waits_for_initialized_clock_and_full_delay():
    node = Mock(vehicle_id="prop", command=None, last_status=None,
                first_seen=None, auto_start=True, start_delay=3.0)
    e = Exchange()
    status = String(data=json.dumps(dict(e.envelope("status"), state="WAIT_START")))
    node.now.return_value = 0.0
    MockOCS.status(node, status)
    assert node.command is None
    node.now.return_value = 100.0
    MockOCS.status(node, status)
    assert node.first_seen == 100.0
    node.now.return_value = 102.9
    MockOCS.tick(node)
    node.command_pub.publish.assert_not_called()
    node.now.return_value = 103.0
    MockOCS.tick(node)
    assert decode(node.command_pub.publish.call_args.args[0].data)["action"] == "START"
