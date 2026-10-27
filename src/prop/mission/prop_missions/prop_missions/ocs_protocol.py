"""Versioned internal docking protocol; no RoboCommand or ROS dependency."""

import json
import math
import uuid

VERSION = 1


def decode(raw):
    """Reject malformed envelopes; callers validate message-specific fields."""
    try:
        msg = json.loads(raw)
    except (ValueError, TypeError):
        return None
    if not isinstance(msg, dict) or type(msg.get("version")) is not int or msg["version"] != VERSION:
        return None
    for key in ("kind", "vehicle_id", "session_id"):
        if not isinstance(msg.get(key), str) or not msg[key] or len(msg[key]) > 128:
            return None
    return msg


def identifier(value):
    return isinstance(value, str) and 0 < len(value) <= 128


class Exchange:
    """One run per mission process. Restart creates a fresh replay boundary."""

    def __init__(self, vehicle_id="prop", ack_timeout=15.0):
        if not identifier(vehicle_id) or not math.isfinite(ack_timeout) or ack_timeout <= 0:
            raise ValueError("invalid vehicle ID or acknowledgement timeout")
        self.vehicle_id = vehicle_id
        self.session_id = uuid.uuid4().hex
        self.ack_timeout = ack_timeout
        self.run_id = None
        self.command_id = None
        self.result = None
        self.receipt = "NONE"
        self.closed = False
        self.next_send = 0.0
        self.deadline = None

    def envelope(self, kind):
        return dict(version=VERSION, kind=kind, vehicle_id=self.vehicle_id,
                    session_id=self.session_id, run_id=self.run_id)

    def addressed(self, msg):
        return (msg is not None and msg.get("vehicle_id") == self.vehicle_id
                and msg.get("session_id") == self.session_id)

    def command(self, msg):
        if not self.addressed(msg) or msg.get("kind") != "command":
            return "REJECTED"
        if not identifier(msg.get("run_id")) or not identifier(msg.get("command_id")):
            return "REJECTED"
        if msg.get("action") == "START":
            if self.closed:
                return "REJECTED"
            if self.run_id is not None:
                return "DUPLICATE" if (msg["run_id"], msg["command_id"]) == (self.run_id, self.command_id) else "REJECTED"
            self.run_id, self.command_id = msg["run_id"], msg["command_id"]
            return "STARTED"
        if msg.get("action") == "CANCEL" and self.run_id in (None, msg["run_id"]):
            # CANCEL may overtake START on a reconnect or replace its retained
            # sample before the mission receives it. Bind and close this session
            # so a delayed START cannot launch an already-aborted task.
            if self.run_id is None:
                self.run_id = msg["run_id"]
            self.close()
            return "CANCELLED"
        return "REJECTED"

    def close(self):
        self.closed = True
        if self.receipt == "PENDING":
            self.receipt = "CANCELLED"

    def complete(self, bay_number, now):
        if self.run_id is None or self.closed or self.result is not None:
            return
        if type(bay_number) is not int or bay_number not in (1, 2, 3):
            raise ValueError("bay number must be 1, 2, or 3")
        self.result = dict(self.envelope("docking_result"), event_id=uuid.uuid4().hex,
                           bay_number=bay_number, completed_at=now, clock="ros")
        self.receipt = "PENDING"
        self.next_send = now
        self.deadline = now + self.ack_timeout

    def outgoing(self, now):
        if self.closed or self.receipt != "PENDING":
            return None
        if now >= self.deadline:
            self.receipt = "TIMED_OUT"
            return None
        if now < self.next_send:
            return None
        self.next_send = now + 1.0
        return self.result

    def acknowledge(self, msg, now):
        if (not self.addressed(msg) or self.closed or self.receipt != "PENDING"
                or now >= self.deadline or msg.get("kind") != "receipt"
                or msg.get("run_id") != self.run_id
                or msg.get("event_id") != self.result["event_id"]
                or msg.get("status") != "RECEIVED"):
            return False
        self.receipt = "RECEIVED"
        return True
