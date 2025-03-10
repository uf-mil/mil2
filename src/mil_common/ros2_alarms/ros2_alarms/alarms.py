from __future__ import annotations

import json
import traceback
from typing import Callable

import rclpy
from ros2_alarms_msgs.msg import Alarm as AlarmMsg
from ros2_alarms_msgs.srv import (
    AlarmGet,
    AlarmGetRequest,
    AlarmGetResponse,
    AlarmSet,
    AlarmSetRequest,
)

class Alarm:
   pass

class AlarmBroadcaster:
    pass

class AlarmListener:
    pass

class HeartbeatMonitor:
    pass

def parse_json_str(json_str: str) -> dict:
    pass