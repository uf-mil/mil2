import datetime

from rclpy.time import Time


def rospy_to_datetime(time: Time) -> datetime.datetime:
    return datetime.datetime.utcfromtimestamp(time.to_sec())


def datetime_to_rospy(dt: datetime.datetime) -> Time:
    dt_utc = dt.replace(tzinfo=datetime.timezone.utc)
    seconds = dt_utc.timestamp()
    nanoseconds = int(seconds * 1e9)
    return Time(seconds=nanoseconds // 1_000_000_000, nanoseconds=nanoseconds % 1_000_000_000)
