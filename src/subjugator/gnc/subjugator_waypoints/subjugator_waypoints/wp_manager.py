from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

class MyPose:
    def __init__(self, x: float, y: float, z: float, i: float, j: float, k: float, w: float):
        self.x: float = x
        self.y: float = y
        self.z: float = z
        self.i: float = i
        self.j: float = j
        self.k: float = k
        self.w: float = w

    def as_geo_msg_pose(self) -> Pose:
        p = Pose()
        p.position.x = self.x
        p.position.y = self.y
        p.position.z = self.z
        p.orientation.x = self.i
        p.orientation.y = self.j
        p.orientation.z = self.k
        p.orientation.w = self.w

        return p

def pose_from_odom(odom: Odometry) -> MyPose:
    return MyPose(
        odom.pose.pose.position.x,
        odom.pose.pose.position.y,
        odom.pose.pose.position.z,
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w
    )

def quaternion_conjugate(q: MyPose) -> MyPose:
    return MyPose(0, 0, 0, -q.i, -q.j, -q.k, q.w)

def quaternion_multiply(q1: MyPose, q2: MyPose) -> MyPose:
    i = q1.w * q2.i + q1.i * q2.w + q1.j * q2.k - q1.k * q2.j
    j = q1.w * q2.j - q1.i * q2.k + q1.j * q2.w + q1.k * q2.i
    k = q1.w * q2.k + q1.i * q2.j - q1.j * q2.i + q1.k * q2.w
    w = q1.w * q2.w - q1.i * q2.i - q1.j * q2.j - q1.k * q2.k
    return MyPose(0, 0, 0, i, j, k, w)

def rotate_point_by_quaternion(point: MyPose, q: MyPose) -> MyPose:
    p = MyPose(point.x, point.y, point.z, 0, 0, 0, 0)
    q_conj = quaternion_conjugate(q)
    rotated = quaternion_multiply(quaternion_multiply(q, p), q_conj)
    return MyPose(rotated.x, rotated.y, rotated.z, 0, 0, 0, 0)

def transform_pose(pose: MyPose, origin: MyPose) -> MyPose:
    rel_x = pose.x - origin.x
    rel_y = pose.y - origin.y
    rel_z = pose.z - origin.z
    rel_position = MyPose(rel_x, rel_y, rel_z, 0, 0, 0, 0)

    origin_inv = quaternion_conjugate(origin)
    rotated_pos = rotate_point_by_quaternion(rel_position, origin_inv)
    new_rotation = quaternion_multiply(origin_inv, pose)

    return MyPose(rotated_pos.x, rotated_pos.y, rotated_pos.z,
                new_rotation.i, new_rotation.j, new_rotation.k, new_rotation.w)

def transform_poses(poses, origin):
    return [transform_pose(p, origin) for p in poses]

class WaypointManager:
    def __init__(self):
        self.waypoints: dict[str, MyPose] = {}

    # if the waypoint already exists, it is reset
    def add_waypoint(self, wp_name: str, wp: MyPose):
        self.waypoints[wp_name] = wp

    # if the waypoint doesn't exist, does nothing
    def remove_waypoint(self, wp_name: str):
        del self.waypoints[wp_name]

    def reset_localization(self, current_pose: MyPose):
        for (key,val) in self.waypoints.items():
            self.waypoints[key] = transform_pose(val, current_pose)

