from rclpy import Node
import rclpy

class Pose:
    def __init__(self, x: float, y: float, z: float, i: float, j: float, k: float, w: float):
        self.x: float = x
        self.y: float = y
        self.z: float = z
        self.i: float = i
        self.j: float = j
        self.k: float = k
        self.w: float = w

def quaternion_conjugate(q: Pose) -> Pose:
    return Pose(0, 0, 0, -q.i, -q.j, -q.k, q.w)

def quaternion_multiply(q1: Pose, q2: Pose) -> Pose:
    i = q1.w * q2.i + q1.i * q2.w + q1.j * q2.k - q1.k * q2.j
    j = q1.w * q2.j - q1.i * q2.k + q1.j * q2.w + q1.k * q2.i
    k = q1.w * q2.k + q1.i * q2.j - q1.j * q2.i + q1.k * q2.w
    w = q1.w * q2.w - q1.i * q2.i - q1.j * q2.j - q1.k * q2.k
    return Pose(0, 0, 0, i, j, k, w)

def rotate_point_by_quaternion(point: Pose, q: Pose) -> Pose:
    p = Pose(point.x, point.y, point.z, 0, 0, 0, 0)
    q_conj = quaternion_conjugate(q)
    rotated = quaternion_multiply(quaternion_multiply(q, p), q_conj)
    return Pose(rotated.x, rotated.y, rotated.z, 0, 0, 0, 0)

def transform_pose(pose: Pose, origin: Pose) -> Pose:
    rel_x = pose.x - origin.x
    rel_y = pose.y - origin.y
    rel_z = pose.z - origin.z
    rel_position = Pose(rel_x, rel_y, rel_z, 0, 0, 0, 0)

    origin_inv = quaternion_conjugate(origin)
    rotated_pos = rotate_point_by_quaternion(rel_position, origin_inv)
    new_rotation = quaternion_multiply(origin_inv, pose)

    return Pose(rotated_pos.x, rotated_pos.y, rotated_pos.z,
                new_rotation.i, new_rotation.j, new_rotation.k, new_rotation.w)

def transform_poses(poses, origin):
    return [transform_pose(p, origin) for p in poses]

class WaypointNode(Node):
    def __init__(self):
        super().__init__('waypoint_node')

        self.waypoints: dict[str, Pose] = {}

    def add_waypoint(self, wp_name: str, wp: Pose):
        self.waypoints[wp_name] = wp

    def add_current_pose_waypoint(self, wp_name: str):
        pass

    def reset_localization(self, current_pose: Pose):
        for (key,val) in self.waypoints.items():
            self.waypoints[key] = transform_pose(val, current_pose)

def main():
    pass

if __name__ == "__main__":
    main()
