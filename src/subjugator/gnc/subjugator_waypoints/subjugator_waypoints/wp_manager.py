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
        p.position.x = float(self.x)
        p.position.y = float(self.y)
        p.position.z = float(self.z)
        p.orientation.x = float(self.i)
        p.orientation.y = float(self.j)
        p.orientation.z = float(self.k)
        p.orientation.w = float(self.w)

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
    # Create a quaternion representing just the position (with w=0 for pure vector part)
    p = MyPose(point.x, point.y, point.z, 0, 0, 0, 0)
    
    # Perform rotation: q * p * q^-1
    rotated = quaternion_multiply(quaternion_multiply(q, p), quaternion_conjugate(q))
    
    # Return the rotated position
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

def inverse_transform_pose(local_pose: MyPose, origin: MyPose) -> MyPose:
    # First apply rotation to the position part
    local_position = MyPose(local_pose.x, local_pose.y, local_pose.z, 0, 0, 0, 0)
    rotated_pos = rotate_point_by_quaternion(local_position, origin)
    
    # Then add the origin position
    world_x = rotated_pos.x + origin.x
    world_y = rotated_pos.y + origin.y
    world_z = rotated_pos.z + origin.z
    
    # Calculate world orientation by multiplying quaternions
    world_orientation = quaternion_multiply(origin, local_pose)
    
    return MyPose(world_x, world_y, world_z,
                world_orientation.i, world_orientation.j, world_orientation.k, world_orientation.w)

def transform_poses(poses, origin):
    return [transform_pose(p, origin) for p in poses]

class WaypointManager:
    def __init__(self):
        self.waypoints: dict[str, MyPose] = {}

    # if the waypoint already exists, it is reset
    def add_waypoint(self, wp_name: str, wp: MyPose):
        self.waypoints[wp_name] = wp

    def print(self):
        for (key,val) in self.waypoints.items():
            print(key + " ", val.x, " ",val.y, " ",val.z, " ",val.i, " ",val.j, " ",val.k, " ",val.w, " ")

    # if the waypoint doesn't exist, does nothing
    def remove_waypoint(self, wp_name: str):
        del self.waypoints[wp_name]

    def reset_localization(self, current_pose: MyPose):
        for key, wp in self.waypoints.items():
            # Calculate vector from current position to waypoint
            dx = wp.x - current_pose.x
            dy = wp.y - current_pose.y
            dz = wp.z - current_pose.z
            
            # For rotation, we need the conjugate of the quaternion
            # The conjugate is (−i, −j, −k, w)
            q_i = -current_pose.i
            q_j = -current_pose.j
            q_k = -current_pose.k
            q_w = current_pose.w
            
            # Rotate the vector using our new function
            new_x, new_y, new_z = rotate_vector(dx, dy, dz, q_i, q_j, q_k, q_w)
            
            # For orientation, multiply quaternions (using your existing function)
            # Create orientation quaternion for waypoint
            wp_quat = MyPose(0, 0, 0, wp.i, wp.j, wp.k, wp.w)
            
            # Create conjugate quaternion for current pose
            conj_quat = MyPose(0, 0, 0, q_i, q_j, q_k, q_w)
            
            # Calculate new orientation
            new_orientation = quaternion_multiply(conj_quat, wp_quat)
            
            # Update the waypoint
            self.waypoints[key] = MyPose(
                new_x, new_y, new_z,
                new_orientation.i, new_orientation.j, new_orientation.k, new_orientation.w
            )

def rotate_vector(v_x, v_y, v_z, q_i, q_j, q_k, q_w):
    """Rotate a vector [x,y,z] by a quaternion [i,j,k,w]"""
    # Formula from: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    
    # Pre-calculate some common terms
    ii = q_i * q_i
    jj = q_j * q_j
    kk = q_k * q_k
    ij = q_i * q_j
    ik = q_i * q_k
    jk = q_j * q_k
    iw = q_i * q_w
    jw = q_j * q_w
    kw = q_k * q_w
    
    # Calculate rotated vector
    x = v_x * (1 - 2*(jj + kk)) + v_y * 2*(ij - kw) + v_z * 2*(ik + jw)
    y = v_x * 2*(ij + kw) + v_y * (1 - 2*(ii + kk)) + v_z * 2*(jk - iw)
    z = v_x * 2*(ik - jw) + v_y * 2*(jk + iw) + v_z * (1 - 2*(ii + jj))
    
    return x, y, z
