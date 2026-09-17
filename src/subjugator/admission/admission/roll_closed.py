import math

import numpy as np
import transforms3d
from geometry_msgs.msg import Pose, Quaternion, Wrench

from admission import adm


def quat(q):
    quat = Quaternion()
    quat.w = q[0]
    quat.x = q[1]
    quat.y = q[2]
    quat.z = q[3]
    return quat


async def roll_closed():
    goal = Pose()
    goal.position.z = -1.0
    adm.goal_pub.publish(goal)

    while odom := await adm.odom_sub():
        dx = odom.pose.pose.position.x - goal.position.x
        dy = odom.pose.pose.position.y - goal.position.y
        dz = odom.pose.pose.position.z - goal.position.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < 0.2:
            break

    roll_amt = 0
    roll_prev = 0

    wrench = Wrench()
    wrench.torque.x = 10.0
    adm.add_wrench_pub.publish(wrench)

    while odom := await adm.odom_sub():
        q = odom.pose.pose.orientation
        mat = transforms3d.quaternions.quat2mat([q.w, q.x, q.y, q.z])

        mat[:, 0] = [1, 0, 0]  # x

        y = mat[:, 1]
        y[0] = 0
        y /= np.linalg.norm(y)

        roll = math.atan2(y[2], y[1]) + 1
        y[1], y[2] = math.cos(roll), math.sin(roll)

        mat[:, 2] = np.cross(mat[:, 0], y)  # z

        goal.orientation = quat(transforms3d.quaternions.mat2quat(mat))
        adm.goal_pub.publish(goal)

        roll_amt += math.remainder(roll - roll_prev, math.tau)
        roll_prev = roll
        if roll_amt >= 315 * (math.pi / 180):
            break

    goal.orientation = Quaternion()
    adm.goal_pub.publish(goal)


async def main():
    try:
        await roll_closed()
    finally:
        goal = Pose()
        goal.position.z = -1.0
        adm.goal_pub.publish(Pose())
        adm.add_wrench_pub.publish(Wrench())


if __name__ == "__main__":
    adm.run(main())
