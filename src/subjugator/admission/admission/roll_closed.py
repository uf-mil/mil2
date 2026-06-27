import math
import transforms3d
import admission as adm
from geometry_msgs.msg import Pose, Quaternion, Wrench

def quat(r, p, y):
    q = transforms3d.euler.euler2quat(r, p, y)
    quat = Quaternion()
    quat.w = q[0]
    quat.x = q[1]
    quat.y = q[2]
    quat.z = q[3]
    return quat

async def roll_closed():
    goal = Pose()
    goal.position.z = -1.0
    goal.orientation = quat(0, 0, 0)
    adm.goal_pub.publish(goal)

    while odom := await adm.odom_sub():
        dx = odom.pose.pose.position.x - goal.position.x
        dy = odom.pose.pose.position.y - goal.position.y
        dz = odom.pose.pose.position.z - goal.position.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < 0.2:
            break

    q = odom.pose.pose.orientation
    roll_prev, _, yaw_prev = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])
    roll_amt = 0

    wrench = Wrench()
    wrench.torque.x = 10.0
    adm.add_wrench_pub.publish(wrench)

    while odom := await adm.odom_sub():
        q = odom.pose.pose.orientation
        roll, _, _ = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])

        goal.orientation = quat(roll + 0.5, 0, yaw_prev)
        adm.goal_pub.publish(goal)

        roll_amt += math.remainder(roll - roll_prev, math.tau)
        roll_prev = roll
        print(roll + 0.5, roll_amt)
        if roll_amt >= 315 * (math.pi / 180):
            break

    goal.orientation = quat(0, 0, yaw_prev)
    adm.goal_pub.publish(goal)

if __name__ == "__main__":
    adm.run(roll_closed())
