import math
import transforms3d
import admission as adm
from geometry_msgs.msg import Pose, Quaternion, Wrench

def quatRPY(r, p, y):
    q = transforms3d.taitbryan.euler2quat(y, p, r)
    quat = Quaternion()
    quat.w = q[0]
    quat.x = q[1]
    quat.y = q[2]
    quat.z = q[3]
    return quat

async def roll_closed():
    goal = Pose()
    goal.position.z = -1.0
    goal.orientation = quatRPY(0, 0, 0)
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
        _, _, roll = transforms3d.taitbryan.quat2euler([q.w, q.x, q.y, q.z])

        goal.orientation = quatRPY(roll + 0.5, 0, 0)
        adm.goal_pub.publish(goal)

        roll_amt += math.remainder(roll - roll_prev, math.tau)
        roll_prev = roll
        print(roll + 0.5, roll_amt)
        if roll_amt >= 315 * (math.pi / 180):
            break

    goal.orientation = quatRPY(0, 0, 0)
    adm.goal_pub.publish(goal)

if __name__ == "__main__":
    adm.run(roll_closed())
