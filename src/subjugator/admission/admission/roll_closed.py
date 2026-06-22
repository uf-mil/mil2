import math
import transforms3d
import admission as adm
from geometry_msgs.msg import Pose

async def roll_closed():
    goal = Pose()
    goal_q = transforms3d.euler.euler2quat(0, 0, 0)
    goal.position.z = -1.0
    goal.orientation.w = goal_q[0]
    goal.orientation.x = goal_q[1]
    goal.orientation.y = goal_q[2]
    goal.orientation.z = goal_q[3]
    adm.goal_pub.publish(goal)

    while odom := await adm.odom_sub:
        dx = odom.pose.pose.position.x - goal.position.x
        dy = odom.pose.pose.position.y - goal.position.y
        dz = odom.pose.pose.position.z - goal.position.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < 0.2:
            break

    q = odom.pose.pose.orientation
    roll_prev, _, _ = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])
    roll_amt = 0

    while odom := await adm.odom_sub:
        q = odom.pose.pose.orientation
        roll, _, _ = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])

        roll_amt += math.remainder(roll - roll_prev, math.tau)
        roll_prev = roll
        print(roll_amt)
        if roll_amt >= 315 * (math.pi / 180):
            break

if __name__ == "__main__":
    adm.run(roll_closed())
