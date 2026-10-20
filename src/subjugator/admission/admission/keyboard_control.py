import sys
import termios
import tty

import gtsam
from geometry_msgs.msg import Pose

from admission import adm

transforms = {
    key: gtsam.Pose3(gtsam.Rot3(), t)
    for t, key in zip(
        [
            [0.1, 0, 0],
            [-0.1, 0, 0],
            [0, 0.1, 0],
            [0, -0.1, 0],
            [0, 0, 0.1],
            [0, 0, -0.1],
        ],
        ["w", "s", "a", "d", " ", "c"],
    )
}

transforms["q"] = gtsam.Pose3(gtsam.Rot3.Yaw(0.1), [0, 0, 0])
transforms["e"] = gtsam.Pose3(gtsam.Rot3.Yaw(-0.1), [0, 0, 0])


async def main():
    try:
        restore = tty.setcbreak(sys.stdin)
        pose = gtsam.Pose3()
        while True:
            c = sys.stdin.read(1)
            if c in transforms:
                pose = pose.transformPoseFrom(transforms[c])
                print(pose.translation(), pose.rotation().yaw())
                goal = Pose()
                (goal.position.x, goal.position.y, goal.position.z) = pose.translation()
                (
                    goal.orientation.x,
                    goal.orientation.y,
                    goal.orientation.z,
                    goal.orientation.w,
                ) = (
                    pose.rotation().toQuaternion().coeffs()
                )
                adm.goal_pub.publish(goal)
            else:
                print("unknown c", c)
    finally:
        tty.tcsetattr(sys.stdin, termios.TCSAFLUSH, restore)


if __name__ == "__main__":
    adm.run(main())
