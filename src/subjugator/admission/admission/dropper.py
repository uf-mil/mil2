import numpy as np
import gtsam
import admission as adm
from geometry_msgs.msg import Pose
from gtsam.symbol_shorthand import X, L

NOISE = gtsam.noiseModel.Diagonal.Sigmas([0.1] * 6)
DET_NOISE = gtsam.noiseModel.Diagonal.Sigmas([0.5] * 3)
CAL = gtsam.Cal3_S2(80, 640, 360)
CAM = gtsam.PinholeCameraCal3_S2(gtsam.Pose3(), CAL)

def odom_pose(odom):
    o_p = odom.pose.pose.position
    o_q = odom.pose.pose.orientation
    pose = gtsam.Pose3(
        gtsam.Rot3(o_q.w, o_q.x, o_q.y, o_q.z),
        [o_p.x, o_p.y, o_p.z]
    )
    noise = gtsam.noiseModel.Gaussian.Covariance(
        odom.pose.covariance.reshape(6, 6)
    )
    return pose, noise

async def estimate_bins():
    # measure initial odom
    odom_prev, initial_noise = odom_pose(await adm.odom_sub())

    # initialize smoother
    smoother = gtsam.IncrementalFixedLagSmoother(100)
    factors = gtsam.NonlinearFactorGraph()
    values = gtsam.Values()
    ix, il = 0, 0

    factors.addPriorPose3(X(0), odom_prev, NOISE)
    values.insert(X(0), odom_prev)

    smoother.update(factors, values, {X(0): 0})
    estimate = smoother.calculateEstimate()

    # landmarks
    landmarks = []

    async for yolo, odom in adm.Join(adm.yolo_sub, adm.odom_sub):
        if yolo:
            factors = gtsam.NonlinearFactorGraph()
            values = gtsam.Values()
            timestamps = {}

            for det in yolo.detections:
                landmark = CAM.backproject([
                    det.bbox.center.position.x,
                    det.bbox.center.position.y
                ], 1)

                planar_dist = 10
                factors.add(gtsam.BearingRangeFactor3D(
                    X(ix), L(il), gtsam.Unit3(landmark),
                    planar_dist * np.linalg.norm(landmark), DET_NOISE
                ))

                pose_estimate = estimate.atPose3(X(ix))
                values.insert(
                    L(il), pose_estimate.transformFrom(landmark * planar_dist)
                )
                timestamps[L(il)] = ix
                il += 1

            smoother_result = smoother.update(factors, values, timestamps)
            estimate = smoother.calculateEstimate()
        elif odom:
            odom_next, _ = odom_pose(odom)
            odom_diff = odom_prev.between(odom_next)

            factors = gtsam.NonlinearFactorGraph()
            values = gtsam.Values()
            factors.add(gtsam.BetweenFactorPose3(
                X(ix), X(ix + 1), odom_diff, NOISE
            ))

            pose_estimate = estimate.atPose3(X(ix))
            values.insert(X(ix + 1), pose_estimate.compose(odom_diff))

            smoother_result = smoother.update(factors, values, {X(ix + 1): ix + 1})
            estimate = smoother.calculateEstimate()
            print(pose_estimate)

            odom_prev = odom_next
            ix += 1

if __name__ == "__main__":
    adm.run(estimate_bins())
