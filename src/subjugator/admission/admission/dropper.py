import dataclasses
import math
from operator import itemgetter

import numpy as np
import scipy.linalg

import gtsam
import admission as adm
from geometry_msgs.msg import Pose
from gtsam.symbol_shorthand import X, L
from visualization_msgs.msg import Marker

ODOM_NOISE = gtsam.noiseModel.Diagonal.Sigmas([0.01] * 6)
LANDMARK_COV_INV = np.linalg.inv(np.diag([2, 1, 1]))
LANDMARK_NOISE = gtsam.noiseModel.Diagonal.Sigmas([0.1, 0.1, 2])
CAL = gtsam.Cal3_S2(80, 640, 360)
CAM = gtsam.PinholePoseCal3_S2(gtsam.Pose3(), CAL)

@dataclasses.dataclass(slots=True)
class Landmark:
    mean: np.ndarray
    cov_inv: np.ndarray

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

    factors.addPriorPose3(X(0), odom_prev, ODOM_NOISE)
    values.insert(X(0), odom_prev)

    smoother.update(factors, values, {X(0): 0})
    estimate = smoother.calculateEstimate()

    # landmarks
    landmarks = {}

    async for yolo, odom in adm.Join(adm.yolo_sub, adm.odom_sub):
        if yolo:
            for key, landmark in landmarks.items():
                if not estimate.exists(key): continue
                landmark.mean = estimate.atPoint3(key)
                landmark.cov_inv = np.linalg.inv(
                    smoother.marginalCovariance(key)
                )

            for i, landmark in enumerate(landmarks.values()):
                if landmark.mean is None: continue

                marker = Marker()
                marker.header.frame_id = "odom"
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.color.a = marker.color.g = 1.0

                u, p = scipy.linalg.polar(np.linalg.inv(landmark.cov_inv))

                marker.scale.x, marker.scale.y, marker.scale.z = np.fmax(
                    np.full(3, 1), p.diagonal()
                )
                (
                    marker.pose.orientation.w,
                    marker.pose.orientation.x,
                    marker.pose.orientation.y,
                    marker.pose.orientation.z,
                ) = gtsam.Rot3(u).toQuaternion().coeffs()
                (
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z
                ) = landmark_pos
                adm.marker_pub.publish(marker)

            # try:
            #     goal = [l.mean for l in landmarks.values()][0]
            #     pose = Pose()
            #     pose.position.x, pose.position.y, pose.position.z = goal
            #     adm.goal_pub.publish(pose)
            # except IndexError:
            #     pass

            if not yolo.detections: continue

            factors = gtsam.NonlinearFactorGraph()
            values = gtsam.Values()
            timestamps = {}

            pose_estimate = estimate.atPose3(X(ix))

            for det in yolo.detections:
                landmark_vec = CAM.backproject([
                    det.bbox.center.position.x,
                    det.bbox.center.position.y
                ], 1)
                # backprojection:
                #      neg y
                # neg x     pos x
                #      pos y
                # depth: z=1
                landmark_vec[:] = 1, -landmark_vec[0], -landmark_vec[1]

                size = math.sqrt(det.bbox.size.x ** 2 + det.bbox.size.y ** 2)
                planar_dist = 320 * 0.4 / (math.tan(40 * math.pi / 180) * size)

                # associate closest landmark
                landmark_pos = pose_estimate.transformFrom(landmark_vec * planar_dist)

                pose_rot = pose_estimate.rotation().matrix()
                sigma_inv = pose_rot.T @ LANDMARK_COV_INV @ pose_rot

                closest_landmarks = []
                for i, other in landmarks.items():
                    if other.mean is None: continue
                    error = landmark_pos - other.mean
                    maha = error.T @ sigma_inv @ error
                    closest_landmarks.append((maha, i))

                if closest_landmarks:
                    maha, sym = sorted(closest_landmarks)[0]
                else:
                    maha, sym = 99, None

                if maha > 1:
                    # create new landmark
                    sym = L(il)
                    landmarks[sym] = Landmark(None, None)
                    values.insert(sym, landmark_pos)
                    il += 1
                elif not estimate.exists(sym) and not values.exists(sym):
                    # restore previous landmark
                    readd = landmarks[sym]
                    values.insert(sym, readd.mean)
                    factors.addPriorPoint3(
                        sym, readd.mean,
                        gtsam.noiseModel.Gaussian.Covariance(
                            np.linalg.inv(readd.cov_inv)
                        )
                    )

                factors.add(gtsam.BearingRangeFactor3D(
                    X(ix), sym, gtsam.Unit3(landmark_vec),
                    planar_dist * np.linalg.norm(landmark_vec), LANDMARK_NOISE
                ))

                timestamps[sym] = ix

            smoother_result = smoother.update(factors, values, timestamps)
            estimate = smoother.calculateEstimate()
        elif odom:
            odom_next, _ = odom_pose(odom)
            odom_diff = odom_prev.between(odom_next)

            factors = gtsam.NonlinearFactorGraph()
            values = gtsam.Values()
            factors.add(gtsam.BetweenFactorPose3(
                X(ix), X(ix + 1), odom_diff, ODOM_NOISE
            ))

            pose_estimate = estimate.atPose3(X(ix))
            values.insert(X(ix + 1), pose_estimate.compose(odom_diff))

            smoother_result = smoother.update(factors, values, {X(ix + 1): ix + 1})
            estimate = smoother.calculateEstimate()

            odom_prev = odom_next
            ix += 1

if __name__ == "__main__":
    adm.run(estimate_bins())
