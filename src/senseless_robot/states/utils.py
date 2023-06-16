from geometry_msgs.msg import PoseWithCovariance, Pose, Quaternion
import numpy as np
from spatialmath import UnitQuaternion
from typing import List, Tuple


def heading_to_quat(theta: float) -> List[float]:
    quat = UnitQuaternion.Rz(theta, unit="rad")
    return [quat.v[0], quat.v[1], quat.v[2], quat.s]


def posecovar_to_2darray(covar: List[float]) -> np.ndarray:
    return np.array(
        [
            covar[0],
            covar[1],
            covar[5],
            covar[6],
            covar[7],
            covar[11],
            covar[30],
            covar[31],
            covar[35],
        ],
        dtype=np.float64,
    ).reshape(3, 3)


def posewithcovar_to_belief2D(
    pose: PoseWithCovariance,
) -> Tuple[np.ndarray, np.ndarray]:
    # Extract the 2D pose.
    theta = quat_to_heading(quat=pose.pose.orientation)
    x = np.array([pose.pose.position.x, pose.pose.position.y, theta]).reshape(3, 1)

    # Extract the 2D pose covariance.
    covar = np.array(
        [pose.covariance[i] for i in [0, 1, 5, 6, 7, 11, 30, 31, 35]]
    ).reshape(3, 3)

    return x, covar


def quat_to_heading(quat: Quaternion) -> float:
    return UnitQuaternion(s=quat.w, v=[quat.x, quat.y, quat.z]).rpy(
        unit="rad", order="zyx"
    )[2]


def xyquat_to_ros_pose(x: float, y: float, quat: List[float]) -> Pose:
    pose = Pose()

    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0

    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose


def xyquat_covar_to_ros_posewithcovar(
    x: float, y: float, quat: List[float], covar: List[float]
) -> PoseWithCovariance:
    belief = PoseWithCovariance()

    belief.pose = xyquat_to_ros_pose(x=x, y=y, quat=quat)

    belief.covariance = [0.0] * 36
    belief.covariance[0] = covar[0]
    belief.covariance[1] = covar[1]
    belief.covariance[5] = covar[2]

    belief.covariance[6] = covar[3]
    belief.covariance[7] = covar[4]
    belief.covariance[11] = covar[5]

    belief.covariance[30] = covar[6]
    belief.covariance[31] = covar[7]
    belief.covariance[35] = covar[8]

    return belief
