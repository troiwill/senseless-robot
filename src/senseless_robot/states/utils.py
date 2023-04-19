from __future__ import annotations
from geometry_msgs.msg import PoseWithCovariance, Pose
import numpy as np
from senseless_robot.filters.belief import GaussianBelief
from spatialmath import UnitQuaternion


def heading_to_quat(theta: float) -> list[float]:
    quat = UnitQuaternion.Rz(theta, unit="rad")
    return [quat.v[0], quat.v[1], quat.v[2], quat.s]


def posewithcovar_to_belief(pose: PoseWithCovariance) -> GaussianBelief:
    position = [ pose.pose.position.x, pose.pose.position.y ]
    quat = pose.pose.orientation
    _, _, theta = UnitQuaternion(s=quat.w, v=[quat.x, quat.y, quat.z]).rpy(order="zyx")
    x = np.array(position + [ theta ]).reshape(3,1)

    covar = np.array([ pose.covariance[i] for i in [0, 1, 5, 6, 7, 11, 30, 31, 35]]).reshape(3,3)

    return GaussianBelief.xP(x=x, P=covar)


def xyquat_to_ros_pose(x: float, y: float, quat: list[float]) -> Pose:
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
    x: float, y: float, quat: list[float], covar: list[float]
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
