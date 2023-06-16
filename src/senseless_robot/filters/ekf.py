import copy

from geometry_msgs.msg import PoseWithCovariance
import numpy as np
from numpy.typing import ArrayLike
from senseless_robot.states.utils import (
    heading_to_quat,
    quat_to_heading,
    posecovar_to_2darray,
)


class ExtendedKalmanFilter2D:
    def __init__(self) -> None:
        # Set up the internal variables.
        self._belief = PoseWithCovariance()
        self._belief.pose.orientation.w = 1.0
        self._belief.covariance = np.zeros((36,), dtype=np.float64).tolist()
        self._Q = np.eye(3)

    def predict(self, u: np.ndarray, dt: float = 1.0) -> None:
        # Get the prior position and yaw, and the pose covariance.
        x_pos, y_pos, yaw = self.x.flatten()
        P_prior = self.P

        # Compute the linear and angular movements.
        vdt, wdt = u.flatten() * dt

        cos_t = np.cos(yaw)
        sin_t = np.sin(yaw)

        cos_vdt = cos_t * vdt
        sin_vdt = sin_t * vdt

        # Compute the new 2D pose.
        pose2D = [x_pos + cos_vdt, y_pos + sin_vdt, yaw + wdt]

        # Scale the process noise based on the velocity.
        # NOTE: Got this trick from robot_localization ROS package.
        vx, wz = u.flatten()
        Vmat = np.eye(3) * np.linalg.norm([vx, 0.0, wz])
        vel_scaled_Q = Vmat @ self._Q @ np.transpose(Vmat)

        # Compute the 2D pose covariance.
        Fx = np.array([1.0, 0.0, -sin_vdt, 0.0, 1.0, cos_vdt, 0.0, 0.0, 1.0]).reshape(
            3, 3
        )
        P_post = (Fx @ P_prior @ np.transpose(Fx)) + (vel_scaled_Q * dt)

        # Update the pose and covariance.
        self.set_belief_2D(pose2D=pose2D, P_2D=P_post)

    def set_belief_2D(self, pose2D: ArrayLike, P_2D: ArrayLike) -> None:
        assert np.array(pose2D).shape == (3,)
        assert np.array(P_2D).shape == (3, 3)

        # Set the new 2D pose.
        self._belief.pose.position.x = pose2D[0]
        self._belief.pose.position.y = pose2D[1]
        self._belief.pose.position.z = 0.0

        qx, qy, qz, qw = heading_to_quat(pose2D[2])
        self._belief.pose.orientation.x = qx
        self._belief.pose.orientation.y = qy
        self._belief.pose.orientation.z = qz
        self._belief.pose.orientation.w = qw

        # Set the 2D pose covariance.
        self._belief.covariance = [0.0] * 36
        self._belief.covariance[0] = P_2D[0, 0]
        self._belief.covariance[1] = P_2D[0, 1]
        self._belief.covariance[5] = P_2D[0, 2]

        self._belief.covariance[6] = P_2D[1, 0]
        self._belief.covariance[7] = P_2D[1, 1]
        self._belief.covariance[11] = P_2D[1, 2]

        self._belief.covariance[30] = P_2D[2, 0]
        self._belief.covariance[31] = P_2D[2, 1]
        self._belief.covariance[35] = P_2D[2, 2]

    def set_belief_from_message(self, belief: PoseWithCovariance) -> None:
        # Set the position and orientation.
        self._belief.pose.position.x = belief.pose.position.x
        self._belief.pose.position.y = belief.pose.position.y

        self._belief.pose.orientation.x = belief.pose.orientation.x
        self._belief.pose.orientation.y = belief.pose.orientation.y
        self._belief.pose.orientation.z = belief.pose.orientation.z
        self._belief.pose.orientation.w = belief.pose.orientation.w

        # Set the covariance.
        self._belief.covariance = [0.0] * 36
        self._belief.covariance[0] = belief.covariance[0]
        self._belief.covariance[1] = belief.covariance[1]
        self._belief.covariance[5] = belief.covariance[5]

        self._belief.covariance[6] = belief.covariance[6]
        self._belief.covariance[7] = belief.covariance[7]
        self._belief.covariance[11] = belief.covariance[11]

        self._belief.covariance[30] = belief.covariance[30]
        self._belief.covariance[31] = belief.covariance[31]
        self._belief.covariance[35] = belief.covariance[35]

    @property
    def x(self) -> np.ndarray:
        x = self._belief.pose.position.x
        y = self._belief.pose.position.y
        yaw = quat_to_heading(self._belief.pose.orientation)
        return np.array([x, y, yaw], dtype=np.float64).reshape(3, 1)

    @property
    def P(self) -> np.ndarray:
        return posecovar_to_2darray(self._belief.covariance.copy())

    @property
    def belief(self) -> PoseWithCovariance:
        return copy.deepcopy(self._belief)

    @property
    def Q(self) -> np.ndarray:
        return self._Q

    @Q.setter
    def Q(self, new_value: np.ndarray) -> None:
        assert isinstance(new_value, np.ndarray)
        assert self._Q.shape == new_value.shape
        np.copyto(self._Q, new_value)
