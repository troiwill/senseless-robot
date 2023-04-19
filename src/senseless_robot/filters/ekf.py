from __future__ import annotations
import numpy as np
from senseless_robot.filters.belief import GaussianBelief


class ExtendedKalmanFilter2D:
    def __init__(self, x_dim: int, u_dim: int, z_dim: int) -> ExtendedKalmanFilter2D:
        # Sanity checks.
        assert isinstance(u_dim, int)
        assert 0 < u_dim <= 2
        assert isinstance(z_dim, int)
        assert 0 < z_dim <= 3

        self._belief = GaussianBelief(state_dim=x_dim)

        self._u_dim = u_dim
        self._z_dim = z_dim

        self._V = np.eye(u_dim, dtype=np.float64)

    def predict(self, u: np.ndarray, dt: float = 1.0) -> None:
        theta = self._belief.x[2, 0]
        vdt, wdt = u.flatten() * dt

        cos_t = np.cos(theta)
        sin_t = np.sin(theta)

        cos_vdt = cos_t * vdt
        sin_vdt = sin_t * vdt

        dp = np.array([cos_vdt, sin_vdt, wdt]).reshape(3, 1)
        Fx = np.array([[1.0, 0.0, -sin_vdt], [0.0, 1.0, cos_vdt], [0, 0, 1]]).reshape(3,3)
        Fv = np.array([[cos_t * dt, 0], [sin_t * dt, 0], [0, dt]]).reshape(3,2)

        x = self._belief.x + dp
        theta = x[2,0]
        while theta < -np.pi:
            theta += (np.pi * 2.0)
        while theta >= np.pi:
            theta -= (np.pi * 2.0)
        x[2,0] = theta

        self._belief.x = x
        self._belief.P = (Fx @ self._belief.P @ Fx.T) + (Fv @ self._V @ Fv.T)

    def set_belief(self, belief: GaussianBelief) -> None:
        assert isinstance(belief, GaussianBelief)
        self._belief = belief

    @property
    def x(self) -> np.ndarray:
        return self._belief.x

    @property
    def P(self) -> np.ndarray:
        return self._belief.P

    @property
    def belief(self) -> GaussianBelief:
        return self._belief

    @property
    def V(self) -> np.ndarray:
        return self._V

    @V.setter
    def V(self, new_value: np.ndarray) -> None:
        assert isinstance(new_value, np.ndarray)
        assert self._V.shape == new_value.shape
        self._V[:] = new_value
