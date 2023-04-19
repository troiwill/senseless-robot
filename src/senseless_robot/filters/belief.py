from __future__ import annotations
import numpy as np


class GaussianBelief:
    def __init__(self, state_dim: int) -> GaussianBelief:
        # Sanity check.
        assert isinstance(state_dim, int)
        assert 0 < state_dim <= 15

        self._x = np.zeros((state_dim, 1), dtype=np.float64)
        self._P = np.eye(state_dim, dtype=np.float64)

    @classmethod
    def xP(cls, x: np.ndarray, P: np.ndarray) -> GaussianBelief:
        # Sanity checks.
        assert len(x.shape) == 2
        assert x.shape[1] == 1
        assert len(P.shape) == 2
        assert P.shape == (x.shape[0], x.shape[0])

        rv = cls(x.shape[0])
        rv.x = x
        rv.P = P
        return rv

    @property
    def x(self) -> np.ndarray:
        return self._x.copy()

    @property
    def P(self) -> np.ndarray:
        return self._P.copy()

    def belief(self) -> tuple[np.ndarray, np.ndarray]:
        return (self.x, self.P)

    def copy(self) -> GaussianBelief:
        rv = GaussianBelief(self._x.shape[0])
        rv.x = self._x
        rv.P = self._P

        return rv

    @x.setter
    def x(self, new_value: np.ndarray) -> None:
        # Sanity check.
        assert self._x.shape == new_value.shape
        np.copyto(self._x, new_value)

    @P.setter
    def P(self, new_value: np.ndarray) -> None:
        # Sanity check.
        assert self._P.shape == new_value.shape
        np.copyto(self._P, new_value)

    def __str__(self) -> str:
        return f"x: {self._x.flatten()}\nP:\n{self._P}"