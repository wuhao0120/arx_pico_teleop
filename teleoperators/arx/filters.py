"""Smoothing filters for VR teleoperation.

Three independent, side-effect-free utilities:

- ``OneEuroFilter``: scalar adaptive low-pass (Casiez et al., CHI 2012).
  Use one instance per axis to smooth VR controller xyz position.
- ``SlerpEMA``: quaternion exponential moving average via spherical
  interpolation. Avoids the normalization error of naive component-wise EMA.
  Quaternion convention: ``[w, x, y, z]`` (matches ``teleop_arx.py``).
- ``JointRateLimiter``: per-joint EMA + max-velocity clamp. Smooths IK
  joint solutions and prevents single-frame jumps from being commanded.

All classes expose ``reset()`` so callers can re-seed state on activation
or episode reset to avoid cold-start transients.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as R


class OneEuroFilter:
    """Scalar 1-Euro filter.

    Tuning:
        - ``min_cutoff``: lower => more smoothing at low speeds (less jitter).
        - ``beta``: higher => less lag at high speeds.
        - ``d_cutoff``: cutoff for the derivative low-pass; 1.0 is usually fine.
    """

    def __init__(
        self,
        min_cutoff: float = 1.0,
        beta: float = 0.0,
        d_cutoff: float = 1.0,
    ) -> None:
        assert min_cutoff > 0.0, "min_cutoff must be positive"
        assert d_cutoff > 0.0, "d_cutoff must be positive"
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self._x_prev: Optional[float] = None
        self._dx_prev: float = 0.0
        self._t_prev: Optional[float] = None

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        # Standard exponential smoothing alpha for cutoff frequency `cutoff`.
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def reset(self, x0: float = 0.0, t0: Optional[float] = None) -> None:
        self._x_prev = float(x0)
        self._dx_prev = 0.0
        self._t_prev = t0

    def __call__(self, x: float, t: Optional[float] = None) -> float:
        x = float(x)
        if t is None:
            t = time.perf_counter()
        if self._x_prev is None or self._t_prev is None:
            self.reset(x, t)
            return x
        dt = max(t - self._t_prev, 1e-6)

        dx = (x - self._x_prev) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        dx_hat = a_d * dx + (1.0 - a_d) * self._dx_prev

        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1.0 - a) * self._x_prev

        self._x_prev = x_hat
        self._dx_prev = dx_hat
        self._t_prev = t
        return x_hat


class SlerpEMA:
    """Quaternion EMA via spherical interpolation.

    Quaternion format: ``[w, x, y, z]`` (matches ``meshcat.transformations``
    and ``teleop_arx.py``).
    """

    def __init__(self) -> None:
        self._q_prev: Optional[np.ndarray] = None  # [w, x, y, z]

    @staticmethod
    def _to_xyzw(q_wxyz: np.ndarray) -> np.ndarray:
        return np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], dtype=float)

    @staticmethod
    def _to_wxyz(q_xyzw: np.ndarray) -> np.ndarray:
        return np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]], dtype=float)

    def reset(self, q0_wxyz: Optional[np.ndarray] = None) -> None:
        self._q_prev = None if q0_wxyz is None else np.asarray(q0_wxyz, dtype=float).copy()

    def step(self, q_wxyz: np.ndarray, alpha: float) -> np.ndarray:
        """Blend ``alpha`` of the new quaternion into the previous one.

        ``alpha=1.0`` returns the raw input (no smoothing); ``alpha=0.0``
        returns the previous estimate (frozen).
        """
        assert 0.0 <= alpha <= 1.0, "alpha must be in [0, 1]"
        q_raw = np.asarray(q_wxyz, dtype=float)
        if self._q_prev is None or alpha >= 1.0:
            self._q_prev = q_raw.copy()
            return q_raw

        # Resolve double-cover: pick the hemisphere closer to q_prev.
        if float(np.dot(self._q_prev, q_raw)) < 0.0:
            q_raw = -q_raw

        # Slerp via scipy: build a 2-key rotation series, interpolate at alpha.
        rot_pair = R.from_quat(np.stack([self._to_xyzw(self._q_prev), self._to_xyzw(q_raw)]))
        from scipy.spatial.transform import Slerp

        slerp = Slerp([0.0, 1.0], rot_pair)
        q_blend_xyzw = slerp([alpha]).as_quat()[0]
        q_blend = self._to_wxyz(q_blend_xyzw)

        self._q_prev = q_blend
        return q_blend


class JointRateLimiter:
    """Per-joint EMA + max-velocity clamp.

    On each ``step(target)`` call:
        1. EMA: ``y = alpha * target + (1 - alpha) * y_prev`` (``alpha=1.0``
           disables EMA).
        2. Clamp: ``y_prev + clip(y - y_prev, -max_delta, +max_delta)`` where
           ``max_delta = max_velocity * dt``. ``max_velocity <= 0`` disables
           the clamp.
    """

    def __init__(
        self,
        dim: int,
        alpha: float = 1.0,
        max_velocity: float = 0.0,
        dt: float = 1.0 / 30.0,
    ) -> None:
        assert dim > 0, "dim must be positive"
        assert 0.0 <= alpha <= 1.0, "alpha must be in [0, 1]"
        assert dt > 0.0, "dt must be positive"
        self.dim = int(dim)
        self.alpha = float(alpha)
        self.max_velocity = float(max_velocity)
        self.dt = float(dt)
        self._prev: Optional[np.ndarray] = None

    def reset(self, v0: np.ndarray) -> None:
        v0 = np.asarray(v0, dtype=float).reshape(-1)
        assert v0.shape[0] == self.dim, f"reset vector dim {v0.shape[0]} != {self.dim}"
        self._prev = v0.copy()

    def step(self, target: np.ndarray) -> np.ndarray:
        target = np.asarray(target, dtype=float).reshape(-1)
        assert target.shape[0] == self.dim, f"target dim {target.shape[0]} != {self.dim}"
        if self._prev is None:
            self._prev = target.copy()
            return target.copy()

        y = self.alpha * target + (1.0 - self.alpha) * self._prev

        if self.max_velocity > 0.0:
            max_delta = self.max_velocity * self.dt
            y = self._prev + np.clip(y - self._prev, -max_delta, max_delta)

        self._prev = y
        return y.copy()
