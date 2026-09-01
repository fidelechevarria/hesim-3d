from __future__ import annotations

from typing import List, Tuple, Optional, Union
import numpy as np
from scipy.spatial.transform import Rotation
from hesim3d._hesim3d_core import SE3Spline as _CppSE3Spline, TrajectorySample


class Trajectory:
    """High-level Python wrapper around the C++ SE(3) Cumulative Cubic B-Spline."""

    def __init__(self, cpp_spline: _CppSE3Spline):
        self._spline = cpp_spline

    @classmethod
    def from_control_poses(
        cls,
        timestamps: Union[np.ndarray, List[float]],
        positions: Union[np.ndarray, List[List[float]]],
        orientations_xyzw: Union[np.ndarray, List[List[float]]],
        dt_ctrl: float = 0.05,
    ) -> Trajectory:
        """Create an SE(3) spline from discrete timestamped control poses."""
        ts = np.ascontiguousarray(timestamps, dtype=np.float64)
        pos = np.ascontiguousarray(positions, dtype=np.float64)
        ori = np.ascontiguousarray(orientations_xyzw, dtype=np.float64)
        cpp_spline = _CppSE3Spline(dt_ctrl)
        total_duration = float(ts[-1] - ts[0]) if len(ts) > 1 else 1.0
        cpp_spline.build_from_waypoints(pos, ori, total_duration)
        return cls(cpp_spline)

    @classmethod
    def create_eight_loop(
        cls,
        center: Union[np.ndarray, Tuple[float, float, float]] = (0.0, 0.0, 1.5),
        extent: Union[np.ndarray, Tuple[float, float, float]] = (2.5, 1.5, 0.4),
        duration_sec: float = 5.0,
        speed_factor: float = 1.0,
        dt_ctrl: float = 0.05,
    ) -> Trajectory:
        """Create a smooth 3D figure-eight loop trajectory."""
        c = np.ascontiguousarray(center, dtype=np.float64)
        e = np.ascontiguousarray(extent, dtype=np.float64)
        cpp_spline = _CppSE3Spline.create_eight_loop(c, e, float(duration_sec), float(speed_factor), float(dt_ctrl))
        return cls(cpp_spline)

    @classmethod
    def create_circle_orbit(
        cls,
        center: Union[np.ndarray, Tuple[float, float, float]] = (0.0, 0.0, 1.5),
        radius: float = 2.0,
        height: float = 1.5,
        duration_sec: float = 5.0,
        dt_ctrl: float = 0.05,
        speed_factor: float = 1.0,
    ) -> Trajectory:
        """Create a smooth circular orbit looking at center."""
        c = np.ascontiguousarray(center, dtype=np.float64)
        cpp_spline = _CppSE3Spline.create_circle_lookat(c, float(radius), float(height), float(duration_sec), float(dt_ctrl))
        return cls(cpp_spline)

    @classmethod
    def create_random_walk(
        cls,
        bounds_min: Union[np.ndarray, Tuple[float, float, float]] = (-2.0, -2.0, 0.5),
        bounds_max: Union[np.ndarray, Tuple[float, float, float]] = (2.0, 2.0, 2.5),
        num_waypoints: int = 10,
        duration_sec: float = 10.0,
        seed: int = 42,
        dt_ctrl: float = 0.05,
    ) -> Trajectory:
        """Create a continuous random-walk spline with smooth look-ahead orientation."""
        rng = np.random.default_rng(seed)
        b_min = np.array(bounds_min, dtype=np.float64)
        b_max = np.array(bounds_max, dtype=np.float64)
        positions = [b_min + rng.uniform(0.0, 1.0, 3) * (b_max - b_min) for _ in range(num_waypoints)]
        orientations = []

        for i in range(num_waypoints):
            next_pos = positions[(i + 1) % num_waypoints]
            forward = next_pos - positions[i]
            norm = np.linalg.norm(forward)
            if norm > 1e-4:
                forward = forward / norm
            else:
                forward = np.array([1.0, 0.0, 0.0])

            up = np.array([0.0, 0.0, 1.0])
            right = np.cross(forward, up)
            r_norm = np.linalg.norm(right)
            if r_norm > 1e-4:
                right = right / r_norm
            else:
                right = np.array([0.0, 1.0, 0.0])
            up = np.cross(right, forward)
            R = np.column_stack([right, up, -forward])
            q = Rotation.from_matrix(R).as_quat() # xyzw
            orientations.append(q)

        cpp_spline = _CppSE3Spline(dt_ctrl)
        cpp_spline.build_from_waypoints(positions, orientations, duration_sec)
        return cls(cpp_spline)

    @property
    def min_time(self) -> float:
        return self._spline.min_time()

    @property
    def max_time(self) -> float:
        return self._spline.max_time()

    def evaluate(self, time_sec: float) -> TrajectorySample:
        """Sample the continuous trajectory and get position, orientation, and analytical IMU data."""
        return self._spline.evaluate(float(time_sec))

    def evaluate_batch(self, timestamps_sec: Union[np.ndarray, List[float]]) -> List[TrajectorySample]:
        """Sample multiple timestamps along the trajectory."""
        return [self.evaluate(t) for t in timestamps_sec]

    @property
    def cpp_spline(self) -> _CppSE3Spline:
        return self._spline
