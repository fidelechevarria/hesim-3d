from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from typing import List, Tuple, Optional
from hesim3d._hesim3d_core import TrajectorySample


@dataclass
class ImuNoiseConfig:
    """Noise parameters for synthetic 6-DoF inertial measurement units (IMU)."""
    gyro_noise_density: float = 0.0001        # rad/s/sqrt(Hz) (Continuous white noise)
    accel_noise_density: float = 0.001        # m/s^2/sqrt(Hz)
    gyro_random_walk: float = 0.00001         # rad/s^2/sqrt(Hz) (Bias instability)
    accel_random_walk: float = 0.0001         # m/s^3/sqrt(Hz)
    gyro_bias_init: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    accel_bias_init: Tuple[float, float, float] = (0.0, 0.0, 0.0)


class ImuSynthesizer:
    """Generates time-series IMU measurements with realistic stochastic noise and bias walk."""

    def __init__(self, config: Optional[ImuNoiseConfig] = None, seed: Optional[int] = None):
        self.config = config or ImuNoiseConfig()
        self.rng = np.random.default_rng(seed)

        self.gyro_bias = np.array(self.config.gyro_bias_init, dtype=np.float64)
        self.accel_bias = np.array(self.config.accel_bias_init, dtype=np.float64)
        self._last_time: Optional[float] = None

    def process_sample(self, sample: TrajectorySample, time_sec: float) -> Tuple[np.ndarray, np.ndarray]:
        """Apply noise and bias drift to an exact analytical kinematic sample."""
        if self._last_time is None:
            dt = 0.001
        else:
            dt = max(1e-6, time_sec - self._last_time)
        self._last_time = time_sec

        # Update bias random walk
        gyro_bias_step = self.config.gyro_random_walk * np.sqrt(dt) * self.rng.standard_normal(3)
        accel_bias_step = self.config.accel_random_walk * np.sqrt(dt) * self.rng.standard_normal(3)
        self.gyro_bias += gyro_bias_step
        self.accel_bias += accel_bias_step

        # Add Gaussian white measurement noise
        gyro_noise = (self.config.gyro_noise_density / np.sqrt(dt)) * self.rng.standard_normal(3)
        accel_noise = (self.config.accel_noise_density / np.sqrt(dt)) * self.rng.standard_normal(3)

        measured_gyro = sample.angular_velocity_body + self.gyro_bias + gyro_noise
        measured_accel = sample.imu_acceleration + self.accel_bias + accel_noise

        return measured_gyro, measured_accel
