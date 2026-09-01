import numpy as np
import pytest
from hesim3d.trajectory import Trajectory, get_trajectory_preset, ImuNoiseConfig, ImuSynthesizer


def test_eight_loop_spline():
    traj = Trajectory.create_eight_loop(
        center=(0.0, 0.0, 1.5),
        extent=(2.5, 1.5, 0.4),
        duration_sec=5.0,
        speed_factor=1.0,
    )
    assert traj.min_time == 0.0
    assert traj.max_time == 5.0

    # Sample at multiple points
    sample0 = traj.evaluate(0.0)
    sample1 = traj.evaluate(1.25)
    sample2 = traj.evaluate(2.5)

    assert sample0.position.shape == (3,)
    assert sample0.orientation_xyzw.shape == (4,)
    # Quaternion norm must be 1.0
    assert np.isclose(np.linalg.norm(sample0.orientation_xyzw), 1.0, atol=1e-5)
    assert np.isclose(np.linalg.norm(sample1.orientation_xyzw), 1.0, atol=1e-5)

    # Check analytical IMU properties
    assert sample1.angular_velocity_body.shape == (3,)
    assert sample1.imu_acceleration.shape == (3,)


def test_circle_orbit_spline():
    traj = get_trajectory_preset("circle_orbit", duration_sec=4.0, radius=3.0)
    sample = traj.evaluate(2.0)
    assert sample.position.shape == (3,)
    assert np.isclose(np.linalg.norm(sample.orientation_xyzw), 1.0, atol=1e-5)


def test_random_walk_spline():
    traj = get_trajectory_preset("random_walk", duration_sec=6.0, seed=123)
    sample = traj.evaluate(3.0)
    assert sample.position.shape == (3,)


def test_imu_synthesizer():
    traj = get_trajectory_preset("eight_loop")
    synthesizer = ImuSynthesizer(ImuNoiseConfig(), seed=42)

    sample = traj.evaluate(0.5)
    gyro, accel = synthesizer.process_sample(sample, 0.5)

    assert gyro.shape == (3,)
    assert accel.shape == (3,)
    # Measurements should be close to analytical kinematics with small noise
    assert np.all(np.abs(gyro - sample.angular_velocity_body) < 1.0)
