import tempfile
from pathlib import Path
import json
import numpy as np
import pytest
from hesim3d.trajectory.spline import Trajectory
from hesim3d.trajectory.presets import get_trajectory_preset
from hesim3d.simulator import Simulator
from hesim3d.sensor.config import SensorConfig


def test_trajectory_from_keyframes():
    keyframes = [
        {"time_sec": 0.0, "position": [0.0, 1.0, 0.0], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        {"time_sec": 2.0, "position": [1.0, 2.0, 1.0], "rotation_euler_deg": [15.0, -5.0, 0.0]},
        {"time_sec": 4.0, "position": [2.0, 1.0, 0.0], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
    ]
    traj = Trajectory.from_keyframes(keyframes, total_duration_sec=4.0)
    assert traj.max_time >= 3.9

    sample_0 = traj.evaluate(0.0)
    assert np.allclose(sample_0.position, [0.0, 1.0, 0.0], atol=0.2)

    sample_mid = traj.evaluate(2.0)
    assert np.allclose(sample_mid.position, [1.0, 2.0, 1.0], atol=0.4)


def test_trajectory_json_export_import():
    with tempfile.TemporaryDirectory() as tmpdir:
        json_path = Path(tmpdir) / "test_traj.json"

        # Create base eight loop
        base_traj = Trajectory.create_eight_loop(duration_sec=3.0)
        base_traj.to_json(json_path, num_keyframe_samples=6)

        assert json_path.exists()

        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
            assert data["version"] == "1.0"
            assert data["format"] == "hesim3d_earth_studio_trajectory"
            assert len(data["keyframes"]) == 6

        # Reload trajectory from JSON
        loaded_traj = Trajectory.from_json(json_path)
        assert loaded_traj.max_time > 0.0

        # Verify via get_trajectory_preset
        preset_traj = get_trajectory_preset(str(json_path))
        assert preset_traj.max_time > 0.0


def test_simulation_with_json_trajectory():
    with tempfile.TemporaryDirectory() as tmpdir:
        json_path = Path(tmpdir) / "studio_traj.json"
        out_h5 = Path(tmpdir) / "studio_sim.h5"

        keyframes = [
            {"time_sec": 0.0, "position": [0.0, 0.0, 1.5], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
            {"time_sec": 0.1, "position": [0.5, 0.2, 1.6], "orientation_xyzw": [0.0, 0.1, 0.0, 0.99]},
            {"time_sec": 0.2, "position": [0.0, 0.0, 1.5], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        ]
        traj = Trajectory.from_keyframes(keyframes, total_duration_sec=0.2)
        traj.to_json(json_path)

        cfg = SensorConfig(name="test_sensor", width=320, height=240, aps_fps=10.0, evs_fps=200.0)
        sim = Simulator(scene="checkerboard_room", sensor_config=cfg, trajectory=traj, backend_type="opengl")

        results = sim.run(duration_sec=0.2, sim_fps=200.0, output_path=out_h5, show_progress=False)
        assert results["duration_sec"] == 0.2
        assert out_h5.exists()
