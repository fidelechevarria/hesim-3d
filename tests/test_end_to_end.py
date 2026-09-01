import tempfile
from pathlib import Path
import h5py
import numpy as np
import pytest
from hesim3d.simulator import Simulator
from hesim3d.trajectory.presets import get_trajectory_preset
from hesim3d.sensor.config import SensorConfig


def test_headless_end_to_end_simulation():
    with tempfile.TemporaryDirectory() as tmpdir:
        out_h5 = Path(tmpdir) / "sim_output.h5"

        cfg = SensorConfig(
            name="test_sensor",
            width=320,
            height=240,
            aps_fps=10.0,
            evs_fps=200.0,
        )

        traj = get_trajectory_preset("eight_loop", duration_sec=0.2)

        sim = Simulator(
            scene="checkerboard_room",
            sensor_config=cfg,
            trajectory=traj,
            backend_type="opengl",
        )

        results = sim.run(
            duration_sec=0.2,
            sim_fps=200.0,
            output_path=out_h5,
            show_progress=False,
        )

        assert results["duration_sec"] == 0.2
        assert out_h5.exists()

        # Validate HDF5 format integrity
        with h5py.File(out_h5, "r") as f:
            assert "events/t" in f
            assert "frames/images" in f
            assert "imu/angular_velocity" in f
            assert "ground_truth/position" in f
            assert f.attrs["format"] == "HESIM3D_HDF5"
