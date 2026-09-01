import tempfile
from pathlib import Path
import h5py
import numpy as np
import pytest
from hesim3d.io import HDF5DatasetWriter, MCAPDatasetWriter
from hesim3d.sensor.config import SensorConfig


def test_hdf5_dataset_writer():
    with tempfile.TemporaryDirectory() as tmpdir:
        h5_path = Path(tmpdir) / "test_dataset.h5"
        cfg = SensorConfig.from_preset("alpsentek_eiger")

        with HDF5DatasetWriter(h5_path, cfg) as writer:
            # Write structured events
            dtype = [("t", np.uint64), ("x", np.uint16), ("y", np.uint16), ("p", np.int8)]
            events = np.array([(1000, 10, 20, 1), (1050, 15, 25, -1)], dtype=dtype)
            writer.write_events(events)

            # Write frame
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            writer.write_frame(img, 1000)

            # Write IMU
            writer.write_imu(1000, np.array([0.1, 0.2, 0.3]), np.array([0.0, 0.0, 9.81]))

            # Write GT
            writer.write_ground_truth_pose(1000, np.array([1.0, 2.0, 3.0]), np.array([0.0, 0.0, 0.0, 1.0]))

        assert h5_path.exists()

        # Verify dataset contents
        with h5py.File(h5_path, "r") as f:
            assert "events/t" in f
            assert len(f["events/t"]) == 2
            assert f["events/x"][0] == 10
            assert f["events/p"][1] == -1
            assert "frames/images" in f
            assert f["frames/images"].shape == (1, 480, 640, 3)
            assert "imu/angular_velocity" in f
            assert "ground_truth/position" in f


def test_mcap_dataset_writer():
    with tempfile.TemporaryDirectory() as tmpdir:
        mcap_path = Path(tmpdir) / "test_stream.mcap"
        cfg = SensorConfig.from_preset("alpsentek_eiger")

        with MCAPDatasetWriter(mcap_path, cfg) as writer:
            dtype = [("t", np.uint64), ("x", np.uint16), ("y", np.uint16), ("p", np.int8)]
            events = np.array([(1000, 10, 20, 1)], dtype=dtype)
            writer.write_events(events)

            img = np.zeros((480, 640, 3), dtype=np.uint8)
            writer.write_frame(img, 1000)
            writer.write_imu(1000, np.array([0.1, 0.2, 0.3]), np.array([0.0, 0.0, 9.81]))
            writer.write_ground_truth_pose(1000, np.array([1.0, 2.0, 3.0]), np.array([0.0, 0.0, 0.0, 1.0]))

        assert mcap_path.exists()
        assert mcap_path.stat().st_size > 0
