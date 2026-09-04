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


def test_export_baked_simulation():
    from hesim3d.io.hdf5_writer import export_baked_simulation

    with tempfile.TemporaryDirectory() as tmpdir:
        out_h5 = Path(tmpdir) / "test_gui_export.h5"

        num_frames = 2
        w, h = 64, 48
        frame_bytes = (np.random.randint(0, 256, (num_frames, h, w, 3), dtype=np.uint8)).tobytes()
        frame_ts = [0, 33333]

        ev_t = [1000, 2000, 3000]
        ev_x = [10, 20, 30]
        ev_y = [5, 15, 25]
        ev_p = [1, -1, 1]

        imu_ts = [0, 10000, 20000]
        imu_gyro = [0.1, 0.2, 0.3, 0.1, 0.2, 0.3, 0.1, 0.2, 0.3]
        imu_acc = [0.0, 0.0, 9.81, 0.0, 0.0, 9.81, 0.0, 0.0, 9.81]

        gt_ts = [0, 10000, 20000]
        gt_pos = [0.0, 1.0, 2.0, 0.1, 1.1, 2.1, 0.2, 1.2, 2.2]
        gt_quat = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        ok = export_baked_simulation(
            output_path=str(out_h5),
            sensor_name="alpsentek_eiger",
            width=w,
            height=h,
            num_frames=num_frames,
            event_t=ev_t,
            event_x=ev_x,
            event_y=ev_y,
            event_p=ev_p,
            frame_bytes=frame_bytes,
            frame_timestamps_us=frame_ts,
            imu_timestamps_us=imu_ts,
            imu_gyro_flat=imu_gyro,
            imu_acc_flat=imu_acc,
            gt_timestamps_us=gt_ts,
            gt_pos_flat=gt_pos,
            gt_quat_flat=gt_quat,
        )

        assert ok is True
        assert out_h5.exists()

        with h5py.File(out_h5, "r") as f:
            assert f.attrs["format"] == "HESIM3D_HDF5"
            assert "events/t" in f
            assert len(f["events/t"]) == 3
            assert "frames/images" in f
            assert f["frames/images"].shape == (2, h, w, 3)
            assert len(f["imu/timestamps_us"]) == 3
            assert f["imu/angular_velocity"].shape == (3, 3)
            assert len(f["ground_truth/timestamps_us"]) == 3
            assert f["ground_truth/position"].shape == (3, 3)

