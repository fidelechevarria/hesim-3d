from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union
import h5py
import numpy as np
from hesim3d.sensor.config import SensorConfig


class HDF5DatasetWriter:
    """
    Standardized HDF5 dataset exporter for event-based vision and neuromorphic ML benchmarks.
    """

    def __init__(self, output_path: Union[str, Path], sensor_config: Optional[SensorConfig] = None):
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.sensor_config = sensor_config or SensorConfig()

        self._h5 = h5py.File(self.output_path, "w")

        # Create primary groups
        self._g_events = self._h5.create_group("events")
        self._g_frames = self._h5.create_group("frames")
        self._g_imu = self._h5.create_group("imu")
        self._g_gt = self._h5.create_group("ground_truth")

        # Save metadata attributes
        self._h5.attrs["format"] = "HESIM3D_HDF5"
        self._h5.attrs["version"] = "1.0.0"
        self._h5.attrs["sensor_name"] = self.sensor_config.name
        self._h5.attrs["sensor_config_json"] = json.dumps(self.sensor_config.to_dict())

        # Extensible chunked datasets
        chunk_ev = 131072
        self._dset_t = self._g_events.create_dataset(
            "t", shape=(0,), maxshape=(None,), dtype=np.uint64, chunks=(chunk_ev,), compression="gzip", compression_opts=4
        )
        self._dset_x = self._g_events.create_dataset(
            "x", shape=(0,), maxshape=(None,), dtype=np.uint16, chunks=(chunk_ev,), compression="gzip", compression_opts=4
        )
        self._dset_y = self._g_events.create_dataset(
            "y", shape=(0,), maxshape=(None,), dtype=np.uint16, chunks=(chunk_ev,), compression="gzip", compression_opts=4
        )
        self._dset_p = self._g_events.create_dataset(
            "p", shape=(0,), maxshape=(None,), dtype=np.int8, chunks=(chunk_ev,), compression="gzip", compression_opts=4
        )
        self._num_events = 0
        self._g_events.attrs["num_events"] = 0

        self._dset_frames = None
        self._dset_frame_ts = self._g_frames.create_dataset(
            "timestamps_us", shape=(0,), maxshape=(None,), dtype=np.uint64, chunks=True
        )
        self._num_frames = 0

        self._dset_imu_ts = self._g_imu.create_dataset("timestamps_us", shape=(0,), maxshape=(None,), dtype=np.uint64, chunks=True)
        self._dset_imu_gyro = self._g_imu.create_dataset("angular_velocity", shape=(0, 3), maxshape=(None, 3), dtype=np.float64, chunks=True)
        self._dset_imu_accel = self._g_imu.create_dataset("linear_acceleration", shape=(0, 3), maxshape=(None, 3), dtype=np.float64, chunks=True)
        self._num_imu = 0

        self._dset_gt_ts = self._g_gt.create_dataset("timestamps_us", shape=(0,), maxshape=(None,), dtype=np.uint64, chunks=True)
        self._dset_gt_pos = self._g_gt.create_dataset("position", shape=(0, 3), maxshape=(None, 3), dtype=np.float64, chunks=True)
        self._dset_gt_quat = self._g_gt.create_dataset("orientation_xyzw", shape=(0, 4), maxshape=(None, 4), dtype=np.float64, chunks=True)
        self._num_gt = 0

    def write_events_raw(
        self,
        t: Union[np.ndarray, List[int]],
        x: Union[np.ndarray, List[int]],
        y: Union[np.ndarray, List[int]],
        p: Union[np.ndarray, List[int]],
    ) -> None:
        """Directly append raw event component arrays to the extensible HDF5 dataset."""
        n = len(t)
        if n == 0:
            return
        curr = self._num_events
        new_size = curr + n
        self._dset_t.resize((new_size,))
        self._dset_x.resize((new_size,))
        self._dset_y.resize((new_size,))
        self._dset_p.resize((new_size,))

        self._dset_t[curr:new_size] = np.asarray(t, dtype=np.uint64)
        self._dset_x[curr:new_size] = np.asarray(x, dtype=np.uint16)
        self._dset_y[curr:new_size] = np.asarray(y, dtype=np.uint16)
        self._dset_p[curr:new_size] = np.asarray(p, dtype=np.int8)

        self._num_events = new_size
        self._g_events.attrs["num_events"] = new_size

    def write_events(self, events: np.ndarray) -> None:
        """Append a structured batch of events (t, x, y, p) directly to disk."""
        if len(events) == 0:
            return
        if events.dtype.names is not None:
            self.write_events_raw(events["t"], events["x"], events["y"], events["p"])
        else:
            self.write_events_raw(events[:, 0], events[:, 1], events[:, 2], events[:, 3])

    def write_frame(self, image: np.ndarray, timestamp_us: int) -> None:
        """Write an APS exposure frame directly to disk."""
        if self._dset_frames is None:
            H, W, C = image.shape
            self._dset_frames = self._g_frames.create_dataset(
                "images",
                shape=(0, H, W, C),
                maxshape=(None, H, W, C),
                dtype=image.dtype,
                chunks=(1, H, W, C),
                compression="gzip",
                compression_opts=4,
            )
        idx = self._num_frames
        self._dset_frames.resize((idx + 1, *image.shape))
        self._dset_frames[idx] = image
        self._dset_frame_ts.resize((idx + 1,))
        self._dset_frame_ts[idx] = int(timestamp_us)
        self._num_frames += 1

    def write_imu(self, timestamp_us: int, angular_velocity: np.ndarray, linear_acceleration: np.ndarray) -> None:
        """Record an IMU telemetry sample directly to disk."""
        idx = self._num_imu
        self._dset_imu_ts.resize((idx + 1,))
        self._dset_imu_ts[idx] = int(timestamp_us)
        self._dset_imu_gyro.resize((idx + 1, 3))
        self._dset_imu_gyro[idx] = np.asarray(angular_velocity, dtype=np.float64)
        self._dset_imu_accel.resize((idx + 1, 3))
        self._dset_imu_accel[idx] = np.asarray(linear_acceleration, dtype=np.float64)
        self._num_imu += 1

    def write_ground_truth_pose(self, timestamp_us: int, position: np.ndarray, orientation_xyzw: np.ndarray) -> None:
        """Record ground truth 6-DoF camera pose directly to disk."""
        idx = self._num_gt
        self._dset_gt_ts.resize((idx + 1,))
        self._dset_gt_ts[idx] = int(timestamp_us)
        self._dset_gt_pos.resize((idx + 1, 3))
        self._dset_gt_pos[idx] = np.asarray(position, dtype=np.float64)
        self._dset_gt_quat.resize((idx + 1, 4))
        self._dset_gt_quat[idx] = np.asarray(orientation_xyzw, dtype=np.float64)
        self._num_gt += 1

    def close(self) -> None:
        """Flush and close HDF5 file."""
        if self._dset_frames is None:
            H = getattr(self.sensor_config, "height", 480)
            W = getattr(self.sensor_config, "width", 640)
            self._g_frames.create_dataset(
                "images", shape=(0, H, W, 3), dtype=np.uint8, chunks=True
            )
        self._h5.flush()
        self._h5.close()

    def __enter__(self) -> HDF5DatasetWriter:
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()


def export_baked_simulation(
    output_path: str,
    sensor_name: str,
    width: int,
    height: int,
    num_frames: int,
    event_t: list,
    event_x: list,
    event_y: list,
    event_p: list,
    frame_bytes: bytes,
    frame_timestamps_us: list,
    imu_timestamps_us: list,
    imu_gyro_flat: list,
    imu_acc_flat: list,
    gt_timestamps_us: list,
    gt_pos_flat: list,
    gt_quat_flat: list,
) -> bool:
    """
    Export raw simulation arrays directly to a standardized HDF5 dataset.
    """
    out_p = Path(output_path)
    out_p.parent.mkdir(parents=True, exist_ok=True)
    try:
        cfg = SensorConfig.from_preset(sensor_name)
    except Exception:
        cfg = SensorConfig(name=sensor_name, width=width, height=height)
    cfg.width = int(width)
    cfg.height = int(height)

    with HDF5DatasetWriter(out_p, cfg) as writer:
        # 1. Events: stream directly in bounded chunks to prevent RAM spikes
        n_ev = len(event_t)
        if n_ev > 0:
            chunk_batch = 500_000
            for start_idx in range(0, n_ev, chunk_batch):
                end_idx = min(start_idx + chunk_batch, n_ev)
                writer.write_events_raw(
                    event_t[start_idx:end_idx],
                    event_x[start_idx:end_idx],
                    event_y[start_idx:end_idx],
                    event_p[start_idx:end_idx],
                )

        # 2. Frames
        if num_frames > 0 and len(frame_bytes) > 0:
            frames_arr = np.frombuffer(frame_bytes, dtype=np.uint8).reshape((num_frames, height, width, 3))
            for i in range(num_frames):
                writer.write_frame(frames_arr[i], int(frame_timestamps_us[i]))

        # 3. IMU
        if len(imu_timestamps_us) > 0:
            imu_g = np.asarray(imu_gyro_flat, dtype=np.float64).reshape((-1, 3))
            imu_a = np.asarray(imu_acc_flat, dtype=np.float64).reshape((-1, 3))
            for j in range(len(imu_timestamps_us)):
                writer.write_imu(int(imu_timestamps_us[j]), imu_g[j], imu_a[j])

        # 4. Ground Truth
        if len(gt_timestamps_us) > 0:
            gt_p = np.asarray(gt_pos_flat, dtype=np.float64).reshape((-1, 3))
            gt_q = np.asarray(gt_quat_flat, dtype=np.float64).reshape((-1, 4))
            for k in range(len(gt_timestamps_us)):
                writer.write_ground_truth_pose(int(gt_timestamps_us[k]), gt_p[k], gt_q[k])

    return True
