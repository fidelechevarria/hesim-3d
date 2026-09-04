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

        # Storage buffers
        self._event_chunks: List[np.ndarray] = []
        self._frames: List[np.ndarray] = []
        self._frame_ts: List[int] = []
        self._imu_ts: List[int] = []
        self._imu_gyro: List[np.ndarray] = []
        self._imu_accel: List[np.ndarray] = []
        self._gt_ts: List[int] = []
        self._gt_pos: List[np.ndarray] = []
        self._gt_quat: List[np.ndarray] = []

    def write_events(self, events: np.ndarray) -> None:
        """Append a structured batch of events (t, x, y, p)."""
        if len(events) > 0:
            self._event_chunks.append(events)

    def write_frame(self, image: np.ndarray, timestamp_us: int) -> None:
        """Write an APS exposure frame."""
        self._frames.append(image)
        self._frame_ts.append(int(timestamp_us))

    def write_imu(self, timestamp_us: int, angular_velocity: np.ndarray, linear_acceleration: np.ndarray) -> None:
        """Record an IMU telemetry sample."""
        self._imu_ts.append(int(timestamp_us))
        self._imu_gyro.append(np.asarray(angular_velocity, dtype=np.float64))
        self._imu_accel.append(np.asarray(linear_acceleration, dtype=np.float64))

    def write_ground_truth_pose(self, timestamp_us: int, position: np.ndarray, orientation_xyzw: np.ndarray) -> None:
        """Record ground truth 6-DoF camera pose."""
        self._gt_ts.append(int(timestamp_us))
        self._gt_pos.append(np.asarray(position, dtype=np.float64))
        self._gt_quat.append(np.asarray(orientation_xyzw, dtype=np.float64))

    def close(self) -> None:
        """Consolidate and write all chunked datasets to HDF5."""
        # 1. Consolidate events
        if self._event_chunks:
            all_events = np.concatenate(self._event_chunks, axis=0)
            self._g_events.create_dataset("t", data=all_events["t"], compression="gzip", compression_opts=4)
            self._g_events.create_dataset("x", data=all_events["x"], compression="gzip", compression_opts=4)
            self._g_events.create_dataset("y", data=all_events["y"], compression="gzip", compression_opts=4)
            self._g_events.create_dataset("p", data=all_events["p"], compression="gzip", compression_opts=4)
            self._g_events.attrs["num_events"] = len(all_events)
        else:
            self._g_events.create_dataset("t", shape=(0,), dtype=np.uint64)
            self._g_events.create_dataset("x", shape=(0,), dtype=np.uint16)
            self._g_events.create_dataset("y", shape=(0,), dtype=np.uint16)
            self._g_events.create_dataset("p", shape=(0,), dtype=np.int8)
            self._g_events.attrs["num_events"] = 0

        # 2. Consolidate frames
        if self._frames:
            frames_arr = np.stack(self._frames, axis=0)
            self._g_frames.create_dataset("images", data=frames_arr, compression="gzip", compression_opts=4)
            self._g_frames.create_dataset("timestamps_us", data=np.array(self._frame_ts, dtype=np.uint64))
        else:
            self._g_frames.create_dataset("timestamps_us", shape=(0,), dtype=np.uint64)

        # 3. Consolidate IMU
        if self._imu_ts:
            self._g_imu.create_dataset("timestamps_us", data=np.array(self._imu_ts, dtype=np.uint64))
            self._g_imu.create_dataset("angular_velocity", data=np.stack(self._imu_gyro, axis=0))
            self._g_imu.create_dataset("linear_acceleration", data=np.stack(self._imu_accel, axis=0))

        # 4. Consolidate Ground Truth
        if self._gt_ts:
            self._g_gt.create_dataset("timestamps_us", data=np.array(self._gt_ts, dtype=np.uint64))
            self._g_gt.create_dataset("position", data=np.stack(self._gt_pos, axis=0))
            self._g_gt.create_dataset("orientation_xyzw", data=np.stack(self._gt_quat, axis=0))

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
        # 1. Events
        if len(event_t) > 0:
            dtype = [("t", np.uint64), ("x", np.uint16), ("y", np.uint16), ("p", np.int8)]
            ev_arr = np.empty(len(event_t), dtype=dtype)
            ev_arr["t"] = np.asarray(event_t, dtype=np.uint64)
            ev_arr["x"] = np.asarray(event_x, dtype=np.uint16)
            ev_arr["y"] = np.asarray(event_y, dtype=np.uint16)
            ev_arr["p"] = np.asarray(event_p, dtype=np.int8)
            writer.write_events(ev_arr)

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
