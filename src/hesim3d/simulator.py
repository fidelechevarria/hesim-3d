from __future__ import annotations

import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union
import numpy as np
import torch
from tqdm import tqdm

from hesim3d.trajectory.spline import Trajectory
from hesim3d.trajectory.presets import get_trajectory_preset
from hesim3d.trajectory.imu import ImuNoiseConfig, ImuSynthesizer
from hesim3d.sensor.config import SensorConfig
from hesim3d.sensor.isp import SensorISP, forward_srgb_oetf
from hesim3d.sensor.hesim_backend import HESIMEventSimulator
from hesim3d.assets.downloader import resolve_scene_path
from hesim3d.io.hdf5_writer import HDF5DatasetWriter
from hesim3d.io.mcap_writer import MCAPDatasetWriter
from hesim3d._hesim3d_core import FilamentRenderer, CameraIntrinsics


class Simulator:
    """
    End-to-End Hybrid Event-Frame Simulator (H-ESIM 3D).
    Orchestrates continuous kinematics, Google Filament PBR rendering,
    CFA sensor ISP, Poisson-Gaussian noise, H-ESIM neuromorphic event generation,
    and unified multi-format dataset export.
    """

    def __init__(
        self,
        scene: str = "checkerboard_room",
        sensor_config: Optional[SensorConfig] = None,
        trajectory: Optional[Trajectory] = None,
        imu_noise: Optional[ImuNoiseConfig] = None,
        backend_type: str = "vulkan",
        device: Optional[str] = None,
    ):
        self.sensor_config = sensor_config or SensorConfig()
        self.trajectory = trajectory or get_trajectory_preset("eight_loop")
        self.imu_synthesizer = ImuSynthesizer(imu_noise)

        # 1. Resolve and load 3D scene
        self.scene_path = resolve_scene_path(scene)

        # 2. Initialize Google Filament Offscreen PBR Engine
        self.renderer = FilamentRenderer(
            self.sensor_config.width,
            self.sensor_config.height,
            backend_type,
        )
        intrinsics = CameraIntrinsics()
        intrinsics.width = self.sensor_config.width
        intrinsics.height = self.sensor_config.height
        intrinsics.fx = (self.sensor_config.width / 2.0) / np.tan(np.radians(self.sensor_config.fov_deg / 2.0))
        intrinsics.fy = intrinsics.fx
        intrinsics.cx = self.sensor_config.width / 2.0
        intrinsics.cy = self.sensor_config.height / 2.0
        self.renderer.set_intrinsics(intrinsics)

        if str(self.scene_path).endswith(".glb") or str(self.scene_path).endswith(".gltf"):
            self.renderer.load_scene(str(self.scene_path))
        self.renderer.setup_default_lighting()

        # 3. Initialize Sensor ISP and H-ESIM event generation backend
        self.isp = SensorISP(self.sensor_config)
        self.event_backend = HESIMEventSimulator(self.sensor_config, device=device)

    def run(
        self,
        duration_sec: float = 5.0,
        sim_fps: float = 1000.0,
        output_path: Optional[Union[str, Path]] = None,
        show_progress: bool = True,
    ) -> Dict[str, Any]:
        """
        Execute high-speed simulation loop across time duration.
        """
        total_steps = int(duration_sec * sim_fps)
        dt_sec = 1.0 / sim_fps
        dt_us = int(dt_sec * 1e6)

        # Setup dataset exporter if output path provided
        writer = None
        if output_path is not None:
            out_p = Path(output_path)
            if out_p.suffix == ".mcap":
                writer = MCAPDatasetWriter(out_p, self.sensor_config)
            else:
                writer = HDF5DatasetWriter(out_p, self.sensor_config)

        # APS frame interval
        aps_interval_steps = max(1, int(sim_fps / self.sensor_config.aps_fps)) if self.sensor_config.aps_fps > 0 else 0

        # Memory buffer for Filament render readback
        rgb_frame = np.empty((self.sensor_config.height, self.sensor_config.width, 3), dtype=np.uint8)

        # State tracking
        last_raw_tensor: Optional[torch.Tensor] = None
        total_events_generated = 0
        total_frames_generated = 0
        event_chunks = []

        self.event_backend.reset_state(self.sensor_config.height, self.sensor_config.width)

        with tqdm(total=total_steps, desc="Simulating H-ESIM 3D", disable=not show_progress) as bar:
            for step in range(total_steps):
                t_sec = step * dt_sec
                t_us = int(t_sec * 1e6)

                # 1. Sample continuous trajectory & Kinematics
                sample = self.trajectory.evaluate(t_sec)

                # 2. Render PBR view via Google Filament
                self.renderer.set_camera_pose(sample.position, sample.orientation_xyzw)
                self.renderer.render_frame(rgb_frame, t_us)

                # 3. Transform via Sensor ISP to raw mosaic
                raw_mosaic = self.isp.srgb_to_raw(rgb_frame, add_noise=True)
                raw_tensor = torch.from_numpy(raw_mosaic)

                # 4. Generate neuromorphic events via H-ESIM
                if last_raw_tensor is not None:
                    t_prev_us = t_us - dt_us
                    ev_frame, ev_array = self.event_backend.simulate_interval(
                        last_raw_tensor, raw_tensor, t_prev_us, t_us
                    )
                    if len(ev_array) > 0:
                        total_events_generated += len(ev_array)
                        if writer is not None:
                            writer.write_events(ev_array)
                        else:
                            event_chunks.append(ev_array)

                last_raw_tensor = raw_tensor

                # 5. Synthesize IMU telemetry
                meas_gyro, meas_accel = self.imu_synthesizer.process_sample(sample, t_sec)
                if writer is not None:
                    writer.write_imu(t_us, meas_gyro, meas_accel)
                    writer.write_ground_truth_pose(t_us, sample.position, sample.orientation_xyzw)

                # 6. Save APS exposure frame if interval reached
                if aps_interval_steps > 0 and (step % aps_interval_steps == 0):
                    total_frames_generated += 1
                    if writer is not None:
                        writer.write_frame(rgb_frame, t_us)

                bar.update(1)

        if writer is not None:
            writer.close()

        all_events = np.concatenate(event_chunks, axis=0) if event_chunks else np.empty(0)

        return {
            "total_events": total_events_generated,
            "total_frames": total_frames_generated,
            "duration_sec": duration_sec,
            "sim_fps": sim_fps,
            "output_path": str(output_path) if output_path else None,
            "events": all_events,
        }
