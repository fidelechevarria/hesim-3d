from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple, Union
from pathlib import Path
import numpy as np
import torch

from hesim3d.sensor.config import SensorConfig
from hesim3d.sensor.isp import SensorISP
from hesim3d.sensor.hesim_backend import HESIMEventSimulator


class ScientificBakeEngine:
    """
    High-Performance Scientific Neuromorphic Simulation Engine (H-ESIM 3D).
    Coordinates physical optics, Google Filament PBR rendering inputs,
    CFA sensor ISP with Poisson-Gaussian noise, PyTorch CUDA tensor acceleration,
    calibrated 6-parameter beta noise models, Gaussian Q-function stochastic event firing,
    microsecond pixel refractory filtering, and optical shutter motion blur integration.
    """

    def __init__(
        self,
        sensor_name: str = "alpsentek_eiger",
        width: Optional[int] = None,
        height: Optional[int] = None,
        event_threshold: Optional[float] = None,
        refractory_period_us: Optional[int] = None,
        device: Optional[str] = None,
    ):
        # 1. Resolve and configure sensor preset
        try:
            self.config = SensorConfig.from_preset(sensor_name)
        except Exception:
            self.config = SensorConfig()

        if width is not None and width > 0:
            self.config.width = int(width)
        if height is not None and height > 0:
            self.config.height = int(height)

        # Apply user contrast threshold (C) tuning
        if event_threshold is not None and event_threshold > 0:
            self.config.event_threshold = float(event_threshold)
            self.config.theta_scale = 1.0

        # Apply user refractory period (us) tuning
        if refractory_period_us is not None and refractory_period_us > 0:
            self.config.refractory_period_us = float(refractory_period_us)

        # 2. Select compute device (NVIDIA CUDA if available)
        if device is None:
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
        else:
            self.device = device

        # 3. Initialize Sensor ISP and PyTorch H-ESIM Neuromorphic Backend
        self.isp = SensorISP(self.config)
        self.event_backend = HESIMEventSimulator(self.config, device=self.device)
        self.event_backend.reset_state(self.config.height, self.config.width)

        # State tracking for continuous sub-intervals
        self.last_raw_tensor: Optional[torch.Tensor] = None
        self.last_sub_t_us: Optional[int] = None

        # Exposure accumulation for optical shutter motion blur
        self.accum_raw_frame: Optional[np.ndarray] = None
        self.accum_count: int = 0

    def get_device_info(self) -> Dict[str, str]:
        """Return diagnostic strings regarding hardware compute and noise model."""
        if self.device == "cuda" and torch.cuda.is_available():
            dev_name = torch.cuda.get_device_name(0)
            device_str = f"{dev_name} (CUDA)"
        else:
            device_str = "CPU (PyTorch Vectorized)"

        cfa_str = self.config.cfa_pattern.upper()
        noise_str = f"H-ESIM 6-Beta CFA ({cfa_str}) + Poisson-Gaussian + Dark Current"

        return {
            "device": device_str,
            "noise_model": noise_str,
            "cfa_pattern": self.config.cfa_pattern,
            "sensor_name": self.config.name,
            "resolution": f"{self.config.width}x{self.config.height}",
        }

    def reset(self) -> None:
        """Reset internal temporal states and accumulator maps."""
        self.event_backend.reset_state(self.config.height, self.config.width)
        self.last_raw_tensor = None
        self.last_sub_t_us = None
        self.accum_raw_frame = None
        self.accum_count = 0

    def process_aps_subsamples(
        self,
        sub_frames_bytes: bytes,
        sub_timestamps_us: List[int],
        shutter_duration_us: int,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, bytes]:
        """
        Process a batch of sub-frames across an APS frame interval.

        Args:
            sub_frames_bytes: Contiguous uint8 bytes of N sub-frames (N * H * W * 3)
            sub_timestamps_us: List of N microsecond timestamps
            shutter_duration_us: Optical shutter open duration in microseconds

        Returns:
            Tuple of (ev_t_us, ev_x, ev_y, ev_p, blurred_aps_frame_bytes)
        """
        n_samples = len(sub_timestamps_us)
        H = self.config.height
        W = self.config.width
        frame_bytes_len = H * W * 3

        if n_samples == 0 or len(sub_frames_bytes) < frame_bytes_len:
            return (
                np.empty(0, dtype=np.uint64),
                np.empty(0, dtype=np.uint16),
                np.empty(0, dtype=np.uint16),
                np.empty(0, dtype=np.int8),
                b"",
            )

        # Interpret contiguous byte stream as (N, H, W, 3) numpy array
        raw_buffer = np.frombuffer(sub_frames_bytes, dtype=np.uint8)
        frames_array = raw_buffer[: n_samples * frame_bytes_len].reshape((n_samples, H, W, 3))

        event_chunks = []
        frame_start_t_us = sub_timestamps_us[0]
        shutter_end_t_us = frame_start_t_us + max(1, shutter_duration_us)

        # Accumulator for optical exposure motion blur
        accum_raw = np.zeros((H, W), dtype=np.float32)
        accum_count = 0

        for s in range(n_samples):
            sub_frame_rgb = frames_array[s]
            t_us = int(sub_timestamps_us[s])

            # 1. Physical Sensor ISP: sRGB -> Linear Radiance -> Inverse CCM -> CFA Mosaic
            raw_mosaic = self.isp.srgb_to_raw(sub_frame_rgb, add_noise=False)

            # 2. Accumulate optical radiance during shutter open interval [frame_start, shutter_end]
            if t_us <= shutter_end_t_us or accum_count == 0:
                accum_raw += raw_mosaic
                accum_count += 1

            # 3. Transfer to PyTorch CUDA tensor
            raw_tensor = torch.from_numpy(raw_mosaic).to(self.device, dtype=torch.float32)

            # 4. Continuous Neuromorphic Event Simulation across interval [prev_t_us, t_us]
            if self.last_raw_tensor is not None and self.last_sub_t_us is not None:
                prev_t_us = self.last_sub_t_us
                if t_us > prev_t_us:
                    _, ev_array = self.event_backend.simulate_interval(
                        self.last_raw_tensor, raw_tensor, float(prev_t_us), float(t_us)
                    )
                    if len(ev_array) > 0:
                        event_chunks.append(ev_array)

            self.last_raw_tensor = raw_tensor
            self.last_sub_t_us = t_us

        # 5. Consolidate generated events for this APS frame
        if event_chunks:
            all_ev = np.concatenate(event_chunks, axis=0)
            ev_t = all_ev["t"]
            ev_x = all_ev["x"]
            ev_y = all_ev["y"]
            ev_p = all_ev["p"]
        else:
            ev_t = np.empty(0, dtype=np.uint64)
            ev_x = np.empty(0, dtype=np.uint16)
            ev_y = np.empty(0, dtype=np.uint16)
            ev_p = np.empty(0, dtype=np.int8)

        # 6. Synthesize blurred APS frame with real physical exposure integration & Poisson-Gaussian readout noise
        if accum_count > 0:
            mean_raw = accum_raw / accum_count
            var = np.maximum(1e-10, self.config.aps_noise_beta1 * mean_raw + self.config.aps_noise_beta2)
            noise = np.random.normal(0.0, 1.0, size=mean_raw.shape).astype(np.float32) * np.sqrt(var)
            noisy_raw = np.clip(mean_raw + noise, 0.0, 1.0)
            blurred_rgb = self.isp.raw_to_rgb_preview(noisy_raw)
        else:
            blurred_rgb = frames_array[0]

        blurred_bytes = blurred_rgb.tobytes()

        return ev_t, ev_x, ev_y, ev_p, blurred_bytes


# Global singleton instance for seamless nanobind C++ bridge invocation
_GLOBAL_ENGINE: Optional[ScientificBakeEngine] = None


def init_scientific_engine(
    sensor_name: str,
    width: int,
    height: int,
    event_threshold: float,
    refractory_period_us: int,
) -> Tuple[str, str]:
    """
    Initialize or reconfigure the singleton ScientificBakeEngine.
    Returns (device_info_string, model_info_string).
    """
    global _GLOBAL_ENGINE
    _GLOBAL_ENGINE = ScientificBakeEngine(
        sensor_name=sensor_name,
        width=width,
        height=height,
        event_threshold=event_threshold,
        refractory_period_us=refractory_period_us,
    )
    info = _GLOBAL_ENGINE.get_device_info()
    return info["device"], info["noise_model"]


def step_scientific_engine(
    sub_frames_bytes: bytes,
    sub_timestamps_us: List[int],
    shutter_duration_us: int,
) -> Tuple[List[int], List[int], List[int], List[int], bytes]:
    """
    Execute scientific simulation step for one APS frame interval.
    Returns (ev_t, ev_x, ev_y, ev_p, blurred_aps_frame_bytes).
    """
    global _GLOBAL_ENGINE
    if _GLOBAL_ENGINE is None:
        raise RuntimeError("ScientificBakeEngine not initialized. Call init_scientific_engine first.")

    ev_t, ev_x, ev_y, ev_p, blurred_bytes = _GLOBAL_ENGINE.process_aps_subsamples(
        sub_frames_bytes, sub_timestamps_us, shutter_duration_us
    )

    return (
        ev_t.tolist(),
        ev_x.tolist(),
        ev_y.tolist(),
        ev_p.tolist(),
        blurred_bytes,
    )


def reset_scientific_engine() -> None:
    """Reset the singleton engine state."""
    global _GLOBAL_ENGINE
    if _GLOBAL_ENGINE is not None:
        _GLOBAL_ENGINE.reset()
