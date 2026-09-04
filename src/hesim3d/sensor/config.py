from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple, Union
import numpy as np


@dataclass
class SensorConfig:
    """Complete sensor configuration for Hybrid APS + EVS systems."""
    name: str = "alpsentek_eiger"
    description: str = "AlpsenTek ALVium/Eiger 0.5MP Hybrid Quad-Bayer APS+EVS"

    # Resolution & Optical Parameters
    width: int = 640
    height: int = 480
    pixel_pitch_um: float = 4.86
    focal_length_mm: float = 6.0
    fov_deg: float = 65.0

    # APS Parameters
    aps_fps: float = 30.0
    exposure_time_ms: float = 10.0
    cfa_pattern: str = "quad_bayer" # "quad_bayer", "rggb", "mono"
    cfa_size: int = 2               # 2x2 or 4x4
    color_correction_matrix: List[List[float]] = field(default_factory=lambda: [
        [1.65, -0.45, -0.20],
        [-0.25, 1.40, -0.15],
        [-0.05, -0.35, 1.40]
    ])
    black_level: float = 0.01
    white_level: float = 1.0

    # APS Noise (Poisson-Gaussian / Polynomial variance)
    aps_noise_beta1: float = 0.0005 # Shot noise variance coefficient
    aps_noise_beta2: float = 0.0001 # Read noise variance coefficient

    # EVS Parameters (H-ESIM ECCV 2026 Formulations)
    evs_fps: float = 1000.0         # Intermediate simulation rate (Hz)
    theta_hw: float = 0.75e-3       # Hardware baseline threshold voltage (V)
    theta_scale: float = 1.0        # User sensitivity multiplier
    event_threshold: float = 0.20   # Log-intensity contrast threshold (C)
    refractory_period_us: float = 10.0 # Pixel refractory dead time (us)

    # 6-Parameter EVS Noise Model per CFA channel (betas: [b0, b1, b2, b3, b4, b5])
    # b0: Contrast scaling, b1: Conversion gain, b2: DC offset,
    # b3: Shot noise, b4: Read noise, b5: Cross-correlation
    evs_betas: List[List[List[float]]] = field(default_factory=lambda: [
        [[1.20, 1.00, 0.015, 0.08, 0.010, 0.0], [1.15, 1.00, 0.012, 0.07, 0.009, 0.0]],
        [[1.18, 1.00, 0.014, 0.075, 0.010, 0.0], [1.22, 1.00, 0.016, 0.085, 0.011, 0.0]]
    ])

    # Dark Noise Event Rate (Hz/pixel)
    dark_event_rate_pos: float = 0.05
    dark_event_rate_neg: float = 0.03

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> SensorConfig:
        return cls(**{k: v for k, v in data.items() if k in cls.__dataclass_fields__})

    def save_json(self, path: Union[str, Path]) -> None:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load_json(cls, path: Union[str, Path]) -> SensorConfig:
        with open(path, "r") as f:
            data = json.load(f)
        return cls.from_dict(data)

    @classmethod
    def from_preset(cls, name: str) -> SensorConfig:
        name_clean = name.strip().lower()
        preset_dir = Path(__file__).resolve().parent.parent.parent.parent / "assets" / "sensor_presets"

        target_file = preset_dir / f"{name_clean}.json"
        if target_file.exists():
            return cls.load_json(target_file)

        # Built-in fallbacks
        if "eiger" in name_clean or "alpsentek" in name_clean:
            return cls(name="alpsentek_eiger")
        elif "evk4" in name_clean or "prophesee" in name_clean:
            return cls(
                name="prophesee_evk4",
                description="Prophesee EVK4 HD Metavision Sensor (1280x720 Mono EVS)",
                width=1280,
                height=720,
                cfa_pattern="mono",
                cfa_size=1,
                evs_betas=[[[1.0, 1.0, 0.008, 0.05, 0.005, 0.0]]],
            )
        elif "davis" in name_clean or "davis346" in name_clean:
            return cls(
                name="davis346",
                description="iniVation DAVIS346 (346x260 APS+EVS)",
                width=346,
                height=260,
                cfa_pattern="mono",
                cfa_size=1,
                evs_betas=[[[1.5, 1.0, 0.02, 0.12, 0.015, 0.0]]],
            )
        elif "imx636" in name_clean or "sony" in name_clean:
            return cls(
                name="sony_imx636",
                description="Sony IMX636 HD Event-Based Vision Sensor (1280x720)",
                width=1280,
                height=720,
                cfa_pattern="mono",
                cfa_size=1,
                evs_betas=[[[1.0, 1.0, 0.005, 0.04, 0.004, 0.0]]],
            )
        else:
            raise ValueError(
                f"Unknown sensor preset: '{name}'. Available: 'alpsentek_eiger', 'prophesee_evk4', 'davis346', 'sony_imx636'"
            )
