from __future__ import annotations

"""
hesim-3d: High-Performance Photorealistic 3D Neuromorphic Event-Frame Sensor Simulator.
Coupling Google Filament PBR rendering with the ECCV 2026 H-ESIM noise model.
"""

from hesim3d.trajectory import Trajectory, get_trajectory_preset, ImuNoiseConfig, ImuSynthesizer
from hesim3d.sensor import (
    SensorConfig,
    SensorISP,
    HESIMEventSimulator,
    accumulate_events_to_image,
)
from hesim3d.assets import (
    SCENE_CATALOG,
    list_available_scenes,
    download_scene,
    resolve_scene_path,
)
from hesim3d.io import HDF5DatasetWriter, MCAPDatasetWriter
from hesim3d.simulator import Simulator
from hesim3d.gui import launch_interactive_gui

__version__ = "0.1.0"

__all__ = [
    "Trajectory",
    "get_trajectory_preset",
    "ImuNoiseConfig",
    "ImuSynthesizer",
    "SensorConfig",
    "SensorISP",
    "HESIMEventSimulator",
    "accumulate_events_to_image",
    "SCENE_CATALOG",
    "list_available_scenes",
    "download_scene",
    "resolve_scene_path",
    "HDF5DatasetWriter",
    "MCAPDatasetWriter",
    "Simulator",
    "launch_interactive_gui",
]
