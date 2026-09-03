from __future__ import annotations

import os
from pathlib import Path
from typing import Optional, Union
import numpy as np

from hesim3d._hesim3d_core import GuiConfig as _CppGuiConfig, launch_gui as _cpp_launch_gui
from hesim3d.assets.downloader import resolve_scene_path
from hesim3d.trajectory.presets import get_trajectory_preset
from hesim3d.sensor.config import SensorConfig


def launch_interactive_gui(
    scene: str = "chessboard",
    sensor_name: str = "alpsentek_eiger",
    trajectory: str = "",
    duration_sec: float = 5.0,
    sim_fps: float = 1000.0,
    event_threshold: float = 0.20,
    window_width: int = 1600,
    window_height: int = 1000,
) -> int:
    """
    Launch the high-performance Dear ImGui interactive visualizer.
    """
    # 1. Resolve 3D scene path
    scene_path = resolve_scene_path(scene)

    # 2. Configure C++ GUI parameters
    cfg = _CppGuiConfig()
    cfg.scene_path = str(scene_path)
    cfg.sensor_name = str(sensor_name)
    cfg.trajectory_path = str(trajectory) if trajectory else ""
    cfg.duration_sec = float(duration_sec)
    cfg.sim_fps = float(sim_fps)
    cfg.event_threshold = float(event_threshold)
    cfg.window_width = int(window_width)
    cfg.window_height = int(window_height)

    # 3. Launch native C++ GLFW + ImGui loop
    return _cpp_launch_gui(cfg)
