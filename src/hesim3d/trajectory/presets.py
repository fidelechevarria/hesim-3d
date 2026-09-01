from __future__ import annotations

from typing import Dict, Any, Optional
from .spline import Trajectory


def get_trajectory_preset(
    name: str,
    duration_sec: float = 5.0,
    speed_factor: float = 1.0,
    **kwargs: Any,
) -> Trajectory:
    """Generate predefined high-grade 6-DoF trajectory profiles."""
    name_lower = name.strip().lower()

    if name_lower in ("eight", "eight_loop", "figure8", "figure_eight"):
        center = kwargs.get("center", (0.0, 0.0, 1.5))
        extent = kwargs.get("extent", (2.5, 1.5, 0.4))
        return Trajectory.create_eight_loop(
            center=center,
            extent=extent,
            duration_sec=duration_sec,
            speed_factor=speed_factor,
        )

    elif name_lower in ("circle", "orbit", "circle_orbit"):
        center = kwargs.get("center", (0.0, 0.0, 1.5))
        radius = kwargs.get("radius", 2.0)
        return Trajectory.create_circle_orbit(
            center=center,
            radius=radius,
            duration_sec=duration_sec,
            speed_factor=speed_factor,
        )

    elif name_lower in ("random", "random_walk", "flythrough"):
        bounds_min = kwargs.get("bounds_min", (-2.0, -2.0, 0.5))
        bounds_max = kwargs.get("bounds_max", (2.0, 2.0, 2.5))
        num_waypoints = kwargs.get("num_waypoints", 10)
        seed = kwargs.get("seed", 42)
        return Trajectory.create_random_walk(
            bounds_min=bounds_min,
            bounds_max=bounds_max,
            num_waypoints=num_waypoints,
            duration_sec=duration_sec,
            seed=seed,
        )

    else:
        raise ValueError(
            f"Unknown trajectory preset: '{name}'. Available: 'eight_loop', 'circle_orbit', 'random_walk'"
        )
