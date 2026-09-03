from __future__ import annotations

import argparse
import sys
from pathlib import Path
import h5py
import numpy as np

from hesim3d.simulator import Simulator
from hesim3d.trajectory.presets import get_trajectory_preset
from hesim3d.sensor.config import SensorConfig
from hesim3d.assets.catalog import list_available_scenes
from hesim3d.assets.downloader import download_scene, resolve_scene_path
from hesim3d.gui.launcher import launch_interactive_gui


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="hesim3d",
        description="hesim-3d: Photorealistic 3D Neuromorphic Event-Frame Sensor Simulator",
    )
    subparsers = parser.add_subparsers(dest="command", help="Sub-commands")

    # 1. run command
    run_p = subparsers.add_parser("run", help="Run batch headless simulation and export datasets")
    run_p.add_argument("--scene", type=str, default="chessboard", help="3D Scene ID or path (.glb)")
    run_p.add_argument("--sensor", type=str, default="alpsentek_eiger", help="Sensor preset name")
    run_p.add_argument("--trajectory", type=str, default="eight_loop", help="Trajectory preset (eight_loop, circle_orbit, random_walk)")
    run_p.add_argument("--duration", type=float, default=2.0, help="Simulation duration in seconds")
    run_p.add_argument("--fps", type=float, default=500.0, help="Intermediate simulation sampling rate (Hz)")
    run_p.add_argument("--backend", type=str, default="vulkan", choices=["vulkan", "opengl"], help="Filament graphics backend")
    run_p.add_argument("--output", "-o", type=str, default="output_dataset.h5", help="Output file (.h5 or .mcap)")

    # 2. gui command
    gui_p = subparsers.add_parser("gui", help="Launch interactive Dear ImGui real-time visualizer")
    gui_p.add_argument("--scene", type=str, default="chessboard", help="3D Scene ID or path (.glb)")
    gui_p.add_argument("--sensor", type=str, default="alpsentek_eiger", help="Sensor preset name")
    gui_p.add_argument("--trajectory", type=str, default="", help="Path to custom trajectory (.json) or preset")
    gui_p.add_argument("--duration", type=float, default=5.0, help="Loop duration in seconds")
    gui_p.add_argument("--fps", type=float, default=1000.0, help="Simulation sampling rate (Hz)")
    gui_p.add_argument("--threshold", type=float, default=0.20, help="Contrast sensitivity threshold")
    gui_p.add_argument("--width", type=int, default=1600, help="Window width")
    gui_p.add_argument("--height", type=int, default=1000, help="Window height")

    # 3. assets command
    assets_p = subparsers.add_parser("assets", help="Manage 3D benchmark scenes and environments")
    assets_sub = assets_p.add_subparsers(dest="assets_cmd", help="Asset operations")

    assets_sub.add_parser("list", help="List registered scene assets in catalog")
    down_p = assets_sub.add_parser("download", help="Download a scene asset to local cache")
    down_p.add_argument("scene_id", type=str, help="Scene ID (e.g. sponza, cornell_box, bistro)")
    down_p.add_argument("--force", action="store_true", help="Force re-download")

    # 4. inspect command
    inspect_p = subparsers.add_parser("inspect", help="Inspect generated HDF5 / MCAP datasets")
    inspect_p.add_argument("file_path", type=str, help="Path to .h5 dataset file")

    return parser


def handle_run(args: argparse.Namespace) -> int:
    print(f"[hesim-3d] Setting up simulation on scene: {args.scene}...")
    sensor_cfg = SensorConfig.from_preset(args.sensor)
    traj = get_trajectory_preset(args.trajectory, duration_sec=args.duration)

    sim = Simulator(
        scene=args.scene,
        sensor_config=sensor_cfg,
        trajectory=traj,
        backend_type=args.backend,
    )

    results = sim.run(
        duration_sec=args.duration,
        sim_fps=args.fps,
        output_path=args.output,
    )

    print("\n[hesim-3d] Simulation completed successfully!")
    print(f"  - Output file:      {results['output_path']}")
    print(f"  - Total events:     {results['total_events']:,}")
    print(f"  - Total APS frames: {results['total_frames']}")
    print(f"  - Duration:         {results['duration_sec']}s @ {results['sim_fps']} Hz")
    return 0


def handle_gui(args: argparse.Namespace) -> int:
    print(f"[hesim-3d] Launching interactive Dear ImGui visualizer on scene: {args.scene}...")
    return launch_interactive_gui(
        scene=args.scene,
        sensor_name=args.sensor,
        trajectory=args.trajectory,
        duration_sec=args.duration,
        sim_fps=args.fps,
        event_threshold=args.threshold,
        window_width=args.width,
        window_height=args.height,
    )


def handle_assets(args: argparse.Namespace) -> int:
    if args.assets_cmd == "list":
        scenes = list_available_scenes()
        print("\nRegistered 3D Benchmark Scenes:")
        print(f"{'ID':<20} {'Size (MB)':<12} {'License':<18} {'Name'}")
        print("-" * 75)
        for s in scenes:
            builtin_str = " (Built-in)" if s.built_in else ""
            print(f"{s.id:<20} {s.size_mb:<12.1f} {s.license:<18} {s.name}{builtin_str}")
        print()
        return 0

    elif args.assets_cmd == "download":
        p = download_scene(args.scene_id, force=args.force)
        print(f"[hesim-3d] Scene downloaded and verified at: {p}")
        return 0

    else:
        print("[hesim-3d] Specify an asset command: 'list' or 'download <scene_id>'")
        return 1


def handle_inspect(args: argparse.Namespace) -> int:
    p = Path(args.file_path)
    if not p.exists():
        print(f"[hesim-3d] Error: file '{args.file_path}' does not exist.")
        return 1

    if p.suffix == ".h5":
        with h5py.File(p, "r") as f:
            print(f"\n=== Dataset Inspection: {p.name} ===")
            for k, v in f.attrs.items():
                print(f"  Attribute [{k}]: {v}")

            print("\nGroups & Datasets:")
            if "events" in f:
                num_events = len(f["events/t"]) if "t" in f["events"] else 0
                print(f"  - events/: {num_events:,} events")
                if num_events > 0:
                    t_min, t_max = f["events/t"][0], f["events/t"][-1]
                    print(f"    Timestamp range: {t_min / 1e6:.3f}s -> {t_max / 1e6:.3f}s (dt = {(t_max - t_min) / 1e6:.3f}s)")
            if "frames" in f:
                num_frames = len(f["frames/timestamps_us"]) if "timestamps_us" in f["frames"] else 0
                shape = f["frames/images"].shape if "images" in f["frames"] else "N/A"
                print(f"  - frames/: {num_frames} frames (shape: {shape})")
            if "imu" in f:
                num_imu = len(f["imu/timestamps_us"]) if "timestamps_us" in f["imu"] else 0
                print(f"  - imu/: {num_imu:,} samples")
            if "ground_truth" in f:
                num_gt = len(f["ground_truth/timestamps_us"]) if "timestamps_us" in f["ground_truth"] else 0
                print(f"  - ground_truth/: {num_gt:,} 6-DoF poses")
            print("===================================\n")
        return 0
    else:
        print(f"[hesim-3d] MCAP inspection: File size = {p.stat().st_size / 1024:.1f} KB")
        return 0


def main() -> None:
    parser = create_parser()
    args = parser.parse_args()

    if args.command == "run":
        sys.exit(handle_run(args))
    elif args.command == "gui":
        sys.exit(handle_gui(args))
    elif args.command == "assets":
        sys.exit(handle_assets(args))
    elif args.command == "inspect":
        sys.exit(handle_inspect(args))
    else:
        parser.print_help()
        sys.exit(0)


if __name__ == "__main__":
    main()
