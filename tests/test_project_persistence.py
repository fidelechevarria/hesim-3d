import json
import tempfile
from pathlib import Path
import pytest

from hesim3d.sensor.config import SensorConfig
from hesim3d._hesim3d_core import GuiConfig


def test_sensor_config_tuning_defaults():
    """Verify factory defaults for low-level tuning parameters across all presets."""
    eiger = SensorConfig.from_preset("alpsentek_eiger")
    assert eiger.event_threshold == 0.18
    assert eiger.refractory_period_us == 10.0

    davis = SensorConfig.from_preset("davis346")
    assert davis.event_threshold == 0.25
    assert davis.refractory_period_us == 20.0

    evk4 = SensorConfig.from_preset("prophesee_evk4")
    assert evk4.event_threshold == 0.15
    assert evk4.refractory_period_us == 5.0

    imx = SensorConfig.from_preset("sony_imx636")
    assert imx.event_threshold == 0.15
    assert imx.refractory_period_us == 3.0


def test_cpp_gui_config_bindings():
    """Verify C++ GuiConfig has refractory_period_us and project_path exposed."""
    cfg = GuiConfig()
    cfg.sensor_name = "prophesee_evk4"
    cfg.event_threshold = 0.12
    cfg.refractory_period_us = 4
    cfg.project_path = "/tmp/test.hesim"

    assert cfg.sensor_name == "prophesee_evk4"
    assert pytest.approx(cfg.event_threshold, 1e-4) == 0.12
    assert cfg.refractory_period_us == 4
    assert cfg.project_path == "/tmp/test.hesim"


def test_project_file_structure():
    """Verify serialization structure of hesim3d_project files."""
    with tempfile.TemporaryDirectory() as tmpdir:
        proj_path = Path(tmpdir) / "test_project.hesim"

        project_data = {
            "version": "2.0",
            "format": "hesim3d_project",
            "scene": "chessboard",
            "sensor": {
                "name": "prophesee_evk4",
                "event_threshold": 0.14,
                "refractory_period_us": 5,
                "sampling_rate_preset": 2,
                "exposure_ms": 10.0,
                "accumulation_window_ms": 33.33,
            },
            "trajectory": {
                "duration_sec": 4.5,
                "interpolation": "spline_se3",
                "keyframes": [
                    {
                        "time_sec": 0.0,
                        "position": [0.0, 1.0, 2.0],
                        "rotation_euler_deg": [0.0, 0.0, 0.0],
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                    {
                        "time_sec": 4.5,
                        "position": [1.0, 2.0, 3.0],
                        "rotation_euler_deg": [10.0, -5.0, 0.0],
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                ],
            },
        }

        with open(proj_path, "w", encoding="utf-8") as f:
            json.dump(project_data, f, indent=2)

        assert proj_path.exists()

        # Validate reloading
        with open(proj_path, "r", encoding="utf-8") as f:
            loaded = json.load(f)

        assert loaded["format"] == "hesim3d_project"
        assert loaded["sensor"]["name"] == "prophesee_evk4"
        assert loaded["sensor"]["event_threshold"] == 0.14
        assert loaded["sensor"]["refractory_period_us"] == 5
        assert len(loaded["trajectory"]["keyframes"]) == 2
