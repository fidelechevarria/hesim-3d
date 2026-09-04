import numpy as np
import pytest
import torch

from hesim3d.sensor.scientific_bake import (
    ScientificBakeEngine,
    init_scientific_engine,
    step_scientific_engine,
    reset_scientific_engine,
)


def test_scientific_bake_engine_eiger_cuda_or_cpu():
    engine = ScientificBakeEngine("alpsentek_eiger", 640, 480, event_threshold=0.18, refractory_period_us=10)
    info = engine.get_device_info()
    assert "H-ESIM 6-Beta CFA" in info["noise_model"]
    assert info["cfa_pattern"] == "quad_bayer"

    n_samples = 4
    H, W = 480, 640
    # Create gradient moving across frames
    frames = []
    for i in range(n_samples):
        arr = np.zeros((H, W, 3), dtype=np.uint8)
        arr[:, (i * 20) : (i * 20 + 100)] = 200
        frames.append(arr)
    frames_bytes = b"".join(f.tobytes() for f in frames)
    ts = [0, 2000, 4000, 6000]

    ev_t, ev_x, ev_y, ev_p, blurred_bytes = engine.process_aps_subsamples(
        frames_bytes, ts, shutter_duration_us=4000
    )

    assert len(blurred_bytes) == H * W * 3
    if len(ev_t) > 0:
        assert np.all(ev_x < W)
        assert np.all(ev_y < H)
        assert np.all(np.isin(ev_p, [-1, 1]))


def test_scientific_engine_global_singleton():
    dev, model = init_scientific_engine("davis346", 346, 260, 0.25, 20)
    assert "davis" in dev.lower() or "cuda" in dev.lower() or "cpu" in dev.lower()

    H, W = 260, 346
    frames = np.ones((2, H, W, 3), dtype=np.uint8) * 128
    frames_bytes = frames.tobytes()
    ts = [0, 5000]

    ev_t, ev_x, ev_y, ev_p, blurred_bytes = step_scientific_engine(
        frames_bytes, ts, shutter_duration_us=5000
    )
    assert len(blurred_bytes) == H * W * 3
    reset_scientific_engine()


def test_cpp_scientific_bake_bridge_roundtrip():
    from hesim3d._hesim3d_core import _test_cpp_scientific_bake_roundtrip
    ok = _test_cpp_scientific_bake_roundtrip("sony_imx636", 1280, 720)
    assert ok is True

