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


def test_realistic_event_rates_and_hdf5_streaming(tmp_path):
    import h5py
    from hesim3d.io.hdf5_writer import HDF5DatasetWriter, export_baked_simulation

    engine = ScientificBakeEngine("alpsentek_eiger", 640, 480, event_threshold=0.18, refractory_period_us=10)
    H, W = 480, 640

    # 1. Verify static scene produces minimal noise events (Poisson dark current)
    f_static = np.full((H, W, 3), 128, dtype=np.uint8)
    sub_bytes = np.stack([f_static, f_static], axis=0).tobytes()
    ts = [0, 1000]
    ev_t, _, _, _, _ = engine.process_aps_subsamples(sub_bytes, ts, shutter_duration_us=1000)
    # 307,200 pixels over 1 ms should produce very low dark events (< 500)
    assert len(ev_t) < 500, f"Static scene should not explode with noise, got {len(ev_t)} events"

    # 2. Verify dynamic scene produces realistic physical event rates (~10k-50k per frame)
    engine.reset()
    f1 = np.zeros((H, W, 3), dtype=np.uint8)
    f2 = np.zeros((H, W, 3), dtype=np.uint8)
    f1[:, 100:200] = 220
    f2[:, 120:220] = 220
    dyn_bytes = np.stack([f1, f2], axis=0).tobytes()
    ev_t, ev_x, ev_y, ev_p, _ = engine.process_aps_subsamples(dyn_bytes, ts, shutter_duration_us=1000)
    assert len(ev_t) > 1000, f"Moving edge must generate events, got {len(ev_t)}"
    assert len(ev_t) < 50000, f"Single sub-step events must remain bounded, got {len(ev_t)}"

    # 3. Verify streaming HDF5 writer works without RAM buffering
    h5_path = tmp_path / "streaming_test.h5"
    with HDF5DatasetWriter(h5_path, engine.config) as writer:
        for _ in range(5):
            writer.write_events_raw(ev_t, ev_x, ev_y, ev_p)

    with h5py.File(h5_path, "r") as h5:
        assert h5.attrs["format"] == "HESIM3D_HDF5"
        assert len(h5["events/t"]) == len(ev_t) * 5
        assert h5["events"].attrs["num_events"] == len(ev_t) * 5


