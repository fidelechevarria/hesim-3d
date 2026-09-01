import numpy as np
import torch
import pytest
from hesim3d.sensor import (
    SensorConfig,
    SensorISP,
    inverse_srgb_oetf,
    forward_srgb_oetf,
    HESIMEventSimulator,
    accumulate_events_to_image,
)


def test_sensor_config_presets():
    eiger = SensorConfig.from_preset("alpsentek_eiger")
    assert eiger.cfa_pattern == "quad_bayer"
    assert eiger.cfa_size == 2
    assert len(eiger.evs_betas) == 2

    evk4 = SensorConfig.from_preset("prophesee_evk4")
    assert evk4.cfa_pattern == "mono"
    assert evk4.width == 1280


def test_inverse_and_forward_oetf():
    # Test round-trip sRGB <-> linear
    original = np.array([0, 64, 128, 192, 255], dtype=np.uint8)
    lin = inverse_srgb_oetf(original)
    assert np.all(lin >= 0.0) and np.all(lin <= 1.0)
    reconstructed = forward_srgb_oetf(lin)
    assert np.all(np.abs(original.astype(int) - reconstructed.astype(int)) <= 1)


def test_isp_pipeline():
    cfg = SensorConfig.from_preset("alpsentek_eiger")
    isp = SensorISP(cfg)

    rgb = np.random.randint(0, 256, (cfg.height, cfg.width, 3), dtype=np.uint8)
    raw = isp.srgb_to_raw(rgb, add_noise=True)

    assert raw.shape == (cfg.height, cfg.width)
    assert raw.dtype == np.float32
    assert np.all(raw >= 0.0) and np.all(raw <= 1.0)


def test_hesim_event_simulator():
    cfg = SensorConfig.from_preset("alpsentek_eiger")
    backend = HESIMEventSimulator(cfg, device="cpu")

    H, W = cfg.height, cfg.width
    I0 = torch.full((H, W), 0.2, dtype=torch.float32)
    I1 = torch.full((H, W), 0.2, dtype=torch.float32)
    # Create high contrast step in quadrant
    I1[: H // 2, : W // 2] = 0.8

    ev_frame, ev_array = backend.simulate_interval(I0, I1, t0_us=0, t1_us=1000)

    assert ev_frame.shape == (H, W)
    assert len(ev_array) > 0
    # Positive contrast step should produce ON events (+1)
    on_events = ev_array[ev_array["p"] == 1]
    assert len(on_events) > 0

    # Test event accumulator image
    acc_img = accumulate_events_to_image(ev_array, W, H)
    assert acc_img.shape == (H, W, 3)
    assert acc_img.dtype == np.uint8
