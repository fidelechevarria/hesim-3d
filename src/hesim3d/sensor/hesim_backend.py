from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple, Union
import numpy as np
import torch
import torch.nn as nn
from .config import SensorConfig


def gaussian_q(x: torch.Tensor) -> torch.Tensor:
    """Gaussian Q-function: Q(x) = 0.5 * erfc(x / sqrt(2))."""
    return 0.5 * torch.erfc(x / math.sqrt(2.0))


class HESIMEventSimulator(nn.Module):
    """
    Vectorized PyTorch implementation of the ECCV 2026 H-ESIM noise model.
    Accurately models physical log-signal difference, CFA-dependent shot/read noise,
    threshold voltage variations, and probabilistic event firing.
    """

    def __init__(self, config: SensorConfig, device: Optional[str] = None):
        super().__init__()
        self.config = config

        # Determine target compute device (CUDA if available)
        if device is None:
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
        else:
            self.device = device

        # Parse beta parameters [CFA_H, CFA_W, 6]
        betas_arr = np.array(config.evs_betas, dtype=np.float32)
        if betas_arr.ndim == 2:
            betas_arr = betas_arr[None, :, :]
        if betas_arr.shape[-1] != 6:
            raise ValueError(f"Expected 6 beta parameters per CFA block, got shape {betas_arr.shape}")

        self.register_buffer("betas", torch.from_numpy(betas_arr).to(self.device))
        self.cfa_size = int(config.cfa_size)
        self.theta_hw = float(config.theta_hw) * float(config.theta_scale)
        self.refractory_period_us = float(config.refractory_period_us)

        # State tracking for refractory period and dark current
        self.last_timestamp_map: Optional[torch.Tensor] = None
        self.rng = torch.Generator(device=self.device)
        self.rng.manual_seed(9527)

    def reset_state(self, height: int, width: int) -> None:
        """Reset pixel internal timestamp tracking maps."""
        self.last_timestamp_map = torch.zeros((height, width), device=self.device, dtype=torch.float64)

    def _tile_betas_per_pixel(self, H: int, W: int) -> Dict[str, torch.Tensor]:
        """Broadcast CFA beta parameters to full image resolution."""
        if self.cfa_size == 1 or self.betas.shape[0] == 1:
            b = self.betas[0, 0]
            return {f"b{k}": b[k].expand(H, W) for k in range(6)}

        bmaps = {}
        for k in range(6):
            tile = torch.zeros((H, W), device=self.device, dtype=torch.float32)
            for r_off in range(self.cfa_size):
                for c_off in range(self.cfa_size):
                    mask_r = (torch.arange(H, device=self.device) % self.cfa_size) == r_off
                    mask_c = (torch.arange(W, device=self.device) % self.cfa_size) == c_off
                    mask = mask_r[:, None] & mask_c[None, :]
                    tile[mask] = self.betas[r_off, c_off, k]
            bmaps[f"b{k}"] = tile
        return bmaps

    @torch.no_grad()
    def simulate_interval(
        self,
        I0: torch.Tensor,
        I1: torch.Tensor,
        t0_us: float,
        t1_us: float,
    ) -> Tuple[torch.Tensor, np.ndarray]:
        """
        Simulate events over time interval [t0_us, t1_us].
        
        Args:
            I0: (H, W) tensor of initial raw photon intensity [0, 1]
            I1: (H, W) tensor of final raw photon intensity [0, 1]
            t0_us: Start timestamp in microseconds
            t1_us: End timestamp in microseconds

        Returns:
            event_frame: (H, W) int8 tensor with values in {-1, 0, +1}
            event_array: (N, 4) numpy structured array with columns [t_us, x, y, polarity]
        """
        H, W = I0.shape
        if self.last_timestamp_map is None or self.last_timestamp_map.shape != (H, W):
            self.reset_state(H, W)

        I0 = I0.to(self.device, dtype=torch.float32)
        I1 = I1.to(self.device, dtype=torch.float32)

        betas = self._tile_betas_per_pixel(H, W)
        b0, b1, b2, b3, b4, b5 = (betas["b0"], betas["b1"], betas["b2"], betas["b3"], betas["b4"], betas["b5"])

        # Average intensity across interval
        Ic = 0.5 * (I0 + I1)

        # Physical Log-Voltage Signal: S = ln(b1 * I1 + b2) - ln(b1 * I0 + b2)
        V0 = torch.clamp(b1 * I0 + b2, min=1e-15)
        V1 = torch.clamp(b1 * I1 + b2, min=1e-15)
        S = torch.log(V1) - torch.log(V0)

        # Log-Voltage Noise Standard Deviation sigma_n(I)
        # Denom = 2 * ((b3 * Ic)^2 + b4^2 + 2 * b5 * b3 * Ic * b4)
        var_inside = 2.0 * ((b3 * Ic) ** 2 + (b4 ** 2) + 2.0 * b5 * b3 * Ic * b4)
        var_inside = torch.clamp(var_inside, min=1e-15)
        denom_inside = torch.sqrt(var_inside)
        denom_1 = torch.clamp(b1 * Ic + b2, min=1e-15)
        sigma_n = denom_inside / denom_1

        # Effective positive threshold voltage
        theta_pos = b0 * self.theta_hw

        # Probabilities of positive (ON) and negative (OFF) events
        z_on = (theta_pos - S) / torch.clamp(sigma_n, min=1e-15)
        P_on = gaussian_q(z_on)

        z_off = (-theta_pos - S) / torch.clamp(sigma_n, min=1e-15)
        P_off = 1.0 - gaussian_q(z_off)

        # Add dark noise background rate
        dt_sec = max(1e-9, (t1_us - t0_us) * 1e-6)
        p_dark_pos = float(self.config.dark_event_rate_pos) * dt_sec
        p_dark_neg = float(self.config.dark_event_rate_neg) * dt_sec

        P_on = torch.clamp(P_on + p_dark_pos, 0.0, 1.0)
        P_off = torch.clamp(P_off + p_dark_neg, 0.0, 1.0)

        # Probabilistic event firing via uniform random sample
        U = torch.rand((H, W), device=self.device, generator=self.rng)
        event_frame = torch.zeros((H, W), device=self.device, dtype=torch.int8)

        th_pos = P_on
        th_neg = torch.clamp(P_on + P_off, max=1.0)

        mask_pos = U < th_pos
        mask_neg = (U >= th_pos) & (U < th_neg)

        # Apply refractory period filtering
        dt_since_last = (t1_us - self.last_timestamp_map)
        valid_refractory = dt_since_last >= self.refractory_period_us

        active_pos = mask_pos & valid_refractory
        active_neg = mask_neg & valid_refractory

        event_frame[active_pos] = 1
        event_frame[active_neg] = -1

        # Update last timestamp for firing pixels
        fired_mask = active_pos | active_neg
        self.last_timestamp_map[fired_mask] = t1_us

        # Extract structured event tuples (t, x, y, p)
        fired_indices = torch.nonzero(fired_mask, as_tuple=False)
        num_events = fired_indices.shape[0]

        if num_events > 0:
            ys = fired_indices[:, 0].cpu().numpy()
            xs = fired_indices[:, 1].cpu().numpy()
            pols = event_frame[fired_indices[:, 0], fired_indices[:, 1]].cpu().numpy()
            ts = np.full(num_events, t1_us, dtype=np.uint64)

            # Structured array: [('t', 'u8'), ('x', 'u2'), ('y', 'u2'), ('p', 'i1')]
            dtype = [("t", np.uint64), ("x", np.uint16), ("y", np.uint16), ("p", np.int8)]
            event_array = np.empty(num_events, dtype=dtype)
            event_array["t"] = ts
            event_array["x"] = xs.astype(np.uint16)
            event_array["y"] = ys.astype(np.uint16)
            event_array["p"] = pols.astype(np.int8)
        else:
            dtype = [("t", np.uint64), ("x", np.uint16), ("y", np.uint16), ("p", np.int8)]
            event_array = np.empty(0, dtype=dtype)

        return event_frame, event_array


def accumulate_events_to_image(
    events: np.ndarray,
    width: int,
    height: int,
    bg_color: Tuple[int, int, int] = (25, 25, 30),
    pos_color: Tuple[int, int, int] = (255, 50, 50),
    neg_color: Tuple[int, int, int] = (50, 100, 255),
) -> np.ndarray:
    """Render a batch of events into an RGB accumulation image for visualization."""
    img = np.full((height, width, 3), bg_color, dtype=np.uint8)
    if len(events) == 0:
        return img

    xs = events["x"].astype(np.int32)
    ys = events["y"].astype(np.int32)
    ps = events["p"].astype(np.int32)

    valid = (xs >= 0) & (xs < width) & (ys >= 0) & (ys < height)
    xs = xs[valid]
    ys = ys[valid]
    ps = ps[valid]

    # Draw OFF events (blue) then ON events (red)
    mask_neg = ps == -1
    img[ys[mask_neg], xs[mask_neg]] = neg_color

    mask_pos = ps == 1
    img[ys[mask_pos], xs[mask_pos]] = pos_color

    return img
