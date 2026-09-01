from __future__ import annotations

import numpy as np
import cv2
import torch
from typing import Tuple, Optional, Union
from .config import SensorConfig


def inverse_srgb_oetf(img_srgb: np.ndarray) -> np.ndarray:
    """Convert sRGB [0, 255] or [0, 1] into linear physical radiance [0, 1]."""
    x = np.clip(img_srgb.astype(np.float32) / 255.0 if img_srgb.dtype == np.uint8 else img_srgb.astype(np.float32), 0.0, 1.0)
    thr = 0.04045
    linear = np.empty_like(x)
    lin_mask = x <= thr
    linear[lin_mask] = x[lin_mask] / 12.92
    linear[~lin_mask] = np.power((x[~lin_mask] + 0.055) / 1.055, 2.4)
    return linear


def forward_srgb_oetf(img_lin: np.ndarray) -> np.ndarray:
    """Convert linear radiance [0, 1] into standard display sRGB [0, 255] uint8."""
    x = np.clip(img_lin, 0.0, 1.0)
    a = 0.055
    thr = 0.0031308
    lin_mask = x <= thr
    srgb = np.empty_like(x)
    srgb[lin_mask] = 12.92 * x[lin_mask]
    srgb[~lin_mask] = (1.0 + a) * np.power(x[~lin_mask], 1.0 / 2.4) - a
    return (np.clip(srgb, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)


class SensorISP:
    """Simulates realistic Camera ISP, Color Filter Array (CFA) physics, and sensor noise."""

    def __init__(self, config: SensorConfig):
        self.config = config
        self.ccm = np.array(config.color_correction_matrix, dtype=np.float32)
        try:
            self.inv_ccm = np.linalg.inv(self.ccm)
        except np.linalg.LinAlgError:
            self.inv_ccm = np.eye(3, dtype=np.float32)

    def srgb_to_raw(self, img_srgb: np.ndarray, add_noise: bool = True) -> np.ndarray:
        """Transform an RGB rendered frame into raw sensor mosaic with Poisson-Gaussian noise."""
        H, W = img_srgb.shape[:2]

        # 1. Linearize radiance
        lin_rgb = inverse_srgb_oetf(img_srgb)

        # 2. Apply inverse Color Correction Matrix
        raw_rgb = lin_rgb @ self.inv_ccm.T
        raw_rgb = np.clip(raw_rgb, 0.0, 1.0)

        # 3. Apply CFA Masking
        if self.config.cfa_pattern == "quad_bayer":
            # 4x4 super-pixel pattern (2x2 Quad-Bayer: GG, RR, BB, GG)
            rr = np.arange(H)[:, None] % 4
            cc = np.arange(W)[None, :] % 4

            mask_R = (rr < 2) & (cc >= 2)
            mask_B = (rr >= 2) & (cc < 2)
            mask_G = ~(mask_R | mask_B)

            raw_mosaic = np.zeros((H, W), dtype=np.float32)
            raw_mosaic[mask_R] = raw_rgb[mask_R, 0]
            raw_mosaic[mask_G] = raw_rgb[mask_G, 1]
            raw_mosaic[mask_B] = raw_rgb[mask_B, 2]

        elif self.config.cfa_pattern == "rggb":
            # Standard 2x2 RGGB Bayer
            rr = np.arange(H)[:, None] % 2
            cc = np.arange(W)[None, :] % 2

            mask_R = (rr == 0) & (cc == 0)
            mask_G = ((rr == 0) & (cc == 1)) | ((rr == 1) & (cc == 0))
            mask_B = (rr == 1) & (cc == 1)

            raw_mosaic = np.zeros((H, W), dtype=np.float32)
            raw_mosaic[mask_R] = raw_rgb[mask_R, 0]
            raw_mosaic[mask_G] = raw_rgb[mask_G, 1]
            raw_mosaic[mask_B] = raw_rgb[mask_B, 2]

        else: # Monochrome
            raw_mosaic = np.mean(raw_rgb, axis=2)

        # 4. Add sensor shot & read noise
        if add_noise:
            # Poisson-Gaussian variance: sigma^2(I) = beta1 * I + beta2
            var = np.maximum(1e-10, self.config.aps_noise_beta1 * raw_mosaic + self.config.aps_noise_beta2)
            sigma = np.sqrt(var)
            noise = np.random.normal(0.0, 1.0, size=raw_mosaic.shape).astype(np.float32) * sigma
            raw_mosaic = np.clip(raw_mosaic + noise, 0.0, 1.0)

        return raw_mosaic

    def raw_to_rgb_preview(self, raw_mosaic: np.ndarray) -> np.ndarray:
        """Fast demosaic approximation to display raw sensor output in RGB viewports."""
        H, W = raw_mosaic.shape[:2]
        if self.config.cfa_pattern == "mono":
            gray = np.clip(raw_mosaic * 255.0, 0, 255).astype(np.uint8)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)

        # Quick bilinear reconstruction
        rgb_lin = np.zeros((H, W, 3), dtype=np.float32)
        rgb_lin[:, :, 0] = raw_mosaic
        rgb_lin[:, :, 1] = raw_mosaic
        rgb_lin[:, :, 2] = raw_mosaic

        # Apply forward CCM
        rgb_corr = rgb_lin @ self.ccm.T
        return forward_srgb_oetf(rgb_corr)
