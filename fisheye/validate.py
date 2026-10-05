"""Validation metrics for undistortion and calibration."""

from __future__ import annotations

from typing import Tuple

import cv2
import numpy as np

from fisheye.model import FisheyeIntrinsics, apply_distortion


def psnr(a: np.ndarray, b: np.ndarray) -> float:
    a = a.astype(np.float64)
    b = b.astype(np.float64)
    mse = np.mean((a - b) ** 2)
    if mse < 1e-12:
        return float("inf")
    return float(10 * np.log10(255.0**2 / mse))


def ssim_gray(a: np.ndarray, b: np.ndarray) -> float:
    """Simple global SSIM on grayscale (no external skimage dependency)."""
    if a.ndim == 3:
        a = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY)
    if b.ndim == 3:
        b = cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)
    a = a.astype(np.float64)
    b = b.astype(np.float64)
    c1, c2 = (0.01 * 255) ** 2, (0.03 * 255) ** 2
    mu_a, mu_b = a.mean(), b.mean()
    sigma_a, sigma_b = a.var(), b.var()
    sigma_ab = ((a - mu_a) * (b - mu_b)).mean()
    num = (2 * mu_a * mu_b + c1) * (2 * sigma_ab + c2)
    den = (mu_a**2 + mu_b**2 + c1) * (sigma_a + sigma_b + c2)
    return float(num / den)


def verify_forward_inverse(
    intr: FisheyeIntrinsics,
    width: int,
    height: int,
    n_samples: int = 5000,
    seed: int = 42,
) -> Tuple[float, float]:
    """
    Sample random 3D rays, project to pixel, unproject back; report angular and pixel error.
    """
    rng = np.random.default_rng(seed)
    x = rng.normal(size=n_samples)
    y = rng.normal(size=n_samples)
    z = rng.uniform(0.1, 1.0, size=n_samples)
    norm = np.sqrt(x * x + y * y + z * z)
    x, y, z = x / norm, y / norm, z / norm

    u, v = apply_distortion(x, y, z, intr)
    from fisheye.model import incidence_from_pixel

    x2, y2, z2 = incidence_from_pixel(u, v, intr)
    dot = np.clip(x * x2 + y * y2 + z * z2, -1.0, 1.0)
    ang_err = np.degrees(np.arccos(dot))

    u2, v2 = apply_distortion(x2, y2, z2, intr)
    pix_err = np.hypot(u - u2, v - v2)
    return float(np.median(ang_err)), float(np.median(pix_err))


def straightness_residual(image: np.ndarray, line_pts: np.ndarray) -> float:
    """Fit line to Nx2 points; return RMS distance to fitted line in pixels."""
    pts = line_pts.astype(np.float64)
    if len(pts) < 3:
        return float("nan")
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).ravel()
    dx, dy = pts[:, 0] - x0, pts[:, 1] - y0
    cross = np.abs(dx * vy - dy * vx)
    return float(np.sqrt(np.mean(cross**2)))
