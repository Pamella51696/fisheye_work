"""Kannala-Brandt (equidistant) fisheye model — forward and inverse mapping."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class FisheyeIntrinsics:
    """Pinhole intrinsics + KB distortion coefficients k1..k4."""

    fx: float
    fy: float
    cx: float
    cy: float
    k1: float = 0.0
    k2: float = 0.0
    k3: float = 0.0
    k4: float = 0.0

    @property
    def K(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    @property
    def D(self) -> np.ndarray:
        return np.array([[self.k1], [self.k2], [self.k3], [self.k4]], dtype=np.float64)

    def scaled(self, sx: float, sy: float) -> FisheyeIntrinsics:
        return FisheyeIntrinsics(
            self.fx * sx,
            self.fy * sy,
            self.cx * sx,
            self.cy * sy,
            self.k1,
            self.k2,
            self.k3,
            self.k4,
        )


def _theta_d(theta: np.ndarray, D: np.ndarray) -> np.ndarray:
    t2 = theta * theta
    poly = 1.0 + D[0] * t2 + D[1] * t2**2 + D[2] * t2**3 + D[3] * t2**4
    return theta * poly


def incidence_from_pixel(u: np.ndarray, v: np.ndarray, intr: FisheyeIntrinsics) -> Tuple[np.ndarray, np.ndarray]:
    """
    Distorted pixel (u,v) -> unit direction in camera frame (x,y,z) with z along optical axis.
    Uses Newton iteration to invert r = f * theta_d(theta).
    """
    x_d = (u - intr.cx) / intr.fx
    y_d = (v - intr.cy) / intr.fy
    r_d = np.hypot(x_d, y_d)
    D = intr.D.ravel()

    theta = r_d.copy()
    for _ in range(12):
        t2 = theta * theta
        poly = 1.0 + D[0] * t2 + D[1] * t2**2 + D[2] * t2**3 + D[3] * t2**4
        f_val = theta * poly - r_d
        d_poly = D[0] + 2 * D[1] * t2 + 3 * D[2] * t2**2 + 4 * D[3] * t2**3
        df = poly + theta * d_poly
        theta = theta - f_val / np.maximum(df, 1e-8)

    with np.errstate(divide="ignore", invalid="ignore"):
        scale = np.where(r_d > 1e-10, np.tan(theta) / r_d, 1.0)
        scale = np.nan_to_num(scale, nan=1.0, posinf=1.0, neginf=1.0)
    x = x_d * scale
    y = y_d * scale
    z = np.ones_like(x)
    norm = np.sqrt(x * x + y * y + z * z)
    return x / norm, y / norm, z / norm


def apply_distortion(
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    intr: FisheyeIntrinsics,
) -> Tuple[np.ndarray, np.ndarray]:
    """3D ray (x,y,z) -> distorted pixel (u,v). Forward KB model."""
    r = np.hypot(x, y)
    theta = np.arctan2(r, z)
    D = intr.D.ravel()
    theta_d = _theta_d(theta, D)
    with np.errstate(divide="ignore", invalid="ignore"):
        s = np.where(r > 1e-10, theta_d / r, 0.0)
        s = np.nan_to_num(s, nan=0.0, posinf=0.0, neginf=0.0)
    u = intr.fx * x * s + intr.cx
    v = intr.fy * y * s + intr.cy
    return u, v


def pixel_from_incidence(
    azimuth: np.ndarray,
    elevation: np.ndarray,
    intr: FisheyeIntrinsics,
) -> Tuple[np.ndarray, np.ndarray]:
    """Spherical angles -> distorted pixel. azimuth in [-pi,pi], elevation from horizon."""
    ce = np.cos(elevation)
    x = ce * np.sin(azimuth)
    y = np.sin(elevation)
    z = ce * np.cos(azimuth)
    return apply_distortion(x, y, z, intr)


def default_intrinsics(width: int, height: int, fov_deg: float = 190.0) -> FisheyeIntrinsics:
    """Equidistant focal length from full-image FOV (edge pixel incidence)."""
    f = (min(width, height) / 2.0) / np.radians(fov_deg / 2.0)
    return FisheyeIntrinsics(f, f, width / 2.0, height / 2.0)
