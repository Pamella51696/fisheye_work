"""LUT-based undistortion: output pinhole <- fisheye source."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from fisheye.model import FisheyeIntrinsics, apply_distortion


def build_remap_maps(
    out_w: int,
    out_h: int,
    intr: FisheyeIntrinsics,
    fov_out_deg: float = 110.0,
    balance: float = 0.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Inverse mapping: for each output (pinhole) pixel, source fisheye (u,v).
    balance in [0,1]: 0 = max FOV/coverage, 1 = crop toward center (less stretch).
    """
    fov = fov_out_deg * (1.0 - 0.35 * np.clip(balance, 0.0, 1.0))
    f_out = (out_w / 2.0) / np.tan(np.radians(fov / 2.0))
    u, v = np.meshgrid(np.arange(out_w, dtype=np.float64), np.arange(out_h, dtype=np.float64))
    x = (u - out_w / 2.0) / f_out
    y = (v - out_h / 2.0) / f_out
    z = np.ones_like(x)
    norm = np.sqrt(x * x + y * y + z * z)
    x, y, z = x / norm, y / norm, z / norm
    map_x, map_y = apply_distortion(x, y, z, intr)
    return map_x.astype(np.float32), map_y.astype(np.float32)


@dataclass
class UndistortLUT:
    map_x: np.ndarray
    map_y: np.ndarray
    fov_out_deg: float
    out_size: Tuple[int, int]

    @classmethod
    def create(
        cls,
        out_w: int,
        out_h: int,
        intr: FisheyeIntrinsics,
        fov_out_deg: float = 110.0,
        balance: float = 0.0,
    ) -> UndistortLUT:
        mx, my = build_remap_maps(out_w, out_h, intr, fov_out_deg, balance)
        return cls(mx, my, fov_out_deg, (out_w, out_h))

    def remap(self, frame: np.ndarray, interpolation: int = cv2.INTER_LINEAR) -> np.ndarray:
        return cv2.remap(
            frame,
            self.map_x,
            self.map_y,
            interpolation,
            borderMode=cv2.BORDER_CONSTANT,
        )

    def fixed_point_maps(self) -> Tuple[np.ndarray, np.ndarray]:
        return cv2.convertMaps(self.map_x, self.map_y, cv2.CV_16SC2)

    def mean_map_error_vs(
        self,
        other_map_x: np.ndarray,
        other_map_y: np.ndarray,
        mask: Optional[np.ndarray] = None,
    ) -> float:
        if mask is None:
            mask = np.isfinite(self.map_x) & np.isfinite(other_map_x)
        dx = self.map_x - other_map_x
        dy = self.map_y - other_map_y
        err = np.sqrt(dx * dx + dy * dy)
        return float(np.mean(err[mask])) if np.any(mask) else float("nan")


def build_cylindrical_maps(
    out_w: int,
    out_h: int,
    intr: FisheyeIntrinsics,
    hfov_deg: float = 180.0,
    vfov_deg: float = 60.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """Cylindrical panorama sampling from fisheye."""
    hfov = np.radians(hfov_deg)
    vfov = np.radians(vfov_deg)
    u, v = np.meshgrid(np.arange(out_w, dtype=np.float64), np.arange(out_h, dtype=np.float64))
    theta = (u / out_w - 0.5) * hfov
    phi = (0.5 - v / out_h) * vfov
    x = np.sin(theta) * np.cos(phi)
    y = np.sin(phi)
    z = np.cos(theta) * np.cos(phi)
    return apply_distortion(x, y, z, intr)
