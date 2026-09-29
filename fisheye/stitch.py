"""Multi-camera surround stitching: undistort -> ground-plane BEV + feather blend."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from fisheye.model import FisheyeIntrinsics, apply_distortion
from fisheye.undistort import UndistortLUT


@dataclass
class CameraExtrinsics:
    """Camera pose: rotation (vehicle yaw/pitch/roll deg) and height above ground."""

    yaw_deg: float = 0.0
    pitch_deg: float = -15.0
    roll_deg: float = 0.0
    height_m: float = 1.2
    forward_m: float = 2.5


def _rotation_matrix(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    y, p, r = map(np.radians, (yaw_deg, pitch_deg, roll_deg))
    cy, sy = np.cos(y), np.sin(y)
    cp, sp = np.cos(p), np.sin(p)
    cr, sr = np.cos(r), np.sin(r)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


@dataclass
class SurroundStitcher:
    """
    Bird's-eye view stitcher for N fisheye cameras with known intrinsics and extrinsics.
    """

    intrinsics: Sequence[FisheyeIntrinsics]
    extrinsics: Sequence[CameraExtrinsics]
    bev_w: int = 800
    bev_h: int = 800
    meters_per_pixel: float = 0.02
    undistort_fov_deg: float = 100.0

    def __post_init__(self) -> None:
        if len(self.intrinsics) != len(self.extrinsics):
            raise ValueError("intrinsics and extrinsics must have same length")
        self._luts: List[Optional[UndistortLUT]] = [None] * len(self.intrinsics)

    def _ensure_lut(self, cam_idx: int, frame_shape: Tuple[int, int, int]) -> UndistortLUT:
        h, w = frame_shape[:2]
        if self._luts[cam_idx] is None or self._luts[cam_idx].out_size != (w, h):
            out_w, out_h = w, h
            self._luts[cam_idx] = UndistortLUT.create(
                out_w, out_h, self.intrinsics[cam_idx], self.undistort_fov_deg
            )
        return self._luts[cam_idx]

    def build_bev_maps(
        self,
        cam_idx: int,
        src_w: int,
        src_h: int,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        For each BEV pixel, sample fisheye source directly (skip separate undistort pass).
        Returns map_x, map_y, valid mask.
        """
        intr = self.intrinsics[cam_idx]
        ext = self.extrinsics[cam_idx]
        R_cam = _rotation_matrix(ext.yaw_deg, ext.pitch_deg, ext.roll_deg)

        cx_bev = self.bev_w / 2.0
        cy_bev = self.bev_h / 2.0
        u, v = np.meshgrid(np.arange(self.bev_w), np.arange(self.bev_h))
        x_veh = (u - cx_bev) * self.meters_per_pixel
        y_veh = (cy_bev - v) * self.meters_per_pixel
        z_veh = np.zeros_like(x_veh)

        # Vehicle frame: x right, y forward, z up (ground at z=0)
        P = np.stack([x_veh, y_veh, z_veh], axis=-1)
        cam_origin = np.array([0.0, ext.forward_m, ext.height_m])
        dirs_world = P - cam_origin
        dirs_world[..., 2] = -ext.height_m
        norm = np.linalg.norm(dirs_world, axis=-1, keepdims=True)
        valid = norm[..., 0] > 1e-6
        dirs_world = dirs_world / np.maximum(norm, 1e-8)

        flat = dirs_world.reshape(-1, 3).T
        dirs_cam = (R_cam.T @ flat).T.reshape(self.bev_h, self.bev_w, 3)
        x_c, y_c, z_c = dirs_cam[..., 0], dirs_cam[..., 1], dirs_cam[..., 2]
        look_valid = z_c > 0.05
        map_x, map_y = apply_distortion(x_c, y_c, z_c, intr)
        in_frame = (
            (map_x >= 0)
            & (map_x < src_w - 1)
            & (map_y >= 0)
            & (map_y < src_h - 1)
            & look_valid
            & valid
        )
        map_x = np.where(in_frame, map_x, -1).astype(np.float32)
        map_y = np.where(in_frame, map_y, -1).astype(np.float32)
        return map_x, map_y, in_frame.astype(np.float32)

    def stitch_frames(
        self,
        frames: Sequence[np.ndarray],
        feather_px: int = 40,
    ) -> np.ndarray:
        if len(frames) != len(self.intrinsics):
            raise ValueError("frame count must match camera count")
        accum = np.zeros((self.bev_h, self.bev_w, 3), dtype=np.float64)
        weight_sum = np.zeros((self.bev_h, self.bev_w), dtype=np.float64)

        for i, frame in enumerate(frames):
            h, w = frame.shape[:2]
            mx, my, valid = self.build_bev_maps(i, w, h)
            warped = cv2.remap(frame, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            w_mask = self._feather_weights(valid, feather_px)
            accum += warped.astype(np.float64) * w_mask[..., None]
            weight_sum += w_mask

        weight_sum = np.maximum(weight_sum, 1e-6)
        out = (accum / weight_sum[..., None]).astype(np.uint8)
        return out

    @staticmethod
    def _feather_weights(valid: np.ndarray, feather_px: int) -> np.ndarray:
        v = (valid > 0.5).astype(np.uint8)
        dist = cv2.distanceTransform(v, cv2.DIST_L2, 5)
        w = np.clip(dist / max(feather_px, 1), 0.0, 1.0)
        return w * valid

    def stitch_panorama_horizontal(
        self,
        frames: Sequence[np.ndarray],
        out_w: int = 2560,
        out_h: int = 480,
        hfov_total_deg: float = 360.0,
    ) -> np.ndarray:
        """Feather-stitch undistorted pinhole views laid out by camera yaw."""
        if not frames:
            raise ValueError("no frames")
        n = len(frames)
        hfov = np.radians(hfov_total_deg / n)
        panel_w = out_w // n
        canvas = np.zeros((out_h, out_w, 3), dtype=np.uint8)
        weight = np.zeros((out_h, out_w), dtype=np.float64)

        for i, frame in enumerate(frames):
            h, w = frame.shape[:2]
            lut = self._ensure_lut(i, frame.shape)
            pinhole = lut.remap(frame)
            ph = cv2.resize(pinhole, (panel_w, out_h), interpolation=cv2.INTER_LINEAR)
            x0 = i * panel_w
            x1 = x0 + panel_w
            alpha = np.linspace(0.3, 1.0, panel_w)[None, :]
            if i > 0:
                alpha[:, : min(80, panel_w // 4)] *= np.linspace(0, 1, min(80, panel_w // 4))
            w_panel = np.broadcast_to(alpha, (out_h, panel_w))
            canvas[:, x0:x1] = (
                canvas[:, x0:x1].astype(np.float64) * (1 - w_panel[..., None])
                + ph.astype(np.float64) * w_panel[..., None]
            ).astype(np.uint8)
            weight[:, x0:x1] += w_panel

        return canvas
