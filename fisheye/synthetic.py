"""Synthetic fisheye images from rectilinear sources with known ground truth."""

from __future__ import annotations

from typing import List, Optional, Tuple

import cv2
import numpy as np

from fisheye.model import FisheyeIntrinsics, apply_distortion


def build_inverse_warp_maps(
    src_w: int,
    src_h: int,
    dst_w: int,
    dst_h: int,
    intr: FisheyeIntrinsics,
    fov_src_deg: float = 90.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    For each destination (fisheye) pixel, sample from rectilinear source.
    Source is a virtual pinhole with given horizontal FOV.
    """
    f_src = (src_w / 2.0) / np.tan(np.radians(fov_src_deg / 2.0))
    u, v = np.meshgrid(np.arange(dst_w, dtype=np.float32), np.arange(dst_h, dtype=np.float32))
    x, y, z = incidence_from_pixels_grid(u, v, intr)
    valid = z > 1e-6
    u_src = np.where(valid, f_src * (x / z) + src_w / 2.0, -1.0).astype(np.float32)
    v_src = np.where(valid, f_src * (y / z) + src_h / 2.0, -1.0).astype(np.float32)
    return u_src, v_src


def incidence_from_pixels_grid(u: np.ndarray, v: np.ndarray, intr: FisheyeIntrinsics):
    """Inverse of apply_distortion: fisheye pixel -> ray direction."""
    from fisheye.model import incidence_from_pixel

    x, y, z = incidence_from_pixel(u, v, intr)
    return x, y, z


def warp_rectilinear_to_fisheye(
    rect: np.ndarray,
    intr: FisheyeIntrinsics,
    fov_src_deg: float = 90.0,
) -> np.ndarray:
    h, w = rect.shape[:2]
    map_x, map_y = build_inverse_warp_maps(w, h, w, h, intr, fov_src_deg)
    return cv2.remap(rect, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


def make_checkerboard(
    squares_x: int = 9,
    squares_y: int = 6,
    square_px: int = 80,
    margin: int = 40,
) -> np.ndarray:
    h = squares_y * square_px + 2 * margin
    w = squares_x * square_px + 2 * margin
    board = np.ones((h, w), dtype=np.uint8) * 255
    for j in range(squares_y):
        for i in range(squares_x):
            if (i + j) % 2 == 0:
                y0, x0 = margin + j * square_px, margin + i * square_px
                board[y0 : y0 + square_px, x0 : x0 + square_px] = 0
    return cv2.cvtColor(board, cv2.COLOR_GRAY2BGR)


def rotate_board_3d(
    board: np.ndarray,
    yaw_deg: float,
    pitch_deg: float,
    distance: float,
    out_w: int,
    out_h: int,
    fov_deg: float = 75.0,
) -> np.ndarray:
    """Render checkerboard as rectilinear view (pinhole) for synthetic capture."""
    f = (out_w / 2.0) / np.tan(np.radians(fov_deg / 2.0))
    yaw = np.radians(yaw_deg)
    pitch = np.radians(pitch_deg)
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    R = np.array(
        [
            [cy, 0, sy],
            [sp * sy, cp, -sp * cy],
            [-cp * sy, sp, cp * cy],
        ],
        dtype=np.float64,
    )
    bh, bw = board.shape[:2]
    plane_z = distance
    scale = distance / max(bw, bh) * 1.2

    u, v = np.meshgrid(np.arange(out_w), np.arange(out_h))
    x_cam = (u - out_w / 2.0) / f
    y_cam = (v - out_h / 2.0) / f
    z_cam = np.ones_like(x_cam)
    dirs = np.stack([x_cam, y_cam, z_cam], axis=-1)
    dirs /= np.linalg.norm(dirs, axis=-1, keepdims=True)
    world = (R.T @ dirs.reshape(-1, 3).T).T
    hit = world[:, 2] > 1e-4
    t = np.where(hit, plane_z / world[:, 2], -1.0)
    px_w = world[:, 0] * t
    py_w = world[:, 1] * t
    u_b = (px_w / scale + bw / 2.0).astype(np.float32)
    v_b = (py_w / scale + bh / 2.0).astype(np.float32)
    map_x = u_b.reshape(out_h, out_w)
    map_y = v_b.reshape(out_h, out_w)
    valid = (t > 0) & (u_b >= 0) & (u_b < bw) & (v_b >= 0) & (v_b < bh)
    map_x = np.where(valid.reshape(out_h, out_w), map_x, -1).astype(np.float32)
    map_y = np.where(valid.reshape(out_h, out_w), map_y, -1).astype(np.float32)
    out = cv2.remap(board, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return out


def _board_object_points(pattern_size: Tuple[int, int], square_size: float) -> np.ndarray:
    cols, rows = pattern_size
    objp = np.zeros((rows * cols, 3), dtype=np.float64)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size
    return objp


def _board_pose_rvec_tvec(
    yaw_deg: float,
    pitch_deg: float,
    distance: float,
    pattern_size: Tuple[int, int],
    square_size: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Board on z=0; tvec places it in front of the camera along +Z."""
    objp = _board_object_points(pattern_size, square_size)
    center = np.array(
        [
            (pattern_size[0] - 1) * square_size / 2.0,
            (pattern_size[1] - 1) * square_size / 2.0,
            0.0,
        ]
    )
    objp = objp - center
    yaw = np.radians(yaw_deg)
    pitch = np.radians(pitch_deg)
    rvec = np.array([pitch, yaw, 0.0], dtype=np.float64)
    R, _ = cv2.Rodrigues(rvec)
    board_center_cam = R @ np.array([0.0, 0.0, distance])
    tvec = board_center_cam.reshape(3, 1)
    return objp, rvec, tvec


def project_board_to_fisheye(
    intr: FisheyeIntrinsics,
    yaw_deg: float,
    pitch_deg: float,
    distance: float,
    pattern_size: Tuple[int, int] = (8, 5),
    square_size: float = 0.04,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return (object_points Nx1x3, image_points Nx1x2) using OpenCV fisheye projection."""
    objp, rvec, tvec = _board_pose_rvec_tvec(yaw_deg, pitch_deg, distance, pattern_size, square_size)
    obj_cv = objp.reshape(-1, 1, 3).astype(np.float64)
    img_pts, _ = cv2.fisheye.projectPoints(obj_cv, rvec, tvec, intr.K, intr.D)
    obj_out = objp.astype(np.float64).reshape(-1, 1, 3)
    return obj_out, img_pts.astype(np.float64)


def generate_calibration_observations(
    intr: FisheyeIntrinsics,
    image_size: Tuple[int, int],
    num_poses: int = 25,
    seed: int = 0,
    pattern_size: Tuple[int, int] = (8, 5),
) -> Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray]]:
    """Synthetic fisheye frames + OpenCV-style obj/img point lists (for calibration validation)."""
    rng = np.random.default_rng(seed)
    w, h = image_size
    obj_all: List[np.ndarray] = []
    img_all: List[np.ndarray] = []
    frames: List[np.ndarray] = []

    for _ in range(num_poses):
        yaw = float(rng.uniform(-45, 45))
        pitch = float(rng.uniform(-30, 30))
        dist = float(rng.uniform(0.35, 0.9))
        objp, imgp = project_board_to_fisheye(intr, yaw, pitch, dist, pattern_size)
        u = imgp[:, 0, 0]
        v = imgp[:, 0, 1]
        if np.any(u < 8) or np.any(u > w - 8) or np.any(v < 8) or np.any(v > h - 8):
            continue
        rect = rotate_board_3d(make_checkerboard(), yaw, pitch, dist, w, h)
        fish = warp_rectilinear_to_fisheye(rect, intr, fov_src_deg=80.0)
        frames.append(fish)
        obj_all.append(objp)
        img_all.append(imgp)

    return frames, obj_all, img_all


def generate_synthetic_dataset(
    intr: FisheyeIntrinsics,
    out_size: Tuple[int, int] = (1280, 720),
    num_poses: int = 24,
    seed: int = 0,
) -> Tuple[List[np.ndarray], List[np.ndarray], FisheyeIntrinsics]:
    """
    Returns list of fisheye images, corresponding ground-truth rectilinear views,
    and the intrinsics used (ground truth).
    """
    rng = np.random.default_rng(seed)
    w, h = out_size
    board = make_checkerboard()
    fisheye_frames: List[np.ndarray] = []
    rect_frames: List[np.ndarray] = []

    for i in range(num_poses):
        yaw = rng.uniform(-35, 35)
        pitch = rng.uniform(-25, 25)
        dist = rng.uniform(2.5, 5.0)
        rect = rotate_board_3d(board, yaw, pitch, dist, w, h)
        fish = warp_rectilinear_to_fisheye(rect, intr)
        fisheye_frames.append(fish)
        rect_frames.append(rect)

    return fisheye_frames, rect_frames, intr


def inject_distortion_params(
    width: int,
    height: int,
    fov_deg: float = 190.0,
    k1: float = -0.02,
    k2: float = 0.001,
    k3: float = 0.0,
    k4: float = 0.0,
) -> FisheyeIntrinsics:
    base = default_intrinsics_from_fov(width, height, fov_deg)
    return FisheyeIntrinsics(base.fx, base.fy, base.cx, base.cy, k1, k2, k3, k4)


def default_intrinsics_from_fov(width: int, height: int, fov_deg: float) -> FisheyeIntrinsics:
    from fisheye.model import default_intrinsics

    return default_intrinsics(width, height, fov_deg)
