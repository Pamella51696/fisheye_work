"""Checkerboard-based fisheye calibration via OpenCV."""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from fisheye.model import FisheyeIntrinsics, default_intrinsics


def _fisheye_calib_flags(use_guess: bool = True) -> int:
    mod = getattr(cv2, "fisheye", cv2)
    flags = mod.CALIB_RECOMPUTE_EXTRINSIC | mod.CALIB_FIX_SKEW
    if use_guess:
        flags |= mod.CALIB_USE_INTRINSIC_GUESS
    return int(flags)


def _opencv_objp(cols: int, rows: int, square_size: float) -> np.ndarray:
    objp = np.zeros((rows * cols, 1, 3), np.float32)
    objp[:, 0, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size
    return objp


def find_checkerboard_corners(
    image: np.ndarray,
    pattern_size: Tuple[int, int],
) -> Optional[np.ndarray]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags)
    if not found:
        found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not found:
        return None
    corners = cv2.cornerSubPix(
        gray,
        corners,
        (11, 11),
        (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
    )
    return corners


def calibrate_from_checkerboards(
    images: Sequence[np.ndarray],
    pattern_size: Tuple[int, int],
    square_size: float = 1.0,
    intrinsics_guess: Optional[FisheyeIntrinsics] = None,
) -> Tuple[FisheyeIntrinsics, float, List[np.ndarray]]:
    """
    Run cv2.fisheye.calibrate. pattern_size is inner corners (cols, rows).
    Returns (intrinsics, rms_reprojection_error, per-image rvecs/tvecs list omitted as rms only).
    """
    cols, rows = pattern_size
    objp = _opencv_objp(cols, rows, square_size)
    obj_points: List[np.ndarray] = []
    img_points: List[np.ndarray] = []
    h, w = images[0].shape[:2]

    for im in images:
        corners = find_checkerboard_corners(im, pattern_size)
        if corners is None:
            continue
        obj_points.append(objp)
        img_points.append(corners)

    if len(obj_points) < 3:
        raise ValueError(f"Need at least 3 valid checkerboard views, got {len(obj_points)}")

    K = np.eye(3, dtype=np.float64)
    D = np.zeros((4, 1), dtype=np.float64)
    if intrinsics_guess is not None:
        K = intrinsics_guess.K.copy()
        D = intrinsics_guess.D.copy()
    else:
        guess = default_intrinsics(w, h)
        K = guess.K.copy()
        D = guess.D.copy()

    flags = _fisheye_calib_flags(use_guess=True)
    rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        obj_points,
        img_points,
        (w, h),
        K,
        D,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
    )
    intr = FisheyeIntrinsics(
        float(K[0, 0]),
        float(K[1, 1]),
        float(K[0, 2]),
        float(K[1, 2]),
        float(D[0, 0]),
        float(D[1, 0]),
        float(D[2, 0]),
        float(D[3, 0]),
    )
    return intr, float(rms), list(rvecs)


def calibrate_from_correspondences(
    object_points: Sequence[np.ndarray],
    image_points: Sequence[np.ndarray],
    image_size: Tuple[int, int],
    intrinsics_guess: Optional[FisheyeIntrinsics] = None,
) -> Tuple[FisheyeIntrinsics, float]:
    """Calibrate when 3D–2D correspondences are already known (e.g. synthetic GT)."""
    w, h = image_size
    K = np.eye(3, dtype=np.float64)
    D = np.zeros((4, 1), dtype=np.float64)
    if intrinsics_guess is not None:
        K = intrinsics_guess.K.copy()
        D = intrinsics_guess.D.copy()
    else:
        guess = default_intrinsics(w, h)
        K = guess.K.copy()
        D = guess.D.copy()
    flags = _fisheye_calib_flags(use_guess=True)
    obj = [o.astype(np.float64) for o in object_points]
    img = [i.astype(np.float64) for i in image_points]
    rms, K, D, _, _ = cv2.fisheye.calibrate(
        obj,
        img,
        (w, h),
        K,
        D,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
    )
    intr = FisheyeIntrinsics(
        float(K[0, 0]),
        float(K[1, 1]),
        float(K[0, 2]),
        float(K[1, 2]),
        float(D[0, 0]),
        float(D[1, 0]),
        float(D[2, 0]),
        float(D[3, 0]),
    )
    return intr, float(rms)


def generate_checkerboard_views(
    fisheye_images: Sequence[np.ndarray],
    pattern_size: Tuple[int, int] = (8, 5),
) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """Filter images that contain a detectable checkerboard; return (images, corners)."""
    valid_imgs: List[np.ndarray] = []
    corners_list: List[np.ndarray] = []
    for im in fisheye_images:
        c = find_checkerboard_corners(im, pattern_size)
        if c is not None:
            valid_imgs.append(im)
            corners_list.append(c)
    return valid_imgs, corners_list


def parameter_error(estimated: FisheyeIntrinsics, ground_truth: FisheyeIntrinsics) -> dict:
    return {
        "fx_rel": abs(estimated.fx - ground_truth.fx) / ground_truth.fx,
        "fy_rel": abs(estimated.fy - ground_truth.fy) / ground_truth.fy,
        "cx_abs": abs(estimated.cx - ground_truth.cx),
        "cy_abs": abs(estimated.cy - ground_truth.cy),
        "k1_abs": abs(estimated.k1 - ground_truth.k1),
        "k2_abs": abs(estimated.k2 - ground_truth.k2),
        "k3_abs": abs(estimated.k3 - ground_truth.k3),
        "k4_abs": abs(estimated.k4 - ground_truth.k4),
    }
