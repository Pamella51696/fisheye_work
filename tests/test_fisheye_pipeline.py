"""End-to-end tests: model, synthetic data, calibration, undistort, stitch."""

import numpy as np
import pytest

from fisheye.calibrate import calibrate_from_correspondences, parameter_error
from fisheye.model import FisheyeIntrinsics, apply_distortion, default_intrinsics
from fisheye.stitch import CameraExtrinsics, SurroundStitcher
from fisheye.synthetic import (
    generate_synthetic_dataset,
    inject_distortion_params,
    warp_rectilinear_to_fisheye,
)
from fisheye.undistort import UndistortLUT, build_remap_maps
from fisheye.validate import psnr, ssim_gray, verify_forward_inverse


def test_forward_inverse_median_error_small():
    intr = inject_distortion_params(1280, 720, k1=-0.015, k2=0.0008)
    ang_med, pix_med = verify_forward_inverse(intr, 1280, 720)
    assert ang_med < 0.05
    assert pix_med < 0.5


def test_undistort_roundtrip_synthetic():
    w, h = 640, 480
    gt = inject_distortion_params(w, h, fov_deg=185.0, k1=-0.01, k2=0.0)
    rect = np.zeros((h, w, 3), dtype=np.uint8)
    rect[:, :] = (40, 120, 200)
    cv2 = pytest.importorskip("cv2")
    for i in range(0, w, 40):
        cv2.line(rect, (i, 0), (i, h - 1), (255, 255, 255), 1)
    for j in range(0, h, 40):
        cv2.line(rect, (0, j), (w - 1, j), (255, 255, 255), 1)

    fish = warp_rectilinear_to_fisheye(rect, gt, fov_src_deg=85.0)
    lut = UndistortLUT.create(w, h, gt, fov_out_deg=85.0)
    recovered = lut.remap(fish)

    # Center crop should resemble original grid
    margin = w // 8
    crop_gt = rect[margin : h - margin, margin : w - margin]
    crop_rec = recovered[margin : h - margin, margin : w - margin]
    assert psnr(crop_gt, crop_rec) > 18.0
    assert ssim_gray(crop_gt, crop_rec) > 0.45


def test_calibration_recovers_injected_params():
    w, h = 960, 540
    gt = inject_distortion_params(w, h, fov_deg=180.0, k1=-0.018, k2=0.0012)
    from fisheye.synthetic import generate_calibration_observations

    _, obj_pts, img_pts = generate_calibration_observations(gt, (w, h), num_poses=32, seed=7)
    est, rms = calibrate_from_correspondences(obj_pts, img_pts, (w, h))
    assert rms < 1.0
    errs = parameter_error(est, gt)
    assert errs["fx_rel"] < 0.08
    assert errs["k1_abs"] < 0.01


def test_lut_matches_explicit_inverse_map():
    intr = default_intrinsics(800, 600, 190.0)
    mx, my = build_remap_maps(800, 600, intr, fov_out_deg=100.0)
    lut = UndistortLUT(mx, my, 100.0, (800, 600))
    mx2, my2 = build_remap_maps(800, 600, intr, fov_out_deg=100.0)
    assert lut.mean_map_error_vs(mx2, my2) < 1e-4


def test_surround_stitcher_runs():
    cv2 = pytest.importorskip("cv2")
    w, h = 320, 240
    intr = [default_intrinsics(w, h) for _ in range(4)]
    ext = [
        CameraExtrinsics(yaw_deg=-90),
        CameraExtrinsics(yaw_deg=0),
        CameraExtrinsics(yaw_deg=90),
        CameraExtrinsics(yaw_deg=180),
    ]
    stitcher = SurroundStitcher(intr, ext, bev_w=200, bev_h=200, meters_per_pixel=0.05)
    frames = []
    for yaw in [-90, 0, 90, 180]:
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[:, :] = (max(0, 30 + yaw), 100, 150)
        cv2.putText(img, str(yaw), (40, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        frames.append(img)
    bev = stitcher.stitch_frames(frames, feather_px=20)
    assert bev.shape == (200, 200, 3)
    assert bev.mean() > 5
