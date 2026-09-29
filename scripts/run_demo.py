#!/usr/bin/env python3
"""Demo: synthetic fisheye -> calibrate -> undistort -> surround stitch."""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from fisheye.calibrate import calibrate_from_correspondences, parameter_error
from fisheye.stitch import CameraExtrinsics, SurroundStitcher
from fisheye.synthetic import (
    generate_calibration_observations,
    generate_synthetic_dataset,
    inject_distortion_params,
)
from fisheye.undistort import UndistortLUT
from fisheye.validate import psnr, verify_forward_inverse


def main() -> None:
    p = argparse.ArgumentParser(description="Fisheye calibration + stitching demo")
    p.add_argument("--width", type=int, default=960)
    p.add_argument("--height", type=int, default=540)
    p.add_argument("--out", type=Path, default=ROOT / "output")
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    w, h = args.width, args.height
    gt = inject_distortion_params(w, h, fov_deg=188.0, k1=-0.02, k2=0.0009)
    ang, pix = verify_forward_inverse(gt, w, h)
    print(f"Forward/inverse check: median ang={ang:.4f}° pix={pix:.4f}")

    fish_list, rect_list, _ = generate_synthetic_dataset(gt, (w, h), num_poses=30, seed=1)
    cv2.imwrite(str(args.out / "sample_fisheye.png"), fish_list[0])
    cv2.imwrite(str(args.out / "sample_rect_gt.png"), rect_list[0])

    _, obj_pts, img_pts = generate_calibration_observations(gt, (w, h), num_poses=28, seed=2)
    est, rms = calibrate_from_correspondences(obj_pts, img_pts, (w, h))
    print(f"Calibration RMS: {rms:.4f} px")
    print("Parameter error vs ground truth:", parameter_error(est, gt))

    lut_gt = UndistortLUT.create(w, h, gt, fov_out_deg=90.0)
    lut_est = UndistortLUT.create(w, h, est, fov_out_deg=90.0)
    und_gt = lut_gt.remap(fish_list[0])
    und_est = lut_est.remap(fish_list[0])
    cv2.imwrite(str(args.out / "undistort_gt_intrinsics.png"), und_gt)
    cv2.imwrite(str(args.out / "undistort_est_intrinsics.png"), und_est)
    print(f"Undistort PSNR (gt vs est intrinsics on same frame): {psnr(und_gt, und_est):.2f} dB")

    intr = [est, est, est, est]
    ext = [
        CameraExtrinsics(yaw_deg=-90, pitch_deg=-14),
        CameraExtrinsics(yaw_deg=0, pitch_deg=-12),
        CameraExtrinsics(yaw_deg=90, pitch_deg=-14),
        CameraExtrinsics(yaw_deg=180, pitch_deg=-18),
    ]
    stitcher = SurroundStitcher(intr, ext, bev_w=600, bev_h=600, meters_per_pixel=0.025)
    frames = [fish_list[i % len(fish_list)] for i in range(4)]
    bev = stitcher.stitch_frames(frames)
    pano = stitcher.stitch_panorama_horizontal(frames, out_w=1920, out_h=400)
    cv2.imwrite(str(args.out / "bev_stitch.png"), bev)
    cv2.imwrite(str(args.out / "panorama_stitch.png"), pano)
    print(f"Wrote artifacts to {args.out}")


if __name__ == "__main__":
    main()
