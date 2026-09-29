"""Fisheye calibration, undistortion (LUT), and multi-camera stitching."""

from fisheye.model import FisheyeIntrinsics, apply_distortion, incidence_from_pixel
from fisheye.undistort import UndistortLUT, build_remap_maps
from fisheye.calibrate import calibrate_from_checkerboards, generate_checkerboard_views
from fisheye.stitch import SurroundStitcher, CameraExtrinsics

__all__ = [
    "FisheyeIntrinsics",
    "apply_distortion",
    "incidence_from_pixel",
    "UndistortLUT",
    "build_remap_maps",
    "calibrate_from_checkerboards",
    "generate_checkerboard_views",
    "SurroundStitcher",
    "CameraExtrinsics",
]
