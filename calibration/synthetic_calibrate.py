#!/usr/bin/env python3
"""
Synthetic (AI-video) intrinsic calibration: circle fit + straight-line model selection.
Writes per-camera intrinsics into config/rig.json (parametric; replace for real checkerboard later).
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Callable, List, Tuple

import cv2
import numpy as np

ROLES = ["left", "front", "right", "rear"]
DEFAULT_EXTRINSICS = {
    "left": {"yaw_deg": -90.0, "pitch_deg": -14.0, "roll_deg": 0.0},
    "front": {"yaw_deg": 0.0, "pitch_deg": -12.0, "roll_deg": 0.0},
    "right": {"yaw_deg": 90.0, "pitch_deg": -14.0, "roll_deg": 0.0},
    "rear": {"yaw_deg": 180.0, "pitch_deg": -21.0, "roll_deg": 0.0},
}


@dataclass
class Intrinsics:
    model: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    k1: float = 0.0
    k2: float = 0.0
    k3: float = 0.0
    k4: float = 0.0
    line_error_px: float = 0.0


def read_frames(video: Path, max_frames: int = 120) -> List[np.ndarray]:
    cap = cv2.VideoCapture(str(video))
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    step = max(1, n // max_frames) if n > 0 else 1
    frames = []
    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % step == 0:
            frames.append(frame)
        idx += 1
        if len(frames) >= max_frames:
            break
    cap.release()
    return frames


def fit_disc(gray: np.ndarray) -> Tuple[float, float, float]:
    mask = (gray > 12).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((21, 21), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        h, w = gray.shape
        return w / 2.0, h / 2.0, min(w, h) / 2.0
    c = max(cnts, key=cv2.contourArea)
    (cx, cy), r = cv2.minEnclosingCircle(c)
    return float(cx), float(cy), float(r)


def theta_d(theta: np.ndarray, k: np.ndarray) -> np.ndarray:
    t2 = theta * theta
    poly = 1.0 + k[0] * t2 + k[1] * t2**2 + k[2] * t2**3 + k[3] * t2**4
    return theta * poly


def undistort_points_norm(
    u: np.ndarray, v: np.ndarray, fx: float, fy: float, cx: float, cy: float,
    model: str, k: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    xd = (u - cx) / fx
    yd = (v - cy) / fy
    rd = np.hypot(xd, yd)
    theta = rd.copy()
    if model == "KANNALA_BRANDT":
        for _ in range(10):
            t2 = theta * theta
            poly = 1.0 + k[0] * t2 + k[1] * t2**2 + k[2] * t2**3 + k[3] * t2**4
            fval = theta * poly - rd
            dpoly = k[0] + 2 * k[1] * t2 + 3 * k[2] * t2**2 + 4 * k[3] * t2**3
            df = poly + theta * dpoly
            theta = theta - fval / np.maximum(df, 1e-8)
    elif model == "EQUISOLID":
        theta = 2.0 * np.arcsin(np.clip(rd / 2.0, -1.0, 1.0))
    elif model == "STEROGRAPHIC":
        theta = 2.0 * np.arctan(rd / 2.0)
    else:  # EQUIDISTANT
        theta = rd
    scale = np.where(rd > 1e-8, np.tan(theta) / rd, 1.0)
    return xd * scale, yd * scale


def collect_line_points(frames: List[np.ndarray], max_lines: int = 40) -> np.ndarray:
    lsd = cv2.createLineSegmentDetector(0)
    pts = []
    for frame in frames[: min(30, len(frames))]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        lines = lsd.detect(gray)[0]
        if lines is None:
            continue
        for line in lines[:max_lines]:
            seg = np.asarray(line).reshape(-1)
            x1, y1, x2, y2 = seg[0], seg[1], seg[2], seg[3]
            length = math.hypot(x2 - x1, y2 - y1)
            if length < 40:
                continue
            n = int(min(20, length / 8))
            for t in np.linspace(0, 1, n):
                pts.append([x1 + t * (x2 - x1), y1 + t * (y2 - y1)])
    if not pts:
        return np.zeros((0, 2), dtype=np.float64)
    return np.array(pts, dtype=np.float64)


def line_fit_error(xn: np.ndarray, yn: np.ndarray) -> float:
    if len(xn) < 8:
        return 1e6
    pts = np.stack([xn, yn], axis=1)
    mean = pts.mean(axis=0)
    cov = np.cov(pts.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    normal = eigvecs[:, 0]
    d = np.abs((pts - mean) @ normal)
    return float(np.mean(d))


def eval_model(
    pts: np.ndarray, cx: float, cy: float, r_disc: float,
    model: str, k: np.ndarray,
) -> Tuple[float, float]:
    theta_edge = math.radians(95.0)  # 190° full FOV at disc edge
    f = r_disc / theta_edge
    xn, yn = undistort_points_norm(pts[:, 0], pts[:, 1], f, f, cx, cy, model, k)
    err = line_fit_error(xn, yn)
    return f, err


def calibrate_camera(role: str, video: Path) -> Intrinsics:
    frames = read_frames(video)
    if not frames:
        raise RuntimeError(f"no frames in {video}")
    h, w = frames[0].shape[:2]
    gray = cv2.cvtColor(frames[len(frames) // 2], cv2.COLOR_BGR2GRAY)
    cx, cy, r_disc = fit_disc(gray)
    pts = collect_line_points(frames)
    models = ["EQUIDISTANT", "EQUISOLID", "STEROGRAPHIC", "KANNALA_BRANDT"]
    best = (1e9, "EQUIDISTANT", 0.0, np.zeros(4))
    for model in models:
        k = np.zeros(4)
        if model == "KANNALA_BRANDT":
            for k1 in np.linspace(-0.15, 0.15, 7):
                k[0] = k1
                f, err = eval_model(pts, cx, cy, r_disc, model, k)
                if err < best[0]:
                    best = (err, model, f, k.copy())
        else:
            f, err = eval_model(pts, cx, cy, r_disc, model, k)
            if err < best[0]:
                best = (err, model, f, k.copy())
    err, model, f, k = best
    print(f"{role}: model={model} fx={f:.2f} cx={cx:.1f} cy={cy:.1f} line_err={err:.4f}")
    return Intrinsics(model, w, h, f, f, cx, cy, k[0], k[1], k[2], k[3], err)


def build_rig(root: Path, intrinsics: dict) -> dict:
    cameras = {}
    for role in ROLES:
        intr = intrinsics[role]
        ext = DEFAULT_EXTRINSICS[role]
        cameras[role] = {
            **asdict(intr),
            **ext,
            "yaw_offset_deg": 0.0,
            "pitch_offset_deg": 0.0,
            "rect_balance": 0.0,
            "rect_output_fov_deg": 90.0,
        }
    return {
        "calibration_mode": "synthetic",
        "description": "Provisional AI-video fit; replace with checkerboard/Charuco for real cameras.",
        "camera_order": ROLES,
        "panorama": {
            "panel_width": 640,
            "panel_height": 400,
            "panel_yaw_deg": 118.0,
            "panel_pitch_deg": 72.0,
            "horizon_fraction": 0.40,
            "max_incidence_deg": 78.0,
            "overlap_px": 34,
            "edge_feather_px": 100,
        },
        "cameras": cameras,
    }


def main() -> None:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
    intrinsics = {}
    for role in ROLES:
        video = root / f"{role}_1.mp4"
        if not video.is_file():
            video = root / f"{role}.mp4"
        intrinsics[role] = calibrate_camera(role, video)
    rig = build_rig(root, intrinsics)
    out = root / "config" / "rig.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(rig, indent=2) + "\n")
    print("wrote", out)


if __name__ == "__main__":
    main()
