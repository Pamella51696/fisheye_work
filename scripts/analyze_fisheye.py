#!/usr/bin/env python3
"""Estimate equidistant fisheye intrinsics from sample MP4 frames."""

import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np


def read_frame(path: Path) -> np.ndarray:
    cap = cv2.VideoCapture(str(path))
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        raise RuntimeError(f"cannot read {path}")
    return frame


def fit_circle(gray: np.ndarray) -> tuple[float, float, float]:
    """Center and radius of bright disc (fisheye valid region)."""
    mask = (gray > 12).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        h, w = gray.shape
        return w / 2.0, h / 2.0, min(w, h) / 2.0
    c = max(cnts, key=cv2.contourArea)
    (cx, cy), r = cv2.minEnclosingCircle(c)
    return float(cx), float(cy), float(r)


def equidistant_f(radius: float, theta_edge_rad: float) -> float:
    return radius / theta_edge_rad if theta_edge_rad > 1e-6 else radius


def analyze(path: Path) -> dict:
    bgr = read_frame(path)
    h, w = bgr.shape[:2]
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    cx, cy, r = fit_circle(gray)
    # Incidence at the visible circle edge (equidistant: r = f * theta).
    theta_edge = r / equidistant_f(r, math.radians(95.0))  # seed
    f = equidistant_f(r, theta_edge)
    # Refine: assume edge pixel is at half of nominal 190° full FOV → 95° from axis.
    fov_half_deg = math.degrees(theta_edge)
    # Alternative: fit f so that circle edge = 95° (common for 190° fisheye in square frame)
    theta_190 = math.radians(190.0 / 2.0)
    f_190 = r / theta_190
    return {
        "width": w,
        "height": h,
        "cx": round(cx, 2),
        "cy": round(cy, 2),
        "image_radius_px": round(r, 2),
        "fx_fy_from_circle": round(f_190, 3),
        "fov_deg_if_edge_is_half": round(math.degrees(r / f_190) * 2, 2),
        "fov_deg_from_enclosing": round(fov_half_deg * 2, 2),
    }


def main() -> None:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
    roles = ["left", "front", "right", "rear"]
    out = {}
    for role in roles:
        p = root / f"{role}_1.mp4"
        if not p.is_file():
            print("missing", p, file=sys.stderr)
            continue
        out[role] = analyze(p)
        print(role, json.dumps(out[role], indent=2))
    calib_dir = root / "calib"
    calib_dir.mkdir(exist_ok=True)
    for role, m in out.items():
        f = m["fx_fy_from_circle"]
        cfg = {
            "projection": "EQUIDISTANT",
            "fx": f,
            "fy": f,
            "cx": m["cx"],
            "cy": m["cy"],
            "fov_deg": m["fov_deg_if_edge_is_half"],
            "k1": 0.0,
            "k2": 0.0,
            "k3": 0.0,
            "k4": 0.0,
            "yaw_offset_deg": 0.0,
            "pitch_offset_deg": 0.0,
        }
        path = calib_dir / f"{role}_fisheye.json"
        path.write_text(json.dumps(cfg, indent=2) + "\n")
        print("wrote", path)


if __name__ == "__main__":
    main()
