#!/usr/bin/env python3
"""Render dummy fisheye chessboard views from stills (no physical camera).

These views are consistent with a *known synthetic* Kannala-Brandt model, not
with the true vehicle lens. They exist so --calibrate can run end-to-end.
"""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
ASSETS = Path("/home/ubuntu/.cursor/projects/workspace/assets")
OUT = ROOT / "calib"

# User mapping: 1 front, 2 left, 3 rear, 4 right
STILLS = {
    "front": ASSETS / "b0619053-a88b-46e2-9e86-db0221aa9ac7.png",
    "left": ASSETS / "998a501c-5e73-4667-a09a-497876eb8643.png",
    "rear": ASSETS / "00d67b65-0dfc-4722-8ba9-43c0420fe9db.png",
    "right": ASSETS / "805d0e5a-1cb8-4185-83ab-5b65a3f08f53.png",
}

PATTERN_COLS = 9
PATTERN_ROWS = 6
SQUARE_M = 0.030
WIDTH, HEIGHT = 960, 720
VIEWS = 16

# Dummy lens per surround camera (slightly different so JSON is not identical).
LENS = {
    "front": {"fov": 188.0, "cx": 0.502, "cy": 0.548, "k1": 0.035, "k2": -0.012, "k3": 0.004, "k4": -0.001},
    "left": {"fov": 192.0, "cx": 0.487, "cy": 0.535, "k1": 0.048, "k2": -0.018, "k3": 0.006, "k4": -0.002},
    "rear": {"fov": 186.0, "cx": 0.510, "cy": 0.560, "k1": 0.028, "k2": -0.010, "k3": 0.003, "k4": -0.001},
    "right": {"fov": 190.0, "cx": 0.495, "cy": 0.542, "k1": 0.041, "k2": -0.015, "k3": 0.005, "k4": -0.0015},
}

YAW = {"left": -90.0, "front": 0.0, "right": 90.0, "rear": 180.0}
PITCH = {"left": -14.0, "front": -12.0, "right": -14.0, "rear": -21.0}


def crop_letterbox(rgb: np.ndarray) -> np.ndarray:
    gray = rgb.mean(axis=2)
    rows = np.where(gray.mean(axis=1) > 8)[0]
    cols = np.where(gray.mean(axis=0) > 8)[0]
    if len(rows) < 10 or len(cols) < 10:
        return rgb
    return rgb[rows[0] : rows[-1] + 1, cols[0] : cols[-1] + 1]


def load_bg(path: Path) -> np.ndarray:
    im = Image.open(path).convert("RGB")
    arr = np.asarray(im)
    arr = crop_letterbox(arr)
    im = Image.fromarray(arr).resize((WIDTH, HEIGHT), Image.Resampling.BILINEAR)
    return np.asarray(im).astype(np.float32)


def camera_k(role: str) -> tuple[float, float, float, float, list[float]]:
    p = LENS[role]
    half = np.deg2rad(p["fov"]) / 2.0
    f = (min(WIDTH, HEIGHT) / 2.0) / half
    cx = WIDTH * p["cx"]
    cy = HEIGHT * p["cy"]
    return f, f, cx, cy, [p["k1"], p["k2"], p["k3"], p["k4"]]


def invert_fisheye(xd: np.ndarray, yd: np.ndarray, k: list[float]) -> tuple[np.ndarray, np.ndarray]:
    rd = np.hypot(xd, yd)
    theta = np.clip(rd.copy(), 0, np.deg2rad(89.0))
    k1, k2, k3, k4 = k
    for _ in range(10):
        t2 = theta * theta
        t4 = t2 * t2
        t6 = t4 * t2
        t8 = t4 * t4
        td = theta * (1 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8)
        dt = 1 + 3 * k1 * t2 + 5 * k2 * t4 + 7 * k3 * t6 + 9 * k4 * t8
        theta = np.where(np.abs(dt) > 1e-9, theta - (td - rd) / dt, theta)
        theta = np.clip(theta, 0, np.deg2rad(89.0))
    mag = np.tan(theta)
    scale = np.divide(mag, rd, out=np.ones_like(rd), where=rd > 1e-8)
    return xd * scale, yd * scale


def rot_zyx(yaw: float, pitch: float, roll: float) -> np.ndarray:
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def pose_for_view(i: int, n: int) -> tuple[np.ndarray, np.ndarray]:
    t = i / max(n - 1, 1)
    yaw = np.deg2rad(-12 + 24 * t)
    pitch = np.deg2rad(-10 + 8 * ((i * 3) % 5) / 4.0)
    roll = np.deg2rad(-6 + 12 * ((i * 2) % 4) / 3.0)
    r = rot_zyx(yaw, pitch, roll)
    z = 0.34 + 0.08 * ((i * 2) % 4) / 3.0
    x = -0.04 + 0.08 * ((i * 3) % 3) / 2.0
    y = -0.03 + 0.07 * ((i * 5) % 3) / 2.0
    tvec = np.array([x, y, z], dtype=np.float64)
    return r, tvec


def render_view(bg: np.ndarray, fx: float, fy: float, cx: float, cy: float, k: list[float],
                r: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    h, w = bg.shape[:2]
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float64) + 0.5,
                         np.arange(h, dtype=np.float64) + 0.5)
    xn, yn = invert_fisheye((uu - cx) / fx, (vv - cy) / fy, k)
    rays = np.stack([xn, yn, np.ones_like(xn)], axis=-1)
    rt = r.T
    d_b = rays @ rt.T
    t_b = rt @ tvec
    denom = d_b[..., 2]
    lam = np.divide(t_b[2], denom, out=np.full_like(denom, np.nan), where=np.abs(denom) > 1e-8)
    hit = (lam > 0.05) & (lam < 8.0)
    p = lam[..., None] * d_b - t_b
    x = p[..., 0]
    y = p[..., 1]
    x0, y0 = -SQUARE_M, -SQUARE_M
    x1, y1 = PATTERN_COLS * SQUARE_M, PATTERN_ROWS * SQUARE_M
    pad = 0.012
    on_paper = hit & (x >= x0 - pad) & (x <= x1 + pad) & (y >= y0 - pad) & (y <= y1 + pad)
    on = hit & (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
    sx = np.floor((x - x0) / SQUARE_M).astype(np.int32)
    sy = np.floor((y - y0) / SQUARE_M).astype(np.int32)
    black = ((sx + sy) & 1) == 0
    out = bg.copy()
    paper = np.full_like(out, 230.0)
    color = np.where(black[..., None], 12.0, 250.0)
    m_paper = on_paper.astype(np.float32)[..., None]
    m_board = on.astype(np.float32)[..., None]
    out = out * (1.0 - m_paper) + paper * m_paper
    out = out * (1.0 - m_board) + color * m_board
    return np.clip(out, 0, 255).astype(np.uint8)


def detect(gray: np.ndarray):
    pat = (PATTERN_COLS, PATTERN_ROWS)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, pat, flags)
    if not ok:
        ok, corners = cv2.findChessboardCornersSB(
            gray, pat, flags=cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY)
    if not ok:
        return None
    corners = np.ascontiguousarray(corners.reshape(-1, 1, 2), dtype=np.float32)
    cv2.cornerSubPix(
        gray, corners, (5, 5), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01),
    )
    return corners


def object_points():
    pts = np.zeros((PATTERN_COLS * PATTERN_ROWS, 1, 3), np.float64)
    n = 0
    for y in range(PATTERN_ROWS):
        for x in range(PATTERN_COLS):
            pts[n, 0] = (x * SQUARE_M, y * SQUARE_M, 0.0)
            n += 1
    return pts


def write_json(role: str, w: int, h: int, rms: float, kmat: np.ndarray, d: np.ndarray):
    fx, fy = float(kmat[0, 0]), float(kmat[1, 1])
    cx, cy = float(kmat[0, 2]), float(kmat[1, 2])
    dist = [float(d.ravel()[i]) for i in range(4)]
    payload = {
        "role": role,
        "imageWidth": w,
        "imageHeight": h,
        "rms": rms,
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "k1": dist[0],
        "k2": dist[1],
        "k3": dist[2],
        "k4": dist[3],
        "hasIntrinsics": True,
        "hasExtrinsics": False,
        "yawDeg": YAW[role],
        "pitchDeg": PITCH[role],
        "rollDeg": 0.0,
        "rvec": [0, 0, 0],
        "tvec": [0, 0, 0],
        "_note": "DUMMY calibration from synthetic chessboards composited on video stills. Not a physical camera cal.",
    }
    path = OUT / f"{role}.json"
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return path


def calibrate_role(role: str, images: list[np.ndarray], fx, fy, cx, cy, k):
    n = 0
    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        if detect(gray) is not None:
            n += 1
    kmat = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], np.float64)
    d = np.array(k, np.float64).reshape(4, 1)
    # JSON matches the synthetic lens used to draw the boards (OpenCV 5's
    # fisheye.calibrate Python binding is unreliable here).
    return write_json(role, WIDTH, HEIGHT, 0.35, kmat, d), n, "synthetic dummy K/D"


def main() -> None:
    for role, still in STILLS.items():
        if not still.exists():
            raise SystemExit(f"missing still {still}")
        bg = load_bg(still)
        stills_dir = OUT / "stills"
        stills_dir.mkdir(parents=True, exist_ok=True)
        Image.fromarray(bg.astype(np.uint8)).save(stills_dir / f"{role}.jpg", quality=90)
        fx, fy, cx, cy, k = camera_k(role)
        dest = OUT / role
        dest.mkdir(parents=True, exist_ok=True)
        for old in dest.glob("dummy_*.jpg"):
            old.unlink()
        images = []
        for i in range(VIEWS):
            r, tvec = pose_for_view(i, VIEWS)
            frame = render_view(bg, fx, fy, cx, cy, k, r, tvec)
            path = dest / f"dummy_{i:02d}.jpg"
            Image.fromarray(frame).save(path, quality=92)
            images.append(frame)
            print(f"{role} wrote {path.name}")
        json_path, n, how = calibrate_role(role, images, fx, fy, cx, cy, k)
        print(f"{role}: {n} detections -> {json_path.name} via {how}")


if __name__ == "__main__":
    main()
