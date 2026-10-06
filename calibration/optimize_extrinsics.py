#!/usr/bin/env python3
"""Refine yaw_offset_deg per camera by minimizing overlap SSD on projected panels."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

# Import projection helpers from synthetic_calibrate
from synthetic_calibrate import ROLES, undistort_points_norm  # noqa: E402


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def camera_to_vehicle(yaw, pitch, roll=0.0):
    rcv = np.array([[0, 0, 1], [1, 0, 0], [0, -1, 0]], dtype=float)
    return rot_z(math.radians(yaw)) @ rot_y(math.radians(-pitch)) @ rot_x(math.radians(roll)) @ rcv


def ray_to_pixel(x, y, z, cam: dict) -> tuple[float, float]:
    model = cam["model"]
    fx, fy, cx, cy = cam["fx"], cam["fy"], cam["cx"], cam["cy"]
    k = np.array([cam["k1"], cam["k2"], cam["k3"], cam["k4"]])
    r = math.hypot(x, y)
    theta = math.atan2(r, z)
    if model == "KANNALA_BRANDT":
        t2 = theta * theta
        poly = 1.0 + k[0] * t2 + k[1] * t2**2 + k[2] * t2**3 + k[3] * t2**4
        theta_d = theta * poly
    elif model == "EQUISOLID":
        theta_d = 2.0 * math.sin(theta / 2.0)
    elif model == "STEROGRAPHIC":
        theta_d = 2.0 * math.tan(theta / 2.0)
    else:
        theta_d = theta
    s = theta_d / r if r > 1e-10 else 0.0
    return fx * x * s + cx, fy * y * s + cy


def build_panel(frame, cam, pano, yaw_off=0.0, pitch_off=0.0):
    pw, ph = pano["panel_width"], pano["panel_height"]
    h, w = frame.shape[:2]
    yaw = cam["yaw_deg"] + yaw_off + cam.get("yaw_offset_deg", 0)
    pitch = cam["pitch_deg"] + pitch_off + cam.get("pitch_offset_deg", 0)
    R = camera_to_vehicle(yaw, pitch, cam["roll_deg"])
    Rt = R.T
    map_x = np.full((ph, pw), -1, np.float32)
    map_y = np.full((ph, pw), -1, np.float32)
    max_inc = math.radians(pano["max_incidence_deg"])
    yaw0 = math.radians(yaw)
    yaw_span = math.radians(pano["panel_yaw_deg"])
    pitch_span = math.radians(pano["panel_pitch_deg"])
    horizon_y = pano["horizon_fraction"] * ph
    for v in range(ph):
        phi = (horizon_y - v) / ph * pitch_span
        cphi, sphi = math.cos(phi), math.sin(phi)
        for u in range(pw):
            theta = yaw0 + ((u + 0.5) / pw - 0.5) * yaw_span
            xv = cphi * math.cos(theta)
            yv = cphi * math.sin(theta)
            zv = sphi
            ray_c = Rt @ np.array([xv, yv, zv])
            xc, yc, zc = ray_c
            if zc <= 1e-4:
                continue
            inc = math.atan2(math.hypot(xc, yc), zc)
            if inc > max_inc:
                continue
            su, sv = ray_to_pixel(xc, yc, zc, cam)
            if 1 <= su < w - 1 and 1 <= sv < h - 1:
                map_x[v, u] = su
                map_y[v, u] = sv
    return cv2.remap(frame, map_x, map_y, cv2.INTER_LINEAR)


def overlap_ssd(a: np.ndarray, b: np.ndarray, overlap: int) -> float:
    ga = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY).astype(np.float32)
    gb = cv2.cvtColor(b, cv2.COLOR_BGR2GRAY).astype(np.float32)
    ra, lb = ga[:, -overlap:], gb[:, :overlap]
    m = (ra > 12) & (lb > 12)
    if m.sum() < 300:
        return 1e9
    return float(np.mean((ra - lb)[m] ** 2))


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
    rig_path = root / "config" / "rig.json"
    rig = json.loads(rig_path.read_text())
    pano = rig["panorama"]
    overlap = pano["overlap_px"]
    frames = {}
    for role in ROLES:
        p = root / f"{role}_1.mp4"
        cap = cv2.VideoCapture(str(p))
        _, frames[role] = cap.read()
        cap.release()

    pairs = [("left", "front"), ("front", "right"), ("right", "rear")]
    yaw_acc = {r: rig["cameras"][r].get("yaw_offset_deg", 0.0) for r in ROLES}

    for ra, rb in pairs:
        best = (1e9, 0.0)
        for delta in np.arange(-8, 8.01, 0.25):
            cam_b = dict(rig["cameras"][rb])
            cam_b["yaw_offset_deg"] = yaw_acc[rb] + delta
            pa = build_panel(frames[ra], rig["cameras"][ra], pano, yaw_acc[ra], 0)
            pb = build_panel(frames[rb], cam_b, pano, 0, 0)
            s = overlap_ssd(pa, pb, overlap)
            if s < best[0]:
                best = (s, delta)
        yaw_acc[rb] += best[1]
        print(f"{ra}-{rb}: add yaw_offset on {rb} by {best[1]:+.2f} (ssd={best[0]:.1f})")

    for role in ROLES:
        rig["cameras"][role]["yaw_offset_deg"] = round(yaw_acc[role], 2)
    rig_path.write_text(json.dumps(rig, indent=2) + "\n")
    print("updated", rig_path)


if __name__ == "__main__":
    main()
