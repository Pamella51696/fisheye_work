#!/usr/bin/env python3
"""Fast per-camera yaw trim from overlap strips (equidistant + calib intrinsics)."""

import json
import math
from pathlib import Path

import cv2
import numpy as np

PANEL_W, PANEL_H = 640, 400
PANEL_YAW_DEG = 118.0
PANEL_PITCH_DEG = 72.0
HORIZON_FRAC = 0.40
MAX_INC_DEG = 78.0
ORDER = ["left", "front", "right", "rear"]
CAM_YAW = {"left": -90.0, "front": 0.0, "right": 90.0, "rear": 180.0}
CAM_PITCH = {"left": -14.0, "front": -12.0, "right": -14.0, "rear": -21.0}
OVERLAP = 80


def camera_to_vehicle(yaw, pitch, roll=0.0):
    def rot_z(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    def rot_y(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    def rot_x(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    rcv = np.array([[0, 0, 1], [1, 0, 0], [0, -1, 0]], dtype=float)
    return rot_z(math.radians(yaw)) @ rot_y(math.radians(-pitch)) @ rot_x(math.radians(roll)) @ rcv


def build_maps(frame_shape, intr, yaw, pitch, yaw_off=0.0, pitch_off=0.0):
    h, w = frame_shape[:2]
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]
    R = camera_to_vehicle(yaw + yaw_off, pitch + pitch_off)
    Rt = R.T
    map_x = np.zeros((PANEL_H, PANEL_W), np.float32)
    map_y = np.zeros((PANEL_H, PANEL_W), np.float32)
    max_inc = math.radians(MAX_INC_DEG)
    yaw0 = math.radians(yaw + yaw_off)
    yaw_span = math.radians(PANEL_YAW_DEG)
    pitch_span = math.radians(PANEL_PITCH_DEG)
    horizon_y = HORIZON_FRAC * PANEL_H
    us = np.arange(PANEL_W, dtype=np.float64) + 0.5
    vs = np.arange(PANEL_H, dtype=np.float64)
    theta_u = yaw0 + (us / PANEL_W - 0.5) * yaw_span
    for v in range(PANEL_H):
        phi = (horizon_y - v) / PANEL_H * pitch_span
        cphi, sphi = math.cos(phi), math.sin(phi)
        xv = cphi * np.cos(theta_u)
        yv = cphi * np.sin(theta_u)
        zv = np.full_like(xv, sphi)
        rays_v = np.stack([xv, yv, zv], axis=0)
        rays_c = Rt @ rays_v
        xc, yc, zc = rays_c[0], rays_c[1], rays_c[2]
        valid = zc > 1e-4
        inc = np.arctan2(np.hypot(xc, yc), zc)
        valid &= inc <= max_inc
        az = np.arctan2(yc, xc)
        r = fx * inc
        su = np.where(valid, cx + r * np.cos(az), -1.0)
        sv = np.where(valid, cy + r * np.sin(az), -1.0)
        valid &= (su >= 1) & (sv >= 1) & (su < w - 1) & (sv < h - 1)
        map_x[v, :] = np.where(valid, su, -1).astype(np.float32)
        map_y[v, :] = np.where(valid, sv, -1).astype(np.float32)
    return map_x, map_y


def project(frame, maps):
    return cv2.remap(frame, maps[0], maps[1], cv2.INTER_LINEAR, borderValue=(0, 0, 0))


def overlap_score(ga, gb):
    ra = ga[:, -OVERLAP:].astype(np.float32)
    lb = gb[:, :OVERLAP].astype(np.float32)
    mask = (ra > 10) & (lb > 10)
    if mask.sum() < 200:
        return 1e9
    return float(np.abs(ra - lb)[mask].mean())


def main():
    root = Path(".")
    calib = {}
    frames = {}
    for role in ORDER:
        calib[role] = json.loads((root / "calib" / f"{role}_fisheye.json").read_text())
        cap = cv2.VideoCapture(str(root / f"{role}_1.mp4"))
        _, frames[role] = cap.read()
        cap.release()

    yaw_off = {r: 0.0 for r in ORDER}
    pairs = [("left", "front"), ("front", "right"), ("right", "rear")]

    for role_a, role_b in pairs:
        best = (1e9, 0.0, 0.0)
        for da in np.arange(-6, 6.01, 0.5):
            for db in np.arange(-6, 6.01, 0.5):
                ya = yaw_off[role_a] + da
                yb = yaw_off[role_b] + db
                ma = build_maps(
                    frames[role_a].shape, calib[role_a],
                    CAM_YAW[role_a], CAM_PITCH[role_a], ya)
                mb = build_maps(
                    frames[role_b].shape, calib[role_b],
                    CAM_YAW[role_b], CAM_PITCH[role_b], yb)
                ga = cv2.cvtColor(project(frames[role_a], ma), cv2.COLOR_BGR2GRAY)
                gb = cv2.cvtColor(project(frames[role_b], mb), cv2.COLOR_BGR2GRAY)
                s = overlap_score(ga, gb)
                if s < best[0]:
                    best = (s, da, db)
        yaw_off[role_a] += best[1]
        yaw_off[role_b] += best[2]
        print(f"{role_a}-{role_b}: Δyaw {role_a}={best[1]:+.1f} {role_b}={best[2]:+.1f} err={best[0]:.2f}")

    for role in ORDER:
        path = root / "calib" / f"{role}_fisheye.json"
        calib[role]["yaw_offset_deg"] = round(yaw_off[role], 2)
        path.write_text(json.dumps(calib[role], indent=2) + "\n")
        print(role, "total yaw_offset_deg", calib[role]["yaw_offset_deg"])


if __name__ == "__main__":
    main()
