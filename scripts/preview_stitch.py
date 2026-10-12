#!/usr/bin/env python3
import json
import math
from pathlib import Path

import cv2
import numpy as np

# Mirror VideoStreamingServer panel + equidistant lens from calib JSON.

PANEL_W, PANEL_H = 640, 400
PANEL_YAW_DEG = 118.0
PANEL_PITCH_DEG = 72.0
HORIZON_FRAC = 0.40
MAX_INC_DEG = 78.0
ORDER = ["left", "front", "right", "rear"]
CAM_YAW = [-90.0, 0.0, 90.0, 180.0]
CAM_PITCH = [-14.0, -12.0, -14.0, -21.0]


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


def build_maps(frame_shape, intr, yaw, pitch, yaw_off, pitch_off):
    h, w = frame_shape[:2]
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]
    R = camera_to_vehicle(yaw + yaw_off, pitch + pitch_off)
    Rt = R.T
    map_x = np.full((PANEL_H, PANEL_W), -1, dtype=np.float32)
    map_y = np.full((PANEL_H, PANEL_W), -1, dtype=np.float32)
    max_inc = math.radians(MAX_INC_DEG)
    yaw0 = math.radians(yaw + yaw_off)
    yaw_span = math.radians(PANEL_YAW_DEG)
    pitch_span = math.radians(PANEL_PITCH_DEG)
    horizon_y = HORIZON_FRAC * PANEL_H
    for v in range(PANEL_H):
        phi = (horizon_y - v) / PANEL_H * pitch_span
        cphi, sphi = math.cos(phi), math.sin(phi)
        for u in range(PANEL_W):
            theta = yaw0 + ((u + 0.5) / PANEL_W - 0.5) * yaw_span
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
            az = math.atan2(yc, xc)
            r = fx * inc
            su = cx + r * math.cos(az)
            sv = cy + r * math.sin(az)
            if 1 <= su < w - 1 and 1 <= sv < h - 1:
                map_x[v, u] = su
                map_y[v, u] = sv
    return map_x, map_y


def main():
    root = Path(".")
    panels = []
    for i, role in enumerate(ORDER):
        intr = json.loads((root / "calib" / f"{role}_fisheye.json").read_text())
        cap = cv2.VideoCapture(str(root / f"{role}_1.mp4"))
        _, frame = cap.read()
        cap.release()
        mx, my = build_maps(
            frame.shape, intr, CAM_YAW[i], CAM_PITCH[i],
            intr.get("yaw_offset_deg", 0), intr.get("pitch_offset_deg", 0))
        panel = cv2.remap(frame, mx, my, cv2.INTER_LINEAR)
        panels.append(panel)
    overlap = 34
    W = PANEL_W
    pano_w = W + (len(panels) - 1) * (W - overlap)
    pano = np.zeros((PANEL_H, pano_w, 3), dtype=np.uint8)
    for i, p in enumerate(panels):
        x0 = i * (W - overlap)
        pano[:, x0:x0 + W] = p
    out = root / "preview_pano.jpg"
    cv2.imwrite(str(out), pano)
    print("wrote", out)


if __name__ == "__main__":
    main()
