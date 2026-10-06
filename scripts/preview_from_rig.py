#!/usr/bin/env python3
"""Preview panorama using config/rig.json (spherical rays, matches Java PanoramaMapper)."""

import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROLES = ["left", "front", "right", "rear"]


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


def ray_to_pixel(x, y, z, cam):
    zoom = max(0.5, cam.get("focal_zoom_out", 1.0))
    fx, fy = cam["fx"] / zoom, cam["fy"] / zoom
    r = math.hypot(x, y)
    theta = math.atan2(r, z)
    s = theta / r if r > 1e-10 else 0.0
    return fx * x * s + cam["cx"], fy * y * s + cam["cy"]


def build_panel(frame, cam, pano):
    h, w = frame.shape[:2]
    pw, ph = int(pano["panel_width"]), int(pano["panel_height"])
    yaw = cam["yaw_deg"] + cam.get("yaw_offset_deg", 0)
    pitch = cam["pitch_deg"] + cam.get("pitch_offset_deg", 0)
    R = camera_to_vehicle(yaw, pitch, cam["roll_deg"])
    Rt = R.T
    max_inc = math.radians(pano["max_incidence_deg"])
    yaw0 = math.radians(yaw)
    yaw_span = math.radians(pano["panel_yaw_deg"])
    pitch_span = math.radians(pano["panel_pitch_deg"])
    horizon_y = pano["horizon_fraction"] * ph
    map_x = np.zeros((ph, pw), np.float32)
    map_y = np.zeros((ph, pw), np.float32)
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
            if math.atan2(math.hypot(xc, yc), zc) > max_inc:
                continue
            su, sv = ray_to_pixel(xc, yc, zc, cam)
            if 1 <= su < w - 1 and 1 <= sv < h - 1:
                map_x[v, u] = su
                map_y[v, u] = sv
    return cv2.remap(frame, map_x, map_y, cv2.INTER_LINEAR)


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
    rig = json.loads((root / "config" / "rig.json").read_text())
    pano = rig["panorama"]
    panels = []
    for role in rig["camera_order"]:
        cap = cv2.VideoCapture(str(root / f"{role}_1.mp4"))
        _, frame = cap.read()
        cap.release()
        panels.append(build_panel(frame, rig["cameras"][role], pano))
    overlap = pano["overlap_px"]
    w = pano["panel_width"]
    pano_w = w + (len(panels) - 1) * (w - overlap)
    out = np.zeros((pano["panel_height"], pano_w, 3), dtype=np.uint8)
    for i, p in enumerate(panels):
        x0 = i * (w - overlap)
        out[:, x0:x0 + w] = p
    path = root / "preview_pano.jpg"
    cv2.imwrite(str(path), out)
    print("wrote", path)


if __name__ == "__main__":
    main()
