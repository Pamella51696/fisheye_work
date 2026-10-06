#!/usr/bin/env python3
"""Align cameras on a common ground plane; per-camera zoom-out (focal_zoom_out)."""

from __future__ import annotations

import json
import math
import sys
from copy import deepcopy
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
    cx, cy = cam["cx"], cam["cy"]
    r = math.hypot(x, y)
    theta = math.atan2(r, z)
    s = theta / r if r > 1e-10 else 0.0
    return fx * x * s + cx, fy * y * s + cy


def build_panel(frame, cam, pano, scale=1.0):
    h, w = frame.shape[:2]
    pw = max(160, int(pano["panel_width"] * scale))
    ph = max(100, int(pano["panel_height"] * scale))
    yaw = cam["yaw_deg"] + cam.get("yaw_offset_deg", 0)
    pitch = cam["pitch_deg"] + cam.get("pitch_offset_deg", 0)
    R = camera_to_vehicle(yaw, pitch, cam["roll_deg"])
    Rt = R.T
    max_inc = math.radians(pano["max_incidence_deg"])
    yaw0 = math.radians(yaw)
    yaw_span = math.radians(pano["panel_yaw_deg"])
    pitch_span = math.radians(pano["panel_pitch_deg"])
    horizon_y = pano["horizon_fraction"] * ph
    blend = int(pano.get("ground_blend_rows", 28))
    ground_on = pano.get("ground_plane_enabled", 0) != 0
    cam_h = pano.get("camera_height_z", 1.0)
    gz = pano.get("ground_plane_z", 0.0)
    d_near = pano.get("ground_distance_near", 0.35)
    d_far = pano.get("ground_distance_far", 12.0)

    map_x = np.zeros((ph, pw), np.float32)
    map_y = np.zeros((ph, pw), np.float32)
    for v in range(ph):
        phi = (horizon_y - v) / ph * pitch_span
        for u in range(pw):
            theta = yaw0 + ((u + 0.5) / pw - 0.5) * yaw_span
            if ground_on and v > horizon_y - blend:
                gfrac = max(0.0, min(1.0, (v - horizon_y) / max(1.0, ph - horizon_y)))
                rng = d_near + gfrac * (d_far - d_near)
                gx, gy = rng * math.cos(theta), rng * math.sin(theta)
                dx, dy, dz = gx, gy, gz - cam_h
                ln = math.hypot(dx, math.hypot(dy, dz))
                rg = np.array([dx / ln, dy / ln, dz / ln])
                rs = np.array([
                    math.cos(phi) * math.cos(theta),
                    math.cos(phi) * math.sin(theta),
                    math.sin(phi),
                ])
                t = 1.0
                if v < horizon_y + blend:
                    t = (v - (horizon_y - blend)) / max(1.0, 2.0 * blend)
                    t = max(0.0, min(1.0, t))
                rv = (1 - t) * rs + t * rg
                rv /= np.linalg.norm(rv)
            else:
                rv = np.array([
                    math.cos(phi) * math.cos(theta),
                    math.cos(phi) * math.sin(theta),
                    math.sin(phi),
                ])
            ray_c = Rt @ rv
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


def seam_score(a, b, overlap):
    ga = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY).astype(np.float32)
    gb = cv2.cvtColor(b, cv2.COLOR_BGR2GRAY).astype(np.float32)
    ra, lb = ga[:, -overlap:], gb[:, :overlap]
    m = (ra > 12) & (lb > 12)
    if m.sum() < 400:
        return 1e9
    return float(np.mean((ra - lb)[m] ** 2))


def tune_pair(role_a, role_b, rig, frames, pano):
    overlap = pano["overlap_px"]
    best = (1e9, 0.0, 0.0, 1.0)
    for yaw_d in np.arange(-4, 4.01, 2.0):
        for pitch_d in np.arange(-2, 2.01, 2.0):
            for zoom in [1.0, 1.06, 1.1]:
                cam_b = deepcopy(rig["cameras"][role_b])
                cam_b["yaw_offset_deg"] = yaw_d
                cam_b["pitch_offset_deg"] = pitch_d
                cam_b["focal_zoom_out"] = zoom
                pa = build_panel(frames[role_a], rig["cameras"][role_a], pano, scale=0.5)
                pb = build_panel(frames[role_b], cam_b, pano, scale=0.5)
                s = seam_score(pa, pb, overlap)
                if s < best[0]:
                    best = (s, yaw_d, pitch_d, zoom)
    return best


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
    rig_path = root / "config" / "rig.json"
    rig = json.loads(rig_path.read_text())
    pano = rig["panorama"]
    pano["ground_plane_enabled"] = 1
    pano.setdefault("camera_height_z", 1.0)
    pano.setdefault("ground_plane_z", 0.0)
    pano.setdefault("ground_distance_near", 0.35)
    pano.setdefault("ground_distance_far", 12.0)
    pano.setdefault("ground_blend_rows", 28)

    for role in ROLES:
        rig["cameras"][role]["yaw_offset_deg"] = 0.0
        rig["cameras"][role]["pitch_offset_deg"] = 0.0
        rig["cameras"][role]["focal_zoom_out"] = 1.0

    frames = {}
    for role in ROLES:
        cap = cv2.VideoCapture(str(root / f"{role}_1.mp4"))
        _, frames[role] = cap.read()
        cap.release()

    for role_a, role_b in [("left", "front"), ("front", "right"), ("right", "rear")]:
        s, yaw_d, pitch_d, zoom = tune_pair(role_a, role_b, rig, frames, pano)
        rig["cameras"][role_b]["yaw_offset_deg"] = round(yaw_d, 2)
        rig["cameras"][role_b]["pitch_offset_deg"] = round(pitch_d, 2)
        rig["cameras"][role_b]["focal_zoom_out"] = round(zoom, 3)
        print(f"{role_a}-{role_b}: yaw={yaw_d:+.0f} pitch={pitch_d:+.0f} zoom={zoom:.2f} seam={s:.1f}")

    rig["panorama"] = pano
    rig_path.write_text(json.dumps(rig, indent=2) + "\n")
    print("updated", rig_path)


if __name__ == "__main__":
    main()
