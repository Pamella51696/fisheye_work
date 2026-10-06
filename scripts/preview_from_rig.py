#!/usr/bin/env python3
"""Preview panorama using config/rig.json (same math as Java SurroundPipeline)."""

import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "calibration"))
from optimize_extrinsics import build_panel, ROLES  # noqa: E402


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
    rig = json.loads((root / "config" / "rig.json").read_text())
    pano = rig["panorama"]
    panels = []
    for role in rig["camera_order"]:
        cap = cv2.VideoCapture(str(root / f"{role}_1.mp4"))
        _, frame = cap.read()
        cap.release()
        cam = rig["cameras"][role]
        panels.append(build_panel(frame, cam, pano))
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
