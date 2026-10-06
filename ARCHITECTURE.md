# Surround fisheye system

## Modes

| Mode | When | Input |
|------|------|--------|
| `synthetic` | AI-generated clips (now) | `calibration/synthetic_calibrate.py` |
| `checkerboard` | Real cameras (future) | Charuco/checkerboard → same `config/rig.json` |

Runtime always loads **`config/rig.json`** — swap parameters, not code.

## Pipeline

```
Fisheye frame → FisheyeRay (K,D) → 3D ray → R,T → vehicle frame
    → PanoramaMapper → feather blend → panorama

Parallel path: Fisheye → RectilinearMapper → corrected feed (/corrected/<role>)
```

Stitching never goes through rectilinear images.

**Ground plane:** experimental only — keep `ground_plane_enabled: 0` for AI fisheye (panel ground warp smears). Spherical rays + feather blend is the stable path.

## Layout

- `surround/calibration/` — rig config loader
- `surround/geometry/` — ray projection
- `surround/stitching/` — panorama maps + blender
- `surround/undistort/` — rectilinear debug output
- `surround/runtime/SurroundPipeline.java` — runtime entry
- `calibration/` — Python synthetic fitting (offline)

## Regenerate synthetic calibration

```bash
./calibration/run_synthetic.sh
```

Place `left_1.mp4`, `front_1.mp4`, `right_1.mp4`, `rear_1.mp4` in the repo root (as on `main`).
