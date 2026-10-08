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

**Ground plane:** keep `ground_plane_enabled: 0` (panel ground warp smears AI feeds).

**Partial undistort for stitch:** `partial_undistort_for_stitch: 1` with `stitch_undistort_balance` (~0.35–0.45) straightens bend before panorama mapping; increase balance for straighter lines, decrease to keep more fisheye FOV.

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
<<<<<<< HEAD
=======

## Curb perception (this branch)

Optional layer on rectified fisheye feeds — see **`CURB_ARCHITECTURE.md`**.

```
Fisheye → rectify → curb detect/track → vehicle distance → GREEN/YELLOW/RED → JSON API
```

Config: `config/curb.json`. Signals: `/api/signals` (curb + vehicle); video unchanged on `/stitch`.

Async layout: `CameraCaptureManager` → `FrameDistributor` → video **and** `CurbDetectionService` (see `CURB_ARCHITECTURE.md`).

## Build / run

See **`BUILD.md`**. On Windows use `scripts\compile.bat` then `scripts\run.bat` so `surround/**` classes are on the classpath.
>>>>>>> 9ef2ef3 (Add Windows build scripts and document full classpath compile)
