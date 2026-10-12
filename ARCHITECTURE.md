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

## Middleware layout (video + analytics)

Video is authoritative and synchronous; curb analytics and vehicle signals are asynchronous consumers.

```
CameraCaptureManager → FrameDistributor ──┬──► MJPEG (/stitch, /corrected/*)
                                          └──► CurbDetectionService (capacity-1 queue)
                                                    │
SignalPublisher ◄── CurbState + VehicleSignalSimulator
       │
       └── /api/signals, /api/signals/stream
```

See **`CURB_ARCHITECTURE.md`** and **`MIDDLEWARE_ARCHITECTURE.md`** for endpoint and failure-isolation details.

## Layout

- `surround/calibration/` — rig config loader
- `surround/geometry/` — ray projection
- `surround/stitching/` — panorama maps + blender
- `surround/undistort/` — rectilinear debug output
- `surround/runtime/` — `SurroundPipeline`, `FrameDistributor`, `CameraCaptureManager`
- `surround/analytics/` — async curb worker + result filter
- `surround/signal/` — unified JSON signal API
- `surround/vehicle/` — steering/yaw simulator (replace with real bus later)
- `perception/curb/` — detector, distance, zones, fusion
- `calibration/` — Python synthetic fitting (offline)

## Regenerate synthetic calibration

```bash
./calibration/run_synthetic.sh
```

Place `left_1.mp4`, `front_1.mp4`, `right_1.mp4`, `rear_1.mp4` in the repo root (as on `main`).

## Build / run

See **`BUILD.md`**. Use `scripts/compile.sh` (Linux) or `scripts\compile.bat` (Windows), then `scripts/run.sh` or `scripts\run.bat`.
