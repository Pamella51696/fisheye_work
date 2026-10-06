# Curb-side detection (perception layer)

Runs **on top of** the Kannala–Brandt / surround fisheye middleware. Video clips stay on `main` (`left_1.mp4`, …); runtime rig + curb config live on this branch.

## Pipeline

```
Fisheye frame (per camera)
    → SurroundPipeline.rectify()   (KB / equidistant → rectilinear)
    → CurbDetector                 (classical CV today; ML-ready interface)
    → CurbTracker                  (temporal smoothing)
    → CoordinateTransform          (image → vehicle ground plane)
    → CurbDistanceEstimator
    → CurbZoneClassifier           (GREEN / YELLOW / RED, configurable meters)
    → CurbFusion                   (four-camera vehicle frame)
    → CurbDetectionResult / FusedCurbResult JSON  →  Android overlay (external)
```

No overlay or UI instructions are emitted—only geometry, distance, zone, and confidence.

## Coordinate systems

| Space | Convention |
|-------|------------|
| **IMAGE** | Rectilinear pixels `(x, y)`, origin top-left |
| **CAMERA** | OpenCV camera frame (X right, Y down, Z forward) |
| **VEHICLE** | X forward, Y right, Z up; ground at `ground_plane_z` |
| **ANDROID** | App maps IMAGE/VEHICLE points to its view (not done in middleware) |

## Configuration

- `config/rig.json` — camera intrinsics/extrinsics (from synthetic or checkerboard calibration)
- `config/curb.json` — `red_threshold_meters`, `yellow_threshold_meters`, `min_confidence`, tracking/hysteresis, ground height

## Modules

| Path | Role |
|------|------|
| `perception/curb/` | Detector, tracker, distance, zones, fusion, pipeline |
| `geometry/CoordinateTransform.java` | Ground intersection / vehicle points |
| `configuration/CurbConfig.java` | Threshold loader |
| `output/` | `CurbDetectionResult`, `FusedCurbResult`, JSON serializer |

## HTTP API (simulation server)

- `GET /api/curb` — newline-delimited JSON (`FusedCurbResult` per frame)
- `GET /api/curb/left|front|right|rear` — single-camera `CurbDetectionResult` stream

## Testing

- **Unit:** `tests/CurbGeometryTest.java` (ground range, zone thresholds) — run with `java` after compile
- **Integration:** `VideoStreamingServer` + AI fisheye clips; tune `config/curb.json` thresholds
- **Future:** replace `ClassicalCurbDetector` with segmentation model behind `CurbDetector`

## Phases delivered

1. Detection + confidence (classical)
2. Tracking + hysteresis
3. Vehicle-ground distance
4. Zone classification (per-segment)
5. Four-camera fusion (vehicle coordinates)
6. Middleware JSON API
