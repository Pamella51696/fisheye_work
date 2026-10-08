# Curb-side detection (perception layer)

Runs **asynchronously** on top of the Kannala–Brandt / surround fisheye middleware.  
**Video never waits on curb detection** — see `CameraCaptureManager` + `FrameDistributor`.

## Domains

| Domain | Responsibility |
|--------|----------------|
| **Video** | `CameraCaptureManager` → `SurroundPipeline` → `/stitch`, `/corrected/*` |
| **Curb analytics** | `CurbDetectionService` (worker thread, capacity-1 frame queue) |
| **Vehicle signals** | `UdpVehicleSignalService` + `VehicleSignalSimulator` (UDP later) |
| **Android signals** | `SignalPublisher` → `/api/signals` |

## Pipeline

```
Fisheye frames (single capture)
    → FrameDistributor ──┬──► video consumers (MJPEG)
                         └──► analytics queue (latest only)
                                    │
                                    ▼
                         CurbDetectionService
                                    │
                    rectify → detect → distance → zones → filter
                                    │
                                    ▼
                              CurbState → /api/signals
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

| Endpoint | Purpose |
|----------|---------|
| `GET /api/signals` | Latest unified JSON (`curb` + `vehicle`) |
| `GET /api/signals/stream` | Filtered NDJSON (~10 Hz) |
| `POST /api/vehicle/simulate?scenario=RIGHT_TURN` | Start steering scenario |

Video remains on `/stitch` and `/corrected/<role>` (separate channel).

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
