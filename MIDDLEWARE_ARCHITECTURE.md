# Middleware application architecture

This document describes how **video**, **curb analytics**, and **vehicle signals** fit together on the `cursor/kannala-brandt-undistort-c4a0` line. The design rule is:

> **Video is the primary pipeline. Curb detection is a consumer of video, not a dependency of video.**

If curb detection is slow, crashes, or leaks memory, MJPEG streams (`/stitch`, `/corrected/*`) keep running. Android sees `curb.status: "UNAVAILABLE"` instead of losing video.

## System diagram

```text
                         MIDDLEWARE APPLICATION
┌───────────────────────────────────────────────────────────────┐
│                                                               │
│                  VideoStreamingServer                         │
│                         │                                     │
│              CameraCaptureManager                             │
│                         │                                     │
│                    Raw Frames                                 │
│                         │                                     │
│              ┌──────────┴──────────┐                          │
│              │   FrameDistributor   │                          │
│              └──────────┬──────────┘                          │
│              ┌──────────┴──────────┐                          │
│              ▼                     ▼                          │
│      VIDEO PIPELINE          ANALYTICS PIPELINE               │
│              │                     │                          │
│              │              capacity-1 queue                   │
│              │                     │                          │
│              ▼                     ▼                          │
│      SurroundPipeline       CurbDetectionService              │
│              │                     │                          │
│       ┌──────┴──────┐       rectify → detect → distance      │
│       │             │              → zones → filter           │
│       ▼             ▼                     │                   │
│  /corrected/*    /stitch                 ▼                   │
│                                    CurbState                  │
│                                              │                │
│  VehicleSignalSimulator ◄── POST /api/vehicle/simulate         │
│              │                              │                │
│              └──────────┬───────────────────┘                │
│                         ▼                                     │
│                  SignalPublisher                              │
│                         │                                     │
│              GET /api/signals  |  /api/signals/stream         │
└─────────────────────────┼─────────────────────────────────────┘
                          │
                   Android client
              ┌───────────┴───────────┐
              ▼                       ▼
        MJPEG video            JSON signals
     (separate HTTP)      (poll or NDJSON stream)
```

## What not to do

Do **not** call curb detection inside the MJPEG encode loop:

```java
// DON'T: curb latency or failures affect every viewer
while (true) {
    readFrames();
    pipeline.rectify(...);
    curbDetector.detect(rect);  // blocks video
    pipeline.stitchPanorama(...);
}
```

Instead, `CameraCaptureManager` publishes synchronized fisheye frames once; `FrameDistributor` fans out to video waiters and a **latest-frame-only** analytics queue.

## Core components

| Component | Package | Role |
|-----------|---------|------|
| `SurroundPipeline` | `surround/runtime` | Kannala–Brandt / equidistant rectify + panorama stitch |
| `CameraCaptureManager` | `surround/runtime` | Single capture thread for all four clips |
| `FrameDistributor` | `surround/runtime` | Latest frame for video; `ArrayBlockingQueue(1)` for analytics |
| `FrameSnapshot` | `surround/runtime` | Timestamp + `Mat[]` with explicit `release()` |
| `CurbDetectionService` | `surround/analytics` | Daemon worker; `catch (Throwable)` isolates failures |
| `CurbPerceptionPipeline` | `perception/curb` | Rectify → detect → distance → zones → fusion |
| `CurbResultFilter` | `surround/analytics` | Zone change / distance delta / heartbeat to Android |
| `VehicleSignalSimulator` | `surround/vehicle` | Event-driven smooth steering + bicycle-model yaw |
| `SignalPublisher` | `surround/signal` | Unified JSON for Android |

Perception details: **`CURB_ARCHITECTURE.md`**.

## Frame distributor behavior

When analytics falls behind:

```text
Video:  101 → 102 → 103 → 104 → 105 → ...
Curb:   101 ──────► 104 ──────► 106 → ...
```

Stale analytics frames are dropped and released — the queue never grows without bound.

## Curb inputs

The detector consumes **per-camera rectified** images (`left`, `front`, `right`; rear optional), not the stitched panorama. That matches:

```java
pipeline.rectify(cameraIndex, frames[cameraIndex], rect);
```

inside `CurbPerceptionPipeline`, using the same intrinsics as `/corrected/<role>`.

## Zone thresholds

Configured in `config/curb.json` (distance in meters):

| Zone | Typical rule |
|------|----------------|
| GREEN | distance > green threshold |
| YELLOW | between yellow and green |
| RED | at or below red threshold |

Tune without recompiling.

## Android-facing JSON

Example from `GET /api/signals`:

```json
{
  "timestamp": 1791430000123,
  "vehicle": {
    "steeringAngleDeg": -14.5,
    "yawAngleDeg": 2.3,
    "velocityMps": 2.0
  },
  "curb": {
    "detected": true,
    "distanceMeters": 0.72,
    "zone": "YELLOW",
    "confidence": 0.91,
    "camera": "right"
  }
}
```

When curb analytics is down:

```json
{ "curb": { "status": "UNAVAILABLE" } }
```

**Rate limiting:** `CurbResultFilter` reduces spam; `SignalPublisher` applies a steering deadband (~1°). Use `/api/signals/stream` for ~10 Hz NDJSON during development. A dedicated WebSocket port can replace the stream later without changing field names.

## Two channels for Android

| Channel | Protocol | Endpoints |
|---------|----------|-----------|
| Video | HTTP MJPEG | `/stitch`, `/corrected/left`, … |
| Signals | HTTP JSON | `/api/signals`, `/api/signals/stream` |
| Dev overlay | HTTP MJPEG | `/debug/curb/<role>` (not for production UI) |

Trigger a steering scenario:

```http
POST /api/vehicle/simulate?scenario=LEFT_TURN
```

## Startup (`VideoStreamingServer.main`)

```text
load OpenCV → SurroundPipeline.load(rig.json)
→ FrameDistributor + CameraCaptureManager.start()
→ CurbDetectionService.start()  (optional if curb.json missing)
→ VehicleSignalSimulator + UdpVehicleSignalService.start()
→ HttpServer: video + signals + play page
```

None of the analytics services own the video server; they are siblings.

## Calibration note (Kannala–Brandt)

The repo implements Kannala–Brandt in `fisheye/` and `FisheyeRay`, but **`config/rig.json` may still declare `EQUIDISTANT`** until real KB coefficients are fitted. Curb distance estimation depends on correct intrinsics — update `model` and `k1`–`k4` after checkerboard/Charuco calibration before trusting meter distances.

## Future: single capture authority

Today each MJPEG client waits on `FrameDistributor` (one capture thread). A later step is `CameraCaptureManager` as the only reader with optional recording — no change required to Android APIs.

## Build and run

See **`BUILD.md`**. Entry point: `VideoStreamingServer`.
