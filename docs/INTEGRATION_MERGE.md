# Integrated middleware (`cursor/integration-merge-7e98`)

This branch merges:

| Source branch | What was merged |
|---------------|-----------------|
| `upd-inetgration` | WP5 viewer, SSE, panorama stitch, `Wp5Simulator` |
| `cursor/wp5-signal-source-switch-7e98` | `wp5.signalSource` UDP / SIMULATOR / AUTO |
| `cursor/analytics-architecture-7e98` | `surround/*`, `perception/curb/*`, rig + camera curb |

## Which main class to run

| Entry point | Use when |
|-------------|----------|
| **`com.middleware.panorama.VideoStreamingServer`** | **Default:** `/wp5/viewer`, WP5 SSE, equidistant stitch, UDP/sim switch |
| **`KannalaSurroundServer`** | Surround rig pipeline, `/corrected/*`, WebSocket `:9091`, legacy JSON `/api/signals` |

## WP5 + camera curb (merged path)

```bash
# From repo root with left_1.mp4 … rear_1.mp4 and config/rig.json + config/curb.json
java -Dwp5.cameraCurb=true \
     -Dwp5.signalSource=AUTO \
     -cp "out:opencv.jar" \
     com.middleware.panorama.VideoStreamingServer 9090
```

- Curb zones on SSE come from **camera perception**, not distance sliders.
- Sliders in `/wp5/viewer` still POST ingest for **dev only** when camera curb is off.
- Vehicle steering: `-Dwp5.signalSource=SIMULATOR|UDP|AUTO` (see `docs/WP5_SIGNAL_SOURCE.md`).

## Build

**Requires OpenCV Java jar on the compile classpath** (not optional for this branch).

Windows:

```bat
set OPENCV_DIR=C:\path\to\opencv\build
scripts\compile.bat
```

Linux / macOS:

```bash
export OPENCV_JAR=/path/to/opencv/java/opencv-490.jar
export JAVA_LIBRARY_PATH=/path/to/opencv/lib
./scripts/compile.sh
```

`compile.sh` / `compile.bat` skip `src/test`, `archive/`, and `target/` (no JUnit on compile path).

Run WP5 server:

```bash
java -cp "out:$OPENCV_JAR" -Djava.library.path="$JAVA_LIBRARY_PATH" \
  com.middleware.panorama.VideoStreamingServer
```
