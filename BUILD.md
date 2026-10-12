# Build and run (Java middleware)

`VideoStreamingServer` depends on packages under `surround/`, `perception/`, `output/`, etc.  
**Do not** compile or run only `VideoStreamingServer.java` — you will get:

`NoClassDefFoundError: surround/calibration/RigConfigLoader`

Compile production `.java` files into `out/` (excludes `src/test`, `archive/`, `target/`) and run with `out` + OpenCV on the classpath.

**`OPENCV_JAR` is required at compile time** on the integration branch (surround + perception use OpenCV types).

## Windows (OpenCV 4.9 example)

Your FFmpeg plugin path shows OpenCV lives under something like  
`C:\Users\ps95973\Downloads\opencv\build`.

```bat
cd path\to\fisheye_work
set OPENCV_DIR=C:\Users\ps95973\Downloads\opencv\build
scripts\compile.bat
scripts\run.bat
```

Then open: http://localhost:9090/play

Signals (curb + vehicle): http://localhost:9090/api/signals  
WebSocket signals (recommended for overlay): `ws://localhost:9091/signals`  

**Curb is not drawn on `/stitch`** (Android overlay). For local testing use:  
http://localhost:9090/play (split view) or http://localhost:9090/debug/curb/right  

Trigger a turn scenario (sim mode): `POST http://localhost:9090/api/vehicle/simulate?scenario=RIGHT_TURN`

Real vehicle / phone UDP ingress (16-byte LE steering + heading):

```bash
java -cp "out:$OPENCV_JAR" VideoStreamingServer --vehicle-pose-mode udp --udp-listen-port 45454
```

Optional mirror to Android phone IP: add `--udp-target 192.168.1.50 --udp-port 45454`

If your jar is not named `opencv-490.jar`, adjust `OPENCV_DIR` or rename/copy the jar under `%OPENCV_DIR%\java\`.

### Manual commands

```bat
set OPENCV_DIR=C:\Users\ps95973\Downloads\opencv\build
set OPENCV_JAR=%OPENCV_DIR%\java\opencv-490.jar
mkdir out
powershell -Command "javac -encoding UTF-8 -cp '%OPENCV_JAR%' -d out (Get-ChildItem -Recurse -Filter *.java | Where-Object { $_.FullName -notmatch '\\out\\' }).FullName"
set PATH=%OPENCV_DIR%\bin;%PATH%
java -cp "out;%OPENCV_JAR%" -Djava.library.path="%OPENCV_DIR%\java\x64;%OPENCV_DIR%\bin" VideoStreamingServer
```

## Maven (`pom.xml`) — recommended

Downloads OpenCV Java (`org.openpnp:opencv`) from Maven Central; no local `OPENCV_JAR` needed for **compile**.

```bash
mvn -q -DskipTests compile
mvn -q exec:java -Dexec.mainClass=com.middleware.panorama.VideoStreamingServer -Dexec.args=9090
```

Or:

```bash
make mvn-compile
make mvn-run PORT=9090
```

Use your own OpenCV jar instead:

```bash
mvn -Plocal-opencv -Dopencv.jar.path=/path/to/opencv-490.jar -DskipTests compile
```

Tests: `mvn test` or `make mvn-test`

## Makefile

```bash
make help
make mvn-compile
make mvn-run
```

Script-based build (local OpenCV jar required at compile time):

## Linux / macOS

```bash
export OPENCV_JAR=/path/to/opencv/java/opencv-490.jar
export JAVA_LIBRARY_PATH=/path/to/opencv/lib
./scripts/compile.sh
java -cp "out:$OPENCV_JAR" -Djava.library.path="$JAVA_LIBRARY_PATH" com.middleware.panorama.VideoStreamingServer
```

## Inputs

- Four clips in repo root: `left_1.mp4`, `front_1.mp4`, `right_1.mp4`, `rear_1.mp4` (from `main`)
- `config/rig.json` (run `python3 calibration/synthetic_calibrate.py .` if missing)
- `config/curb.json` for `/api/curb` (optional; copied on curb branch)
