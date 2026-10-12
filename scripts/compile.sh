#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/out"

if [[ -z "${OPENCV_JAR:-}" ]]; then
  echo "ERROR: Set OPENCV_JAR to your OpenCV Java jar (e.g. opencv-490.jar)." >&2
  echo "  export OPENCV_JAR=/path/to/opencv/java/opencv-490.jar" >&2
  echo "  export JAVA_LIBRARY_PATH=/path/to/opencv/lib" >&2
  exit 1
fi
if [[ ! -f "$OPENCV_JAR" ]]; then
  echo "ERROR: OPENCV_JAR not found: $OPENCV_JAR" >&2
  exit 1
fi

rm -rf "$OUT"
mkdir -p "$OUT"

mapfile -t SOURCES < <(
  find "$ROOT" -name '*.java' \
    ! -path '*/out/*' \
    ! -path '*/target/*' \
    ! -path '*/archive/*' \
    ! -path '*/src/test/*' \
    ! -path '*/tests/*'
)

if [[ ${#SOURCES[@]} -eq 0 ]]; then
  echo "ERROR: No .java sources found under $ROOT" >&2
  exit 1
fi

echo "Compiling ${#SOURCES[@]} sources with $OPENCV_JAR ..."
javac -encoding UTF-8 -cp "$OPENCV_JAR" -d "$OUT" "${SOURCES[@]}"

echo "OK: classes in $OUT"
echo "WP5:  java -cp out:\$OPENCV_JAR -Djava.library.path=\$JAVA_LIBRARY_PATH com.middleware.panorama.VideoStreamingServer"
echo "Rig:  java -cp out:\$OPENCV_JAR -Djava.library.path=\$JAVA_LIBRARY_PATH KannalaSurroundServer"
