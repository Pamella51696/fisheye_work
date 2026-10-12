#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/out"
if [[ ! -d "$OUT" ]]; then
  echo "Run scripts/compile.sh first" >&2
  exit 1
fi
if [[ -z "${OPENCV_JAR:-}" ]]; then
  echo "Set OPENCV_JAR to opencv-490.jar (see BUILD.md)" >&2
  exit 1
fi
exec java -cp "$OUT:$OPENCV_JAR" \
  ${JAVA_LIBRARY_PATH:+-Djava.library.path="$JAVA_LIBRARY_PATH"} \
  VideoStreamingServer "$@"
